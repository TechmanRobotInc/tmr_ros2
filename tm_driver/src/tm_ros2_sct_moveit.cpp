#include "tm_driver/tm_ros2_sct_moveit.h"

TmSctRos2::TmSctRos2(rclcpp::Node::SharedPtr node, TmDriver &iface, bool is_fake)
    : node(node)
    , sct_(iface.sct)
    , svr_(iface.svr)
    , iface_(iface)
    , state_(iface.state)
    , is_fake_(is_fake)
{
  if (!is_fake_) {
    sct_.start_tm_sct(5000);
  }

  jns_.clear();
  jns_.push_back("joint_1");
  jns_.push_back("joint_2");
  jns_.push_back("joint_3");
  jns_.push_back("joint_4");
  jns_.push_back("joint_5");
  jns_.push_back("joint_6");

  if (!is_fake_) {
    sm_.sct_pub = node->create_publisher<tm_msgs::msg::SctResponse>("sct_response", 1);
    sm_.sta_pub = node->create_publisher<tm_msgs::msg::StaResponse>("sta_response", 1);
  }

    sta_updated_ = false;

    sct_reconnect_timeout_ms_ = 1000;
    sct_reconnect_timeval_ms_ = 3000;
    checkListenNodeThread = std::thread(std::bind(&TmSctRos2::check_is_on_listen_node, this));

  if (!is_fake_) {
    sct_thread_ = std::thread(std::bind(&TmSctRos2::sct_responsor, this));


    connect_tm_srv_ = node->create_service<tm_msgs::srv::ConnectTM>(
        "connect_tmsct", std::bind(&TmSctRos2::connect_tmsct, this,
        std::placeholders::_1, std::placeholders::_2));

    send_script_srv_ = node->create_service<tm_msgs::srv::SendScript>(
        "send_script", std::bind(&TmSctRos2::send_script, this,
        std::placeholders::_1, std::placeholders::_2));
        
    set_event_srv_ = node->create_service<tm_msgs::srv::SetEvent>(
        "set_event", std::bind(&TmSctRos2::set_event, this,
        std::placeholders::_1, std::placeholders::_2));
        
    set_io_srv_ = node->create_service<tm_msgs::srv::SetIO>(
        "set_io", std::bind(&TmSctRos2::set_io, this,
        std::placeholders::_1, std::placeholders::_2));
        
    set_positions_srv_ = node->create_service<tm_msgs::srv::SetPositions>(
        "set_positions", std::bind(&TmSctRos2::set_positions, this,
        std::placeholders::_1, std::placeholders::_2));

    ask_sta_srv_ = node->create_service<tm_msgs::srv::AskSta>(
        "ask_sta", std::bind(&TmSctRos2::ask_sta, this,
        std::placeholders::_1, std::placeholders::_2));
  }

  as_ = rclcpp_action::create_server<control_msgs::action::FollowJointTrajectory>(
    node->get_node_base_interface(),
    node->get_node_clock_interface(),
    node->get_node_logging_interface(),
    node->get_node_waitables_interface(),
    "tmr_arm_controller/follow_joint_trajectory",
    std::bind(&TmSctRos2::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&TmSctRos2::handle_cancel, this, std::placeholders::_1),
    std::bind(&TmSctRos2::handle_accepted, this, std::placeholders::_1)
  );
  goal_id_.clear();
  has_goal_ = false;
}

TmSctRos2::~TmSctRos2()
{
    print_info("TM_ROS: (Listen node) halt");		

    if (sct_thread_.joinable()) {
      sct_thread_.join();
    }

    if (is_fake_) return;

    sta_updated_ = true;
    sta_cv_.notify_all();
    if (sct_.is_connected()) {}
    sct_.halt();
}

void TmSctRos2::check_is_on_listen_node_from_script(std::string id, std::string script){
    std::string idzero = "0";
    std::string ok = "OK";
    std::string errorString = "ERROR";
    if(idzero.compare(id)==0 && errorString.compare(script)!=0 &&  ok.compare(script)!=0){
        iface_.back_to_listen_node();
    }
}

void TmSctRos2::sct_msg()
{
    SctAndStaMsg &sm = sm_;
    TmSctData &data = sct_.sct_data;

    sm.sct_msg.id = data.script_id();
    sm.sct_msg.script = std::string{ data.script(), data.script_len() };

    check_is_on_listen_node_from_script(sm.sct_msg.id, sm.sct_msg.script);

    if (data.sct_has_error()) {
        print_error("TM_ROS: (TM_SCT): MSG: (%s): %s", sm.sct_msg.id.c_str(), sm.sct_msg.script.c_str());
        print_error("TM_ROS: (TM_SCT): ROS Node Data Error %d", (int)data.sct_has_error());
    }
    else {
        print_info("TM_ROS: (TM_SCT): MSG: (%s): %s", sm.sct_msg.id.c_str(), sm.sct_msg.script.c_str());
    }

    sm.sct_msg.header.stamp = node->rclcpp::Node::now();
    sm.sct_pub->publish(sm.sct_msg);
}

void TmSctRos2::sta_msg()
{
    SctAndStaMsg &sm = sm_;
    TmStaData &data = sct_.sta_data;
    {
        std::lock_guard<std::mutex> lck(sta_mtx_);
        sm.sta_msg.subcmd = data.subcmd_str();
        sm.sta_msg.subdata = std::string{ data.subdata(), data.subdata_len() };
        sta_updated_ = true;
    }
    sta_cv_.notify_all();

    print_info("TM_ROS: (TM_STA): res: (%s): %s", sm.sta_msg.subcmd.c_str(), sm.sta_msg.subdata.c_str());

    sm.sta_msg.header.stamp = node->rclcpp::Node::now();
    sm.sta_pub->publish(sm.sta_msg);
}

bool TmSctRos2::sct_func()
{
    TmSctCommunication &sct = sct_;
    int n;
    firstCheckIsOnListenNodeCondVar.notify_one();
    
    auto rc = sct.recv_spin_once(1000, &n);
    if (rc == TmCommRC::ERR ||
        rc == TmCommRC::NOTREADY ||
        rc == TmCommRC::NOTCONNECT) {
        return false;
    }
    else if (rc != TmCommRC::OK) {
        return true;
    }
    std::vector<TmPacket> &pack_vec = sct.packet_list();

    for (auto &pack : pack_vec) {
        switch (pack.type) {
        case TmPacket::Header::CPERR:
            sct.tmSctErrData.set_CPError(pack.data.data(), pack.data.size());
            print_error("TM_ROS: (Listen node) ROS Node Header CPERR %d",(int)sct.tmSctErrData.error_code());
            break;

        case TmPacket::Header::TMSCT:

            sct.tmSctErrData.error_code(TmCPError::Code::Ok);

            //TODO ? lock and copy for service response
            TmSctData::build_TmSctData(sct.sct_data, pack.data.data(), pack.data.size(), TmSctData::SrcType::Shallow);

            sct_msg();
            break;

        case TmPacket::Header::TMSTA:

            sct.tmSctErrData.error_code(TmCPError::Code::Ok);

            TmStaData::build_TmStaData(sct.sta_data, pack.data.data(), pack.data.size(), TmStaData::SrcType::Shallow);

            sta_msg();
            break;

        default:
            print_error("TM_ROS: (Listen node): invalid header");
            break;
        }
    }
    return true;
}

void TmSctRos2::check_is_on_listen_node(){
    std::unique_lock<std::mutex> firstCheckIsOnListenNodeLock(firstCheckIsOnListenNodeMutex);
    std::unique_lock<std::mutex> checkIsOnListenNodeLock(checkIsOnListenNodeMutex);
    
    firstCheckIsOnListenNodeCondVar.wait(firstCheckIsOnListenNodeLock);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));

    while (rclcpp::ok()){
        std::string reSubcmd;
        std::string reSubdata;
        ask_sta_struct("00","",1,reSubcmd,reSubdata);
        bool isInListenNode = false;

        std::istringstream(reSubdata) >> std::boolalpha >> isInListenNode;
    
        if(isInListenNode){
            print_info("TM_ROS: On listen node.");
            iface_.back_to_listen_node();
        } else{
            print_info("TM_ROS: Not on listen node!");
        }
        checkIsOnListenNodeCondVar.wait(checkIsOnListenNodeLock);
    }
}

void TmSctRos2::sct_responsor()
{
    TmSctCommunication &sct = sct_;

    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    print_info("TM_ROS: sct_response thread begin");

    while (rclcpp::ok()) {
        //bool reconnect = false;
        if (iface_.get_connect_recovery_guide()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        else   
        {
            if (!sct.recv_init()) {
                print_debug("TM_ROS: (Listen node): is not connected");
            }
            firstEnter = true;
            while (rclcpp::ok() && sct.is_connected() && iface_.svr.is_connected()) {
                if(firstEnter){
                    checkIsOnListenNodeCondVar.notify_one();
                    firstEnter = false;
                }
                if (!sct_func()) break;
            }
            sct.close_socket();
            if (!rclcpp::ok()) break;
            sct_connect_recover();
        }
    }
    checkIsOnListenNodeCondVar.notify_one();
    firstCheckIsOnListenNodeCondVar.notify_one();

    if (sct.is_connected()) {
      sct.send_script_exit();
    }

    sct.close_socket();
    print_info("TM_ROS: sct_response thread end\n");
}

void TmSctRos2::sct_connect_recover()
{
    TmSctCommunication &sct = sct_;
    int timeInterval = 0;
    int lastTimeInterval=1000;
            	
    if (sct_reconnect_timeval_ms_ <= 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    print_info("TM_ROS: (Listen node): Reconnecting...");

    uint64_t startTimeMs = TmCommunication::get_current_time_in_ms();
    while (rclcpp::ok() && timeInterval < sct_reconnect_timeval_ms_) {
        if ( lastTimeInterval/1000 != timeInterval/1000) {
            print_debug("Listen node reconnect remain : %.1f sec...", 0.001 * (sct_reconnect_timeval_ms_ - timeInterval));
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        lastTimeInterval = timeInterval;
        timeInterval = TmCommunication::get_current_time_in_ms() - startTimeMs;
    }
    if (rclcpp::ok() && sct_reconnect_timeval_ms_ >= 0) {
        print_debug("0 sec\nTM_ROS: (Listen node) connect(%d)...", (int)sct_reconnect_timeout_ms_);
        sct.connect_socket("Listen node",sct_reconnect_timeout_ms_);
    }
}

bool TmSctRos2::connect_tmsct(
        const std::shared_ptr<tm_msgs::srv::ConnectTM::Request> req,
        std::shared_ptr<tm_msgs::srv::ConnectTM::Response> res)
{
    bool rb = true;
    int t_o = (int)(1000.0 * req->timeout);
    int t_v = (int)(1000.0 * req->timeval);
    if (req->connect) {
        print_info("TM_ROS: (re)connect(%d) TM_SCT Listen node", (int)t_o);
        sct_.halt();
        rb = sct_.start_tm_sct(t_o);
    }
    if (req->reconnect) {
        if (iface_.get_connect_recovery_guide())
        {        	
            sct_reconnect_timeout_ms_ = 1000;
            sct_reconnect_timeval_ms_ = 3000;
            iface_.set_connect_recovery_guide(false);
            rb = sct_.start_tm_sct(5000);
            print_info("TM_ROS: Listen node resume connection recovery");                     	
        }
        else
        {        	    	
            sct_reconnect_timeout_ms_ = t_o;
            sct_reconnect_timeval_ms_ = t_v;
        }
        print_info("TM_ROS: set Listen node reconnect timeout %dms, timeval %dms", (int)sct_reconnect_timeout_ms_, (int)sct_reconnect_timeval_ms_);
    }
    else {
        // no reconnect
        sct_reconnect_timeval_ms_ = -1;
        print_info("TM_ROS: set Listen node NOT reconnect");
    }
    res->ok = rb;
    return rb;
}

bool TmSctRos2::send_script(
    const std::shared_ptr<tm_msgs::srv::SendScript::Request> req,
    std::shared_ptr<tm_msgs::srv::SendScript::Response> res)
{
    bool rb = (sct_.send_script_str(req->id, req->script) == iface_.RC_OK);
    res->ok = rb;
    return rb;
}

bool TmSctRos2::set_event(
    const std::shared_ptr<tm_msgs::srv::SetEvent::Request> req,
    std::shared_ptr<tm_msgs::srv::SetEvent::Response> res)
{
    bool rb = false;
    std::string content;
    switch (req->func) {
    case tm_msgs::srv::SetEvent_Request::EXIT:
        rb = iface_.script_exit();
        break;
    case tm_msgs::srv::SetEvent_Request::TAG:
        rb = iface_.set_tag((int)(req->arg0), (int)(req->arg1));
        break;
    case tm_msgs::srv::SetEvent_Request::WAIT_TAG:
        rb = iface_.set_wait_tag((int)(req->arg0), (int)(req->arg1));
        break;
    case tm_msgs::srv::SetEvent_Request::STOP:
        rb = iface_.set_stop();
        break;
    case tm_msgs::srv::SetEvent_Request::PAUSE:
        rb = iface_.set_pause();
        break;
    case tm_msgs::srv::SetEvent_Request::RESUME:
        rb = iface_.set_resume();
        break;
    }
    res->ok = rb;
    return rb;
}

bool TmSctRos2::set_io(
    const std::shared_ptr<tm_msgs::srv::SetIO::Request> req,
    std::shared_ptr<tm_msgs::srv::SetIO::Response> res)
{
    bool rb = iface_.set_io(TmIOModule(req->module), TmIOType(req->type), int(req->pin), req->state);
    res->ok = rb;
    return rb;
}

bool TmSctRos2::set_positions(
    const std::shared_ptr<tm_msgs::srv::SetPositions::Request> req,
    std::shared_ptr<tm_msgs::srv::SetPositions::Response> res)
{
    bool rb = false;
    switch(req->motion_type) {
    case tm_msgs::srv::SetPositions_Request::PTP_J:
        rb = iface_.set_joint_pos_PTP(req->positions, req->velocity, req->acc_time, req->blend_percentage, req->fine_goal);
        break;
    case tm_msgs::srv::SetPositions_Request::PTP_T:
        rb = iface_.set_tool_pose_PTP(req->positions, req->velocity, req->acc_time, req->blend_percentage, req->fine_goal);
        break;
    case tm_msgs::srv::SetPositions_Request::LINE_T:
        rb = iface_.set_tool_pose_Line(req->positions, req->velocity, req->acc_time, req->blend_percentage, req->fine_goal);
        break;
    }
    res->ok = rb;
    return rb;
}

bool TmSctRos2::ask_sta_struct(std::string subcmd, std::string subdata, double waitTime,std::string &reSubcmd, std::string &reSubdata){
    SctAndStaMsg &sm = sm_;
    bool rb = false;

    sta_mtx_.lock();
    sta_updated_ = false;
    sta_mtx_.unlock();

    rb = (sct_.send_sta_request(subcmd, subdata) == iface_.RC_OK);

    {
        std::unique_lock<std::mutex> lck(sta_mtx_);
        if (rb && waitTime > 0.0) {
            if (!sta_updated_) {
                sta_cv_.wait_for(lck, std::chrono::duration<double>(waitTime));
            }
            if (!sta_updated_) {
                rb = false;
            }
            reSubcmd = sm.sta_msg.subcmd;
            reSubdata = sm.sta_msg.subdata;
        }
        sta_updated_ = false;
    }

    return rb;
}

bool TmSctRos2::ask_sta(
        const std::shared_ptr<tm_msgs::srv::AskSta::Request> req,
        std::shared_ptr<tm_msgs::srv::AskSta::Response> res){
    res->ok = ask_sta_struct(req->subcmd, req->subdata, req->wait_time, res->subcmd, res->subdata);
    return res->ok;
}

rclcpp_action::GoalResponse TmSctRos2::handle_goal(const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Goal> goal)
{
  auto goal_id = rclcpp_action::to_string(uuid);
  print_info("Received new action goal %s", goal_id.c_str());
  //RCLCPP_INFO_STREAM(node->get_logger(), "Received new action goal " << goal_id);

  if (has_goal_) {
    return rclcpp_action::GoalResponse::ACCEPT_AND_DEFER;
  }

  if (!is_fake_) {
    if (!svr_.is_connected()) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (!sct_.is_connected()) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (state_.has_error()) {
      return rclcpp_action::GoalResponse::REJECT;
    }
  }

  if (!has_points(goal->trajectory)) {
    return rclcpp_action::GoalResponse::REJECT;
  }
  //for (auto &jn : goal->trajectory.joint_names) { tmr_DEBUG_STREAM(jn); }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}
rclcpp_action::CancelResponse TmSctRos2::handle_cancel(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle)
{
  auto goal_id = rclcpp_action::to_string(goal_handle->get_goal_id());
  print_info("Got request to cancel goal %s", goal_id.c_str());
  //RCLCPP_INFO_STREAM(node->get_logger(), "Got request to cancel goal " << goal_id);
  {
    std::lock_guard<std::mutex> lck(as_mtx_);
    if (goal_id_.compare(goal_id) == 0 && has_goal_) {
      has_goal_ = false;
      iface_.stop_pvt_traj();
    }
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}
void TmSctRos2::handle_accepted(
  std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle)
{
  {
    std::unique_lock<std::mutex> lck(as_mtx_);
    goal_id_ = rclcpp_action::to_string(goal_handle->get_goal_id());
    has_goal_ = true;
  }
  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread{std::bind(&TmSctRos2::execute_traj, this, std::placeholders::_1), goal_handle}.detach();

}

void TmSctRos2::execute_traj(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle)
{
  print_info("TM_ROS: trajectory thread begin");

  auto result = std::make_shared<control_msgs::action::FollowJointTrajectory::Result>();

  //actually, no need to reorder
  //std::vector<trajectory_msgs::msg::JointTrajectoryPoint> traj_points;
  //reorder_traj_joints(traj_points, goal_handle->get_goal()->trajectory);
  auto &traj_points = goal_handle->get_goal()->trajectory.points;

  if (!is_positions_match(traj_points.front(), 0.01)) {
    result->error_code = result->PATH_TOLERANCE_VIOLATED;
    result->error_string = "Start point doesn't match current pose";
    print_warn(result->error_string.c_str());
    //RCLCPP_WARN_STREAM(node->get_logger(), result->error_string);

    //goal_handle->abort(result);
  }

  auto pvts = get_pvt_traj(traj_points, 0.025);
  print_info("TM_ROS: traj. total time:= %d", (int)pvts->total_time);
  //RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),"TM_ROS: traj. total time:=" << pvts->total_time);

  if (!goal_handle->is_executing()) {
    goal_handle->execute();
    print_info("goal_handle->execute()");
  }

  if (!is_fake_) {
    iface_.run_pvt_traj(*pvts);
  }
  else {
    iface_.fake_run_pvt_traj(*pvts);
  }
  if (rclcpp::ok()) {
    if (!is_positions_match(traj_points.back(), 0.01)) {
      result->error_code = result->GOAL_TOLERANCE_VIOLATED;
      result->error_string = "Current pose doesn't match Goal point";
      print_warn(result->error_string.c_str());
      //RCLCPP_WARN_STREAM(node->get_logger(), result->error_string);
    }
    else {
      result->error_code = result->SUCCESSFUL;
      result->error_string = "Goal reached, success!";
      print_info(result->error_string.c_str());
      //RCLCPP_INFO_STREAM(node->get_logger(), result->error_string);
    }
    goal_handle->succeed(result);
  }
  {
    std::lock_guard<std::mutex> lck(as_mtx_);
    goal_id_.clear();
    has_goal_ = false;
  }
  print_info("TM_ROS: trajectory thread end");
}

void TmSctRos2::reorder_traj_joints(
  std::vector<trajectory_msgs::msg::JointTrajectoryPoint> &new_traj_points,
  const trajectory_msgs::msg::JointTrajectory& traj)
{
  /* Reorders trajectory - destructive */
  std::vector<size_t> mapping;
  mapping.resize(jns_.size(), jns_.size());
  for (size_t i = 0; i < traj.joint_names.size(); ++i) {
    for (size_t j = 0; j < jns_.size(); ++j) {
      if (traj.joint_names[i] == jns_[j])
        mapping[j] = i;
    }
  }
  new_traj_points.clear();
  for (unsigned int i = 0; i < traj.points.size(); ++i) {
    trajectory_msgs::msg::JointTrajectoryPoint new_point;
    for (unsigned int j = 0; j < traj.points[i].positions.size(); ++j) {
      new_point.positions.push_back(traj.points[i].positions[mapping[j]]);
      new_point.velocities.push_back(traj.points[i].velocities[mapping[j]]);
      if (traj.points[i].accelerations.size() != 0)
        new_point.accelerations.push_back(traj.points[i].accelerations[mapping[j]]);
    }
    new_point.time_from_start = traj.points[i].time_from_start;
    new_traj_points.push_back(new_point);
  }
}
bool TmSctRos2::has_points(const trajectory_msgs::msg::JointTrajectory &traj)
{
  if (traj.points.size() == 0) return false;
  for (auto &point : traj.points) {
    if (point.positions.size() != traj.joint_names.size() ||
      point.velocities.size() != traj.joint_names.size()) return false;
  }
  return true;
}
bool TmSctRos2::is_traj_finite(const trajectory_msgs::msg::JointTrajectory &traj)
{
  for (auto &point : traj.points) {
    for (auto &p : point.positions) {
      if (!std::isfinite(p)) return false;
    }
    for (auto &v : point.velocities) {
      if (!std::isfinite(v)) return false;
    }
  }
  return true;
}
bool TmSctRos2::is_positions_match(
  const trajectory_msgs::msg::JointTrajectoryPoint &point, double eps)
{
  auto q_act = state_.joint_angle();
  for (size_t i = 0; i < point.positions.size(); ++i) {
    if (fabs(point.positions[i] - q_act[i]) > eps)
      return false;
  }
  return true;
}
void TmSctRos2::set_pvt_traj(
  TmPvtTraj &pvts, const std::vector<trajectory_msgs::msg::JointTrajectoryPoint> &traj_points, double Tmin)
{
  size_t i = 0, i_1 = 0, i_2 = 0;
  int skip_count = 0;
  TmPvtPoint point;

  if (traj_points.size() == 0) return;

  pvts.mode = TmPvtMode::Joint;

  auto sec = [](const builtin_interfaces::msg::Duration& t) {
    return static_cast<double>(t.sec) + 1e-9 * static_cast<double>(t.nanosec);
  };

  // first point
  if (sec(traj_points[i].time_from_start) != 0.0) {
    print_warn("TM_ROS: Traj.: first point should be the current position, with time_from_start set to 0.0");
    point.time = sec(traj_points[i].time_from_start);
    point.positions = traj_points[i].positions;
    point.velocities = traj_points[i].velocities;
    pvts.points.push_back(point);
  }
  for (i = 1; i < traj_points.size() - 1; ++i) {
    point.time = sec(traj_points[i].time_from_start) - sec(traj_points[i_1].time_from_start);
    if (point.time >= Tmin) {
      i_2 = i_1;
      i_1 = i;
      point.positions = traj_points[i].positions;
      point.velocities = traj_points[i].velocities;
      pvts.points.push_back(point);
    }
    else {
      ++skip_count;
    }
  }
  if (skip_count > 0) {
    print_warn("TM_ROS: Traj.: skip %d points", (int)skip_count);	
    //RCLCPP_WARN_STREAM(rclcpp::get_logger("rclcpp"),"TM_ROS: Traj.: skip " << (int)skip_count << " points");
  }
  // last point
  if (traj_points.size() > 1) {
    i =  traj_points.size() - 1;
    point.time = sec(traj_points[i].time_from_start) - sec(traj_points[i_1].time_from_start);
    point.positions = traj_points[i].positions;
    point.velocities = traj_points[i].velocities;
    if (point.time >= Tmin) {
      pvts.points.push_back(point);
    }
    else {
      point.time = sec(traj_points[i].time_from_start) - sec(traj_points[i_2].time_from_start);
      pvts.points.back() = point;
      ++skip_count;
      print_warn("TM_ROS: Traj.: skip 1 more last point");
    }
  }
  pvts.total_time = sec(traj_points.back().time_from_start);
}
std::shared_ptr<TmPvtTraj> TmSctRos2::get_pvt_traj(
    const std::vector<trajectory_msgs::msg::JointTrajectoryPoint> &traj_points, double Tmin)
{
  std::shared_ptr<TmPvtTraj> pvts = std::make_shared<TmPvtTraj>();
  set_pvt_traj(*pvts, traj_points, Tmin);
  return pvts;
}
