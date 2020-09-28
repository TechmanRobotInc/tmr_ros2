#include "tmr_driver/tmr_ros2_svr.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


TmSvrRos2::TmSvrRos2(const rclcpp::NodeOptions &options, tmr::Driver &iface, bool is_fake, bool stick_play)
  : Node("tmr_svr", options)
  , svr_(iface.svr)
  , sct_(iface.sct)
  , iface_(iface)
  , state_(iface.state)
  , is_fake_(is_fake)
{
  if (!is_fake_) {
    bool rb = svr_.start(5000);
    if (rb && stick_play) {
      svr_.send_stick_play();
    }
  }
  else {
    std::vector<double> zeros(state_.DOF);
    state_.set_joint_states(zeros, zeros, zeros);
  }

  jns_.clear();
  jns_.push_back("joint_1");
  jns_.push_back("joint_2");
  jns_.push_back("joint_3");
  jns_.push_back("joint_4");
  jns_.push_back("joint_5");
  jns_.push_back("joint_6");

  pm_.fbs_pub = create_publisher<tmr_msgs::msg::FeedbackState>("feedback_states", 1);
  pm_.joint_pub = create_publisher<sensor_msgs::msg::JointState>("joint_states", 1);
  pm_.tool_pose_pub = create_publisher<geometry_msgs::msg::PoseStamped>("tool_pose", 1);
  if (!is_fake_) {
    pm_.svr_pub = create_publisher<tmr_msgs::msg::SvrResponse>("tmr/svr_response", 1);
  }
  pm_.joint_msg.name = jns_;
  pm_.joint_msg.position.assign(6, 0.0);
  pm_.joint_msg.velocity.assign(6, 0.0);
  pm_.joint_msg.effort.assign(6, 0.0);

  svr_updated_ = false;
  pub_reconnect_timeout_ms_ = 1000;
  pub_reconnect_timeval_ms_ = 3000;

  if (!is_fake_) {
    pub_thread_ = std::thread(std::bind(&TmSvrRos2::publisher, this));
  }
  else {
    pub_thread_ = std::thread(std::bind(&TmSvrRos2::fake_publisher, this));
  }
  if (!is_fake_) {
    connect_tm_srv_ = create_service<tmr_msgs::srv::ConnectTM>(
      "tmr/connect_tmsvr", std::bind(&TmSvrRos2::connect_tmsvr, this,
      std::placeholders::_1, std::placeholders::_2));

    write_item_srv_ = create_service<tmr_msgs::srv::WriteItem>(
      "tmr/write_item", std::bind(&TmSvrRos2::write_item, this,
      std::placeholders::_1, std::placeholders::_2));
    ask_item_srv_ = create_service<tmr_msgs::srv::AskItem>(
      "tmr/ask_item", std::bind(&TmSvrRos2::ask_item, this,
      std::placeholders::_1, std::placeholders::_2));
  }
}
TmSvrRos2::~TmSvrRos2()
{
  std::cout << "TM_ROS: (TM_SVR) halt\n";

  if (pub_thread_.joinable()) {
    pub_thread_.join();
  }

  if (is_fake_) return;

  svr_updated_ = true;
  svr_cv_.notify_all();
  svr_.halt();
}


void TmSvrRos2::fake_publisher()
{
  PubMsg &pm = pm_;
  //tmr::TmSvrCommunication &svr = svr_;
  tmr::RobotState &state = state_;

  tmr_INFO_STREAM("TM_ROS: fake publisher thread begin");

  while (rclcpp::ok()) {
    // Publish feedback state
    pm.fbs_msg.header.stamp = rclcpp::Node::now();
    {
      std::lock_guard<std::mutex> lck(state_.mtx);
      pm.fbs_msg.joint_pos = state.joint_angle();
      pm.fbs_msg.joint_vel = state.joint_speed();
      pm.fbs_msg.joint_tor = state.joint_torque();
    }
    pm.fbs_pub->publish(pm.fbs_msg);

    // Publish joint state
    pm.joint_msg.header.stamp = pm.fbs_msg.header.stamp;
    pm.joint_msg.position = pm.fbs_msg.joint_pos;
    pm.joint_msg.velocity = pm.fbs_msg.joint_vel;
    pm.joint_msg.effort = pm.fbs_msg.joint_tor;
    pm.joint_pub->publish(pm.joint_msg);

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  std::cout << "TM_ROS: fake publisher thread end\n";
}

void TmSvrRos2::publish_fbs()
{
  PubMsg &pm = pm_;
  tmr::RobotState &state = state_;

  // Publish feedback state
  pm.fbs_msg.header.stamp = rclcpp::Node::now();

  pm.fbs_msg.is_svr_connected = svr_.is_connected();
  pm.fbs_msg.is_sct_connected = sct_.is_connected();

  pm.fbs_msg.joint_pos = state.joint_angle();
  pm.fbs_msg.joint_vel = state.joint_speed();
  pm.fbs_msg.joint_tor = state.joint_torque();
  //pm.fbs_msg.flange_pose = state.flange_pose(); 
  pm.fbs_msg.tool_pose = state.tool_pose();
  pm.fbs_msg.tcp_speed = state.tcp_speed_vec();
  pm.fbs_msg.tcp_force = state.tcp_force_vec();
  pm.fbs_msg.robot_link = state.is_linked();
  pm.fbs_msg.robot_error = state.has_error();
  pm.fbs_msg.project_run = state.is_project_running();
  pm.fbs_msg.project_pause = state.is_project_paused();
  pm.fbs_msg.safetyguard_a = state.is_safeguard_A();
  pm.fbs_msg.e_stop = state.is_EStop();
  pm.fbs_msg.camera_light = state.camera_light();
  pm.fbs_msg.error_code = state.error_code();
  pm.fbs_msg.project_speed = state.project_speed();
  pm.fbs_msg.ma_mode = state.ma_mode();
  pm.fbs_msg.robot_light = state.robot_light();
  pm.fbs_msg.cb_digital_output = state.ctrller_DO();
  pm.fbs_msg.cb_digital_input = state.ctrller_DI();
  pm.fbs_msg.cb_analog_output = state.ctrller_AO();
  pm.fbs_msg.cb_analog_input = state.ctrller_AI();
  pm.fbs_msg.ee_digital_output = state.ee_DO();
  pm.fbs_msg.ee_digital_input = state.ee_DI();
  //pm.fbs_msg.ee_analog_output = state.ee_AO();
  pm.fbs_msg.ee_analog_input = state.ee_AI();
  pm.fbs_msg.error_content = state.error_content();
  pm.fbs_pub->publish(pm.fbs_msg);

  // Publish joint state
  pm.joint_msg.header.stamp = pm.fbs_msg.header.stamp;
  pm.joint_msg.position = pm.fbs_msg.joint_pos;
  pm.joint_msg.velocity = pm.fbs_msg.joint_vel;
  pm.joint_msg.effort = pm.fbs_msg.joint_tor;
  pm.joint_pub->publish(pm.joint_msg);

  // Publish tool pose
  auto &pose = pm.fbs_msg.tool_pose;
  tf2::Quaternion quat;
  quat.setRPY(pose[3], pose[4], pose[5]);
  tf2::Transform Tbt{ quat, tf2::Vector3(pose[0], pose[1], pose[2]) };
  pm.tool_pose_msg.header.stamp = pm.joint_msg.header.stamp;

  pm.tool_pose_pub->publish(pm.tool_pose_msg);
}
void TmSvrRos2::publish_svr()
{
  PubMsg &pm = pm_;
  tmr::TmSvrData &data = svr_.data;
  {
    std::lock_guard<std::mutex> lck(svr_mtx_);
    pm.svr_msg.id = data.transaction_id();
    pm.svr_msg.mode = (int)(data.mode());
    pm.svr_msg.content = std::string{ data.content(), data.content_len() };
    pm.svr_msg.error_code = (int)(data.error_code());
    svr_updated_ = true;
  }
  svr_cv_.notify_all();

  tmr_INFO_STREAM("TM_ROS: (TM_SVR): (" <<
    pm.svr_msg.id << ") (" << (int)(pm.svr_msg.mode) << ") " << pm.svr_msg.content);

  pm.svr_msg.header.stamp = rclcpp::Node::now();
  pm.svr_pub->publish(pm.svr_msg);
}
bool TmSvrRos2::publish_func()
{
  //PubMsg &pm = pm_;
  tmr::TmSvrCommunication &svr = svr_;
  int n;
  auto rc = svr.recv_spin_once(1000, &n);
  if (rc == tmr::CommRC::ERR ||
    rc == tmr::CommRC::NOTREADY ||
    rc == tmr::CommRC::NOTCONNECT) {
    return false;
  }
  else if (rc != tmr::CommRC::OK) {
      return true;
  }
  bool fbs = false;
  std::vector<tmr::TmPacket> &pack_vec = svr.packet_list();

  for (auto &pack : pack_vec) {
    if (pack.type == tmr::TmPacket::Header::CPERR) {
      tmr_INFO_STREAM("TM_ROS: (TM_SVR): CPERR");
      svr.err_data.set_CPError(pack.data.data(), pack.data.size());
      tmr_ERROR_STREAM(svr.err_data.error_code_str());

      // cpe response

    }
    else if (pack.type == tmr::TmPacket::Header::TMSVR) {

      svr.err_data.error_code(tmr::TmCPError::Code::Ok);

      //TODO ? lock and copy for service response
      tmr::TmSvrData::build_TmSvrData(svr.data, pack.data.data(), pack.data.size(), tmr::TmSvrData::SrcType::Shallow);

      if (svr.data.is_valid()) {
        switch (svr.data.mode()) {
        case tmr::TmSvrData::Mode::RESPONSE:
          //tmr_INFO_STREAM("TM_SVR: RESPONSE (" << svr.data.transaction_id() << "): [" <<
          //  (int)(svr.data.error_code()) << "]: " << std::string(svr.data.content(), svr.data.content_len()));
          publish_svr();
          break;
        case tmr::TmSvrData::Mode::BINARY:
          svr.state.mtx_deserialize(svr.data.content(), svr.data.content_len());
          fbs = true;
          break;
        case tmr::TmSvrData::Mode::READ_STRING:
        case tmr::TmSvrData::Mode::READ_JSON:
          publish_svr();
          break;
        default:
          tmr_WARN_STREAM("TM_ROS: (TM_SVR): (" <<
            svr.data.transaction_id() << "): invalid mode (" << (int)(svr.data.mode()) << ")");
          break;
        }
      }
      else {
        tmr_WARN_STREAM("TM_ROS: (TM_SVR): invalid data");
      }
    }
    else {
      tmr_WARN_STREAM("TM_ROS: (TM_SVR): invalid header");
    }
  }
  if (fbs) {
    publish_fbs();
  }
  return true;
}
void TmSvrRos2::publisher()
{
  //PubMsg &pm = pm_;
  tmr::TmSvrCommunication &svr = svr_;

  tmr_INFO_STREAM("TM_ROS: publisher thread begin");

  //pm.joint_msg.name = joint_names_;
  //pm.joint_msg.position.assign(joint_names_.size(), 0.0);
  //pm.joint_msg.velocity.assign(joint_names_.size(), 0.0);
  //pm.joint_msg.effort.assign(joint_names_.size(), 0.0);

  while (rclcpp::ok()) {
    //bool reconnect = false;
    if (!svr.recv_init()) {
      tmr_INFO_STREAM("TM_ROS: (TM_SVR): is not connected");
    }
    while (rclcpp::ok() && svr.is_connected()) {
      if (!publish_func()) break;
    }
    svr.Close();

    // reconnect == true
    if (!rclcpp::ok()) break;
    if (pub_reconnect_timeval_ms_ <= 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    tmr_INFO_STREAM("TM_ROS: (TM_SVR): reconnect in ");
    int cnt = 0;
    while (rclcpp::ok() && cnt < pub_reconnect_timeval_ms_) {
      if (cnt % 500 == 0) {
        tmr_INFO_STREAM(0.001 * (pub_reconnect_timeval_ms_ - cnt) << " sec...");
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      ++cnt;
    }
    if (rclcpp::ok() && pub_reconnect_timeval_ms_ >= 0) {
      tmr_INFO_STREAM("0 sec\nTM_ROS: (TM_SVR): connect" << pub_reconnect_timeout_ms_ << "ms)...");
      svr.Connect(pub_reconnect_timeout_ms_);
    }
  }
  svr.Close();
  std::cout << "TM_ROS: publisher thread end\n";
}


bool TmSvrRos2::connect_tmsvr(
  const std::shared_ptr<tmr_msgs::srv::ConnectTM::Request> req,
  std::shared_ptr<tmr_msgs::srv::ConnectTM::Response> res)
{
  bool rb = true;
  int t_o = (int)(1000.0 * req->timeout);
  int t_v = (int)(1000.0 * req->timeval);
  if (req->connect) {
    tmr_INFO_STREAM("TM_ROS: (re)connect(" << t_o << ") TM_SVR");
    svr_.halt();
    rb = svr_.start(t_o);
  }
  if (req->reconnect) {
    pub_reconnect_timeout_ms_ = t_o;
    pub_reconnect_timeval_ms_ = t_v;
    tmr_INFO_STREAM("TM_ROS: set TM_SVR reconnect timeout " << t_o << "ms, timeval " << t_v << "ms");
  }
  else {
    // no reconnect
    pub_reconnect_timeval_ms_ = -1;
    tmr_INFO_STREAM("TM_ROS: set TM_SVR NOT reconnect");
  }
  res->ok = rb;
  return rb;
}

bool TmSvrRos2::write_item(
  const std::shared_ptr<tmr_msgs::srv::WriteItem::Request> req,
  std::shared_ptr<tmr_msgs::srv::WriteItem::Response> res)
{
  bool rb;
  std::string content = req->item + "=" + req->value;
  rb = (svr_.send_content_str(req->id, content) == tmr::CommRC::OK);
  res->ok = rb;
  return rb;
}
bool TmSvrRos2::ask_item(
  const std::shared_ptr<tmr_msgs::srv::AskItem::Request> req,
  std::shared_ptr<tmr_msgs::srv::AskItem::Response> res)
{
  PubMsg &pm = pm_;
  //tmr::TmSvrData &data = svr_.data;
  bool rb = false;

  svr_mtx_.lock();
  svr_updated_ = false;
  svr_mtx_.unlock();

  rb = (svr_.send_content(req->id, tmr::TmSvrData::Mode::READ_STRING, req->item) == tmr::CommRC::OK);

  {
    std::unique_lock<std::mutex> lck(svr_mtx_);
    if (rb && req->wait_time > 0.0) {
      if (!svr_updated_) {
        svr_cv_.wait_for(lck, std::chrono::duration<double>(req->wait_time));
      }
      if (!svr_updated_) {
        rb = false;
      }
      res->id = pm.svr_msg.id;
      res->value = pm.svr_msg.content;
    }
    svr_updated_ = false;
  }
  res->ok = rb;
  return rb;
}
