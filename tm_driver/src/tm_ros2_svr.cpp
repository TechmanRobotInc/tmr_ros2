#include "tm_driver/tm_ros2_svr.h"
#include<iostream>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
TmSvrRos2::TmSvrRos2(rclcpp::Node::SharedPtr node, TmDriver &iface, bool is_fake, bool stick_play)
    : node(node)
    , svr_(iface.svr)
    , state_(iface.state)
    , sct_(iface.sct)
    , iface_(iface)
    , is_fake(is_fake)
{
    jns_.clear();
    jns_.push_back("joint_1");
    jns_.push_back("joint_2");
    jns_.push_back("joint_3");
    jns_.push_back("joint_4");
    jns_.push_back("joint_5");
    jns_.push_back("joint_6");

    pm_.fbs_pub = node->create_publisher<tm_msgs::msg::FeedbackState>("feedback_states", 1);
    pm_.joint_pub = node->create_publisher<sensor_msgs::msg::JointState>("joint_states", 1);
    pm_.tool_pose_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>("tool_pose", 1);
    if (!is_fake) {
        pm_.svr_pub = node->create_publisher<tm_msgs::msg::SvrResponse>("svr_response", 1);
    }
    pm_.joint_msg.name = jns_;
    pm_.joint_msg.position.assign(6, 0.0);
    pm_.joint_msg.velocity.assign(6, 0.0);
    pm_.joint_msg.effort.assign(6, 0.0);

    svr_updated_ = false;

    if (!is_fake) {
        ethernetSlaveConnection = std::make_unique<EthernetSlaveConnection>
      (iface,std::bind(&TmSvrRos2::publish_svr, this),stick_play);
        pubDataTimer = node->create_wall_timer(
          std::chrono::milliseconds(publishTimeMs), std::bind(&TmSvrRos2::pub_data, this));
        connect_tm_srv_ = node->create_service<tm_msgs::srv::ConnectTM>(
            "connect_tmsvr", std::bind(&TmSvrRos2::connect_tmsvr, this,
            std::placeholders::_1, std::placeholders::_2));
        write_item_srv_ = node->create_service<tm_msgs::srv::WriteItem>(
            "write_item", std::bind(&TmSvrRos2::write_item, this,
            std::placeholders::_1, std::placeholders::_2));
        ask_item_srv_ = node->create_service<tm_msgs::srv::AskItem>(
            "ask_item", std::bind(&TmSvrRos2::ask_item, this,
            std::placeholders::_1, std::placeholders::_2));
    } else {
      std::vector<double> zeros(state_.DOF);
      state_.set_fake_joint_states(zeros, zeros, zeros);
      pubDataTimer = node->create_wall_timer(
        std::chrono::milliseconds(publishTimeMs), std::bind(&TmSvrRos2::pub_data, this));
      
      getDataThread = std::thread(std::bind(&TmSvrRos2::fake_publisher, this));
    }
}

TmSvrRos2::~TmSvrRos2()
{
    if (getDataThread.joinable()) {
      getDataThread.join();
    }

    print_info("TM_ROS: (Ethernet slave) halt");		
    svr_updated_ = true;
    svr_cv_.notify_all();

     if (is_fake) return;
}

void TmSvrRos2::publish_fbs()
{
    PubMsg &pm = pm_;
    TmRobotState &state = state_;

    // Publish feedback state
    pm.fbs_msg.header.stamp = node->rclcpp::Node::now();
    if(state.get_receive_state() != TmCommRC::TIMEOUT){
      pm.fbs_msg.is_svr_connected = svr_.is_connected();
      pm.fbs_msg.is_sct_connected = sct_.is_connected() & iface_.is_on_listen_node();
      pm.fbs_msg.tmsrv_cperr = (int)svr_.tmSvrErrData.error_code();  //Node State Response 
      pm.fbs_msg.tmsrv_dataerr = (int)pm.svr_msg.error_code;
      pm.fbs_msg.tmscript_cperr = (int)sct_.tmSctErrData.error_code();
      pm.fbs_msg.tmscript_dataerr = (int)sct_.sct_data.sct_has_error();
    } else{
      pm.fbs_msg.is_svr_connected = false;
      pm.fbs_msg.is_sct_connected = false;
      pm.fbs_msg.tmsrv_cperr = false;
      pm.fbs_msg.tmsrv_dataerr = false;
      pm.fbs_msg.tmscript_cperr = false;
      pm.fbs_msg.tmscript_dataerr = false;  
      svr_.tmSvrErrData.set_CPError(TmCPError::Code::Ok);
      pm.svr_msg.error_code = false;
      sct_.tmSctErrData.set_CPError(TmCPError::Code::Ok);
      sct_.sct_data.set_sct_data_has_error(false);
    }    
   
    pm.fbs_msg.max_not_connect_in_s = maxNotConnectTimeInS;
    pm.fbs_msg.disconnection_times = diconnectTimes;

    pm.fbs_msg.is_data_table_correct = state.is_data_table_correct();
    pm.fbs_msg.joint_pos = state.joint_angle();
    pm.fbs_msg.joint_vel = state.joint_speed();
    pm.fbs_msg.joint_tor = state.joint_torque();
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

    // Publish torque state
    pm.fbs_msg.joint_tor_average = state.joint_torque_average();
    pm.fbs_msg.joint_tor_min = state.joint_torque_min();
    pm.fbs_msg.joint_tor_max = state.joint_torque_max();
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
    pm.tool_pose_msg.header.stamp = pm.joint_msg.header.stamp;
    pm.tool_pose_msg.pose.position.x = pose[0];
    pm.tool_pose_msg.pose.position.y = pose[1];
    pm.tool_pose_msg.pose.position.z = pose[2];
    pm.tool_pose_msg.pose.orientation = tf2::toMsg(quat);
    pm.tool_pose_pub->publish(pm.tool_pose_msg);
}
void TmSvrRos2::fake_publisher()
{
    PubMsg &pm = pm_;
    TmRobotState &state = state_;

    print_info("TM_ROS: fake publisher thread begin");		

    while (rclcpp::ok()) {
      // Publish feedback state
      pm.fbs_msg.header.stamp = node->rclcpp::Node::now();
      {
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
    print_info("TM_ROS: fake publisher thread end\n");	
}
void TmSvrRos2::pub_data(){
    iface_.state.update_tm_robot_publish_state();
    publish_fbs();
}
void TmSvrRos2::publish_svr()
{
    PubMsg &pm = pm_;
    TmSvrData &data = svr_.data;
    {
        std::lock_guard<std::mutex> lck(svr_mtx_);
        pm.svr_msg.id = data.transaction_id();
        pm.svr_msg.mode = (int)(data.mode());
        pm.svr_msg.content = std::string{ data.content(), data.content_len() };
        pm.svr_msg.error_code = (int)(data.error_code());
        svr_updated_ = true;
    }
    svr_cv_.notify_all();

    if ((int)(pm.svr_msg.error_code) != 0) {
        print_error("TM_ROS: (Ethernet slave): MSG: (%s) (%d) %s", pm.svr_msg.id.c_str(), (int)pm.svr_msg.mode, pm.svr_msg.content.c_str());
        print_error("TM_ROS: (Ethernet slave): ROS Node Data Error %d", (int)(pm.svr_msg.error_code));
    }
    else {
        print_info("TM_ROS: (Ethernet slave): MSG: (%s) (%d) %s", pm.svr_msg.id.c_str(), (int)pm.svr_msg.mode, pm.svr_msg.content.c_str());
    }    
    
    pm.svr_msg.header.stamp = node->rclcpp::Node::now();
    pm.svr_pub->publish(pm.svr_msg);
}
bool TmSvrRos2::connect_tmsvr(
        const std::shared_ptr<tm_msgs::srv::ConnectTM::Request> req,
        std::shared_ptr<tm_msgs::srv::ConnectTM::Response> res)
{
    bool rb = true;
    int t_o = (int)(1000.0 * req->timeout);
    int t_v = (int)(1000.0 * req->timeval);
    if (req->connect) {

        rb = ethernetSlaveConnection->connect(t_o);
    }
    if (req->reconnect) {
        rb = ethernetSlaveConnection->re_connect(t_o,t_v);
    }
    else {
        ethernetSlaveConnection->no_connect();
    }
    res->ok = rb;
    return rb;
}
bool TmSvrRos2::write_item(
    const std::shared_ptr<tm_msgs::srv::WriteItem::Request> req,
    std::shared_ptr<tm_msgs::srv::WriteItem::Response> res)
{
    bool rb;
    std::string content = req->item + "=" + req->value;
    rb = (svr_.send_content_str(req->id, content) == TmCommRC::OK);
    res->ok = rb;
    return rb;
}
bool TmSvrRos2::ask_item(
    const std::shared_ptr<tm_msgs::srv::AskItem::Request> req,
    std::shared_ptr<tm_msgs::srv::AskItem::Response> res)
{
    PubMsg &pm = pm_;
    bool rb = false;

    svr_mtx_.lock();
    svr_updated_ = false;
    svr_mtx_.unlock();

    rb = (svr_.send_content(req->id, TmSvrData::Mode::READ_STRING, req->item) == TmCommRC::OK);

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
