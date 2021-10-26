#include "tm_driver/tm_ros2_svr.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
TmSvrRos2::TmSvrRos2(const rclcpp::NodeOptions &options, TmDriver &iface, bool stick_play)
    : Node("tm_svr", options)
    , svr_(iface.svr)
    , state_(iface.state)
    , sct_(iface.sct)
    , iface_(iface)
{
    bool rb = svr_.start_tm_svr(5000);
    if (rb && stick_play) {
        svr_.send_stick_play();
    }

    pm_.fbs_pub = create_publisher<tm_msgs::msg::FeedbackState>("feedback_states", 1);
    pm_.joint_pub = create_publisher<sensor_msgs::msg::JointState>("joint_states", 1);
    pm_.tool_pose_pub = create_publisher<geometry_msgs::msg::PoseStamped>("tool_pose", 1);
    pm_.svr_pub = create_publisher<tm_msgs::msg::SvrResponse>("svr_response", 1);

    svr_updated_ = false;

    pub_reconnect_timeout_ms_ = 1000;
    pub_reconnect_timeval_ms_ = 3000;
    pub_thread_ = std::thread(std::bind(&TmSvrRos2::publisher, this));

    connect_tm_srv_ = create_service<tm_msgs::srv::ConnectTM>(
        "connect_tmsvr", std::bind(&TmSvrRos2::connect_tmsvr, this,
        std::placeholders::_1, std::placeholders::_2));

    write_item_srv_ = create_service<tm_msgs::srv::WriteItem>(
        "write_item", std::bind(&TmSvrRos2::write_item, this,
        std::placeholders::_1, std::placeholders::_2));
        
    ask_item_srv_ = create_service<tm_msgs::srv::AskItem>(
        "ask_item", std::bind(&TmSvrRos2::ask_item, this,
        std::placeholders::_1, std::placeholders::_2));
}

TmSvrRos2::~TmSvrRos2()
{
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),"TM_ROS: (TM_SVR) halt");		
    svr_updated_ = true;
    svr_cv_.notify_all();
    if (svr_.is_connected()) {}
    svr_.halt();
}

void TmSvrRos2::publish_fbs(TmCommRC rc)
{
    PubMsg &pm = pm_;
    TmRobotState &state = state_;

    // Publish feedback state
    pm.fbs_msg.header.stamp = rclcpp::Node::now();
    if(rc != TmCommRC::TIMEOUT){
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
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"),"TM_ROS: (TM_SVR): MSG (" << pm.svr_msg.id << ") (" << (int)(pm.svr_msg.mode) << ") " << pm.svr_msg.content);  	
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"),"TM_ROS: (TM_SVR): ROS Node Data Error" << (int)(pm.svr_msg.error_code));
    }
    else {
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),"TM_ROS: (TM_SVR): MSG  (" << pm.svr_msg.id << ") (" << (int)(pm.svr_msg.mode) << ") " << pm.svr_msg.content);
    }    
    
    pm.svr_msg.header.stamp = rclcpp::Node::now();
    pm.svr_pub->publish(pm.svr_msg);
}

bool TmSvrRos2::publish_func()
{
    TmSvrCommunication &svr = svr_;
    int n;
    auto rc = svr.recv_spin_once(1000, &n);

    if (rc == TmCommRC::ERR ||
        rc == TmCommRC::NOTREADY ||
        rc == TmCommRC::NOTCONNECT) {
        return false;
    }
    bool fbs = false;
    std::vector<TmPacket> &pack_vec = svr.packet_list();

    for (auto &pack : pack_vec) {
        if (pack.type == TmPacket::Header::CPERR) {
            svr.tmSvrErrData.set_CPError(pack.data.data(), pack.data.size());
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"),"TM_ROS: (TM_SVR) ROS Node Header CPERR" << (int)svr.tmSvrErrData.error_code());
        }
        else if (pack.type == TmPacket::Header::TMSVR) {

            if (svr.data.is_valid() && (svr.data.mode()== TmSvrData::Mode::BINARY))
            {
                if ((int)svr_.tmSvrErrData.error_code())  {} 
                else
                {
                    svr.tmSvrErrData.error_code(TmCPError::Code::Ok); 
                }
            }
            else if (svr.data.is_valid() && (svr.data.mode() == TmSvrData::Mode::RESPONSE))
            {
                if ((int)svr_.data.error_code() != 0)  {}
                else
                {  
                  svr.tmSvrErrData.error_code(TmCPError::Code::Ok);
                }
            }            
            else
            {
                svr.tmSvrErrData.error_code(TmCPError::Code::Ok); 
            }            
            TmSvrData::build_TmSvrData(svr.data, pack.data.data(), pack.data.size(), TmSvrData::SrcType::Shallow);

            if (svr.data.is_valid()) {
                switch (svr.data.mode()) {
                case TmSvrData::Mode::RESPONSE:
                    //print_info("TM_ROS: (TM_SVR): (%s) RESPONSE [%d]",
                    //    svr.data.transaction_id().c_str(), (int)(svr.data.error_code()));
                    publish_svr();
                    break;
                case TmSvrData::Mode::BINARY:
                    svr.state.mtx_deserialize(svr.data.content(), svr.data.content_len());
                    fbs = true;
                    break;
                case TmSvrData::Mode::READ_STRING:
                case TmSvrData::Mode::READ_JSON:
                    publish_svr();
                    break;
                default:
                    RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"),"TM_ROS: (TM_SVR): (" <<
                        svr.data.transaction_id() << "): invalid mode (" << (int)(svr.data.mode()) << ")");
                    break;
                }
            }
            else {
                RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"),"TM_ROS: (TM_SVR): invalid data");
            }
        }
        else {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"),"TM_ROS: (TM_SVR): invalid header");
        }
    }
    if (fbs) {
        publish_fbs(rc);
    }
    if(rc == TmCommRC::TIMEOUT){
      RCLCPP_INFO_STREAM_ONCE(rclcpp::get_logger("rclcpp"), "TM_ROS: (TM_SVR): LINK TIMEOUT");
      return false;
    }
    return true;
}

void TmSvrRos2::cq_monitor(){  //Connection quality
    diconnectTimes ++;
    initialNotConnectTime =  TmCommunication::get_current_time_in_ms();
}

void TmSvrRos2::cq_manage(){
    
    notConnectTimeInS = (TmCommunication::get_current_time_in_ms() - initialNotConnectTime)/1000;
    if(diconnectTimes == 0){
        return;
    }
    if(notConnectTimeInS>maxNotConnectTimeInS){
        maxNotConnectTimeInS = notConnectTimeInS;
    }
}

bool TmSvrRos2::rc_halt(){  //Stop rescue connection
    if(maxTrialTimeInMinute == -1){
        return false;
    }
    if((int)(notConnectTimeInS/60) >= maxTrialTimeInMinute){
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("rclcpp"),"TM_ROS: (TM_SVR): notConnectTimeInS = " << (int)notConnectTimeInS << ", maxTrialTimeInMinute = " << (int)maxTrialTimeInMinute);
        return true;
    }
    return false;
}

void TmSvrRos2::publisher()
{
    TmSvrCommunication &svr = svr_;

    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),"TM_ROS: publisher thread begin");
    initialNotConnectTime =  TmCommunication::get_current_time_in_ms();
    while (rclcpp::ok()) {
        //bool reconnect = false;
        if (iface_.get_connect_recovery_guide()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        else   
        {	        	
            if (!svr.recv_init()) {
                RCLCPP_DEBUG_STREAM(rclcpp::get_logger("rclcpp"),"TM_ROS: (TM_SVR): is not connected");
                cq_manage();
                publish_fbs(TmCommRC::TIMEOUT);
                if(rc_halt()) {
                    RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"),"TM_ROS: (TM_SVR): Ethernet slave connection stopped");
                    if (diconnectTimes == 0)
                    {
                        RCLCPP_WARN_STREAM(rclcpp::get_logger("rclcpp"),"TM_ROS: (TM_SVR): Please Check Wired Connected Settings");
                    }
                    else
                    {                    
                        RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"),"TM_ROS: (TM_SVR): LinkLost = " << (int)diconnectTimes << ", MaxLostTime(s) = " << (int)maxNotConnectTimeInS);
                    }
                    iface_.set_connect_recovery_guide(true);
                    svr.close_socket();
                }
            }
            if (!iface_.get_connect_recovery_guide()) {
                while (rclcpp::ok() && svr.is_connected()) {
                    if (!publish_func()){
                        cq_monitor();
                        break;
                    }
                }
                        
                svr.close_socket();
                if (!rclcpp::ok()) break;
                svr_connect_recover();
            }
        } 
    }
    svr.close_socket();
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),"TM_ROS: publisher thread end");
}

void TmSvrRos2::svr_connect_recover()
{
    TmSvrCommunication &svr = svr_;
    int timeInterval = 0;
    int lastTimeInterval = 1000;
    	    	
    if (pub_reconnect_timeval_ms_ <= 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),"TM_ROS: (TM_SVR): Reconnecting... ");

    uint64_t startTimeMs = TmCommunication::get_current_time_in_ms();
    while (rclcpp::ok() && timeInterval < pub_reconnect_timeval_ms_) {
        if ( lastTimeInterval/1000 != timeInterval/1000) {
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger("rclcpp"),"TM_SVR reconnect remain :" << 0.001 * (pub_reconnect_timeval_ms_ - timeInterval) << " sec...");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        lastTimeInterval = timeInterval;
        timeInterval = TmCommunication::get_current_time_in_ms() - startTimeMs;
    }    
    if (rclcpp::ok() && pub_reconnect_timeval_ms_ >= 0) {
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("rclcpp"),"0 sec\nTM_ROS: (TM_SVR): connect" << (int)pub_reconnect_timeout_ms_ << "ms)...");
        svr.connect_socket(pub_reconnect_timeout_ms_);
    }
}

bool TmSvrRos2::connect_tmsvr(
        const std::shared_ptr<tm_msgs::srv::ConnectTM::Request> req,
        std::shared_ptr<tm_msgs::srv::ConnectTM::Response> res)
{
    bool rb = true;
    int t_o = (int)(1000.0 * req->timeout);
    int t_v = (int)(1000.0 * req->timeval);
    if (req->connect) {
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),"TM_ROS: (re)connect(" << (int)t_o << ") TM_SVR");
        svr_.halt();
        rb = svr_.start_tm_svr(t_o);
    }
    if (req->reconnect) {
        if (iface_.get_connect_recovery_guide())
        {
            pub_reconnect_timeout_ms_ = 1000;
            pub_reconnect_timeval_ms_ = 3000;
            initialNotConnectTime =  TmCommunication::get_current_time_in_ms();
            notConnectTimeInS = 0;
            diconnectTimes = 0;
            maxNotConnectTimeInS = 0;
            iface_.set_connect_recovery_guide(false);
            rb = svr_.start_tm_svr(5000);
            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),"TM_ROS: TM_SVR resume connection recovery");      	
        }
        else
        {
            pub_reconnect_timeout_ms_ = t_o;
            pub_reconnect_timeval_ms_ = t_v;
        }	
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),"TM_ROS: set TM_SVR reconnect timeout " << (int)pub_reconnect_timeout_ms_ << "ms, timeval " << (int)pub_reconnect_timeval_ms_ << "ms"); 	
    }
    else {
        // no reconnect
        pub_reconnect_timeval_ms_ = -1;
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),"TM_ROS: set TM_SVR NOT reconnect");
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
