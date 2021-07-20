#include "tm_driver/tm_ros2_svr.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

TmSvrRos2::TmSvrRos2(const rclcpp::NodeOptions &options, TmDriver &iface, bool stick_play)
    : Node("tm_svr", options)
    , svr_(iface.svr)
    , state_(iface.state)
    , sct_(iface.sct)
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
    svr_updated_ = true;
    svr_cv_.notify_all();
    if (svr_.is_connected()) {
    }
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
      pm.fbs_msg.is_sct_connected = sct_.is_connected();
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
    tf2::Transform Tbt{ quat, tf2::Vector3(pose[0], pose[1], pose[2]) };
    pm.tool_pose_msg.header.stamp = pm.joint_msg.header.stamp;
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
        print_error("TM_ROS: (TM_SVR): MSG: (%s) (%d) %s", pm.svr_msg.id.c_str(), pm.svr_msg.mode, pm.svr_msg.content.c_str());   	
        print_error("TM_ROS: (TM_SVR) ROS Node Data Error %d",(int)(pm.svr_msg.error_code));
    }
    else {
        print_info("TM_ROS: (TM_SVR): MSG: (%s) (%d) %s", pm.svr_msg.id.c_str(), pm.svr_msg.mode, pm.svr_msg.content.c_str());
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
            print_error("TM_ROS: (TM_SVR) ROS Node Header CPERR %d",(int)svr.tmSvrErrData.error_code());
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
                    print_error("TM_ROS: (TM_SVR): (%s): invalid mode (%d)",
                        svr.data.transaction_id().c_str(), (int)(svr.data.mode()));
                    break;
                }
            }
            else {
                print_error("TM_ROS: (TM_SVR): invalid data");
            }
        }
        else {
            print_error("TM_ROS: (TM_SVR): invalid header");
        }
    }
    if (fbs) {
        publish_fbs(rc);
    }
    if(rc == TmCommRC::TIMEOUT){
      return false;
    }
    return true;
}

void TmSvrRos2::publisher()
{
    TmSvrCommunication &svr = svr_;

    print_info("TM_ROS: publisher thread begin");

    while (rclcpp::ok()) {
        //bool reconnect = false;
        if (!svr.recv_init()) {
            print_info("TM_ROS: (TM_SVR): is not connected");
            publish_fbs(TmCommRC::TIMEOUT);
        }
        while (rclcpp::ok() && svr.is_connected()) {
            if (!publish_func()) break;
        }
        svr.close_socket();

        // reconnect == true
        if (!rclcpp::ok()) break;
        if (pub_reconnect_timeval_ms_ <= 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        print_info("TM_ROS: (TM_SVR): reconnect in ");
        int cnt = 0;
        while (rclcpp::ok() && cnt < pub_reconnect_timeval_ms_) {
            if (cnt % 500 == 0) {
                print_info("%.1f sec...", 0.001 * (pub_reconnect_timeval_ms_ - cnt));
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            ++cnt;
        }
        if (rclcpp::ok() && pub_reconnect_timeval_ms_ >= 0) {
            print_info("0 sec\nTM_ROS: (TM_SVR): connect(%d)...", pub_reconnect_timeout_ms_);
            svr.connect_socket(pub_reconnect_timeout_ms_);
        }
    }
    svr.close_socket();
    print_info("TM_ROS: publisher thread end\n");
}

bool TmSvrRos2::connect_tmsvr(
        const std::shared_ptr<tm_msgs::srv::ConnectTM::Request> req,
        std::shared_ptr<tm_msgs::srv::ConnectTM::Response> res)
{
    bool rb = true;
    int t_o = (int)(1000.0 * req->timeout);
    int t_v = (int)(1000.0 * req->timeval);
    if (req->connect) {
        print_info("TM_ROS: (re)connect(%d) TM_SVR", t_o);
        sct_.halt();
        rb = sct_.start_tm_sct(t_o);
    }
    if (req->reconnect) {
        pub_reconnect_timeout_ms_ = t_o;
        pub_reconnect_timeval_ms_ = t_v;
        print_info("TM_ROS: set SVR reconnect timeout %dms, timeval %dms", t_o, t_v);
    }
    else {
        // no reconnect
        pub_reconnect_timeval_ms_ = -1;
        print_info("TM_ROS: set SVR NOT reconnect");
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
    bool rb= false;

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
