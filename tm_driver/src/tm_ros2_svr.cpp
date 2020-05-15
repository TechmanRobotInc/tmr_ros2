#include "tm_driver/tm_ros2_svr.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


TmSvrRos2::TmSvrRos2(const rclcpp::NodeOptions &options, TmDriver &iface, bool stick_play)
    : Node("tm_svr", options)
    , svr_(iface.svr)
    , state_(iface.state)
    , sct_(iface.sct)
{
    bool rb = svr_.start(5000);
    if (rb && stick_play) {
        svr_.send_stick_play();
    }

    pm_.fbs_pub = create_publisher<tm_msgs::msg::FeedbackState>("feedback_states", 1);
    pm_.joint_pub = create_publisher<sensor_msgs::msg::JointState>("joint_states", 1);
    pm_.tool_pose_pub = create_publisher<geometry_msgs::msg::PoseStamped>("tool_pose", 1);
    pm_.svr_pub = create_publisher<tm_msgs::msg::SvrResponse>("svr_response", 1);

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
    if (svr_.is_connected()) {
    }
    svr_.halt();
}


void TmSvrRos2::publish_fbs()
{
    PubMsg &pm = pm_;
    TmRobotState &state = state_;

    // Publish feedback state
    pm.fbs_msg.header.stamp = rclcpp::Node::now();

    pm.fbs_msg.is_svr_connected = svr_.is_connected();
    pm.fbs_msg.is_sct_connected = sct_.is_connected();

    pm.fbs_msg.joint_pos = state.joint_angle();
    pm.fbs_msg.joint_vel = state.joint_speed();
    pm.fbs_msg.joint_tor = state.joint_torque();
    pm.fbs_msg.tool_pose = state.tool_pose();
    pm.fbs_msg.tcp_speed = state.tcp_speed_vec();
    pm.fbs_msg.robot_link = state.is_linked();
    pm.fbs_msg.robot_error = state.has_error();
    pm.fbs_msg.project_run = state.is_project_running();
    pm.fbs_msg.project_pause = state.is_project_paused();
    pm.fbs_msg.safetyguard_a = state.is_safeguard_A();
    pm.fbs_msg.e_stop = state.is_EStop();
    pm.fbs_msg.error_code = state.error_code();
    pm.fbs_msg.error_content = state.error_content();
    pm.fbs_msg.cb_digital_input = state.ctrller_DI();
    pm.fbs_msg.cb_analog_input = state.ctrller_AI();
    pm.fbs_msg.ee_digital_input = state.ee_DI();
    pm.fbs_msg.ee_analog_input = state.ee_AI();
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

    pm.svr_msg.id = data.transaction_id();
    pm.svr_msg.mode = (int)(data.mode());
    pm.svr_msg.content = std::string{ data.content(), data.content_len() };
    pm.svr_msg.error_code = (int)(data.error_code());

    print_info("TM_ROS: (TM_SVR): (%s) (%d) %s",
        pm.svr_msg.id.c_str(), pm.svr_msg.mode, pm.svr_msg.content.c_str());

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
    else if (rc != TmCommRC::OK) {
        return true;
    }
    bool fbs = false;
    std::vector<TmPacket> &pack_vec = svr.packet_list();

    for (auto &pack : pack_vec) {
        if (pack.type == TmPacket::Header::CPERR) {
            print_info("TM_ROS: (TM_SVR): CPERR");
            svr.err_data.set_CPError(pack.data.data(), pack.data.size());
            print_error(svr.err_data.error_code_str().c_str());

            // cpe response

        }
        else if (pack.type == TmPacket::Header::TMSVR) {

            svr.err_data.error_code(TmCPError::Code::Ok);

            //TODO ? lock and copy for service response
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
                    print_info("TM_ROS: (TM_SVR): (%s): invalid mode (%d)",
                        svr.data.transaction_id().c_str(), (int)(svr.data.mode()));
                    break;
                }
            }
            else {
                print_info("TM_ROS: (TM_SVR): invalid data");
            }
        }
        else {
            print_info("TM_ROS: (TM_SVR): invalid header");
        }
    }
    if (fbs) {
        publish_fbs();
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
            svr.Connect(pub_reconnect_timeout_ms_);
        }
    }
    svr.Close();
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
        rb = sct_.start(t_o);
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
    bool rb = (svr_.send_content(req->id, TmSvrData::Mode::READ_STRING, req->item) == TmCommRC::OK);
    res->ok = rb;
    return rb;
}
