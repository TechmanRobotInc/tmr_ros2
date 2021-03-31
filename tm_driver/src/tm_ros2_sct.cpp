#include "tm_driver/tm_ros2_sct.h"


TmSctRos2::TmSctRos2(const rclcpp::NodeOptions &options, TmDriver &iface)
    : Node("tm_sct", options)
    , sct_(iface.sct)
    , iface_(iface)
{
    sct_.start(5000);

    sm_.sct_pub = create_publisher<tm_msgs::msg::SctResponse>("sct_response", 1);
    sm_.sta_pub = create_publisher<tm_msgs::msg::StaResponse>("sta_response", 1);

    sta_updated_ = false;

    sct_reconnect_timeout_ms_ = 1000;
    sct_reconnect_timeval_ms_ = 3000;
    sct_thread_ = std::thread(std::bind(&TmSctRos2::sct_responsor, this));


    connect_tm_srv_ = create_service<tm_msgs::srv::ConnectTM>(
        "connect_tmsct", std::bind(&TmSctRos2::connect_tmsct, this,
        std::placeholders::_1, std::placeholders::_2));

    send_script_srv_ = create_service<tm_msgs::srv::SendScript>(
        "send_script", std::bind(&TmSctRos2::send_script, this,
        std::placeholders::_1, std::placeholders::_2));
    set_event_srv_ = create_service<tm_msgs::srv::SetEvent>(
        "set_event", std::bind(&TmSctRos2::set_event, this,
        std::placeholders::_1, std::placeholders::_2));
    set_io_srv_ = create_service<tm_msgs::srv::SetIO>(
        "set_io", std::bind(&TmSctRos2::set_io, this,
        std::placeholders::_1, std::placeholders::_2));
    set_positions_srv_ = create_service<tm_msgs::srv::SetPositions>(
        "set_positions", std::bind(&TmSctRos2::set_positions, this,
        std::placeholders::_1, std::placeholders::_2));

    ask_sta_srv_ = create_service<tm_msgs::srv::AskSta>(
        "ask_sta", std::bind(&TmSctRos2::ask_sta, this,
        std::placeholders::_1, std::placeholders::_2));

}
TmSctRos2::~TmSctRos2()
{
    sta_updated_ = true;
    sta_cv_.notify_all();
    if (sct_.is_connected()) {
    }
    sct_.halt();
}


void TmSctRos2::sct_msg()
{
    SctMsg &sm = sm_;
    TmSctData &data = sct_.sct_data;

    sm.sct_msg.id = data.script_id();
    sm.sct_msg.script = std::string{ data.script(), data.script_len() };

    if (data.has_error()) {
        print_info("TM_ROS: (TM_SCT): err: (%s): %s", sm.sct_msg.id.c_str(), sm.sct_msg.script.c_str());
    }
    else {
        print_info("TM_ROS: (TM_SCT): res: (%s): %s", sm.sct_msg.id.c_str(), sm.sct_msg.script.c_str());
    }

    sm.sct_msg.header.stamp = rclcpp::Node::now();
    sm.sct_pub->publish(sm.sct_msg);
}
void TmSctRos2::sta_msg()
{
    SctMsg &sm = sm_;
    TmStaData &data = sct_.sta_data;
    {
        std::lock_guard<std::mutex> lck(sta_mtx_);
        sm.sta_msg.subcmd = data.subcmd_str();
        sm.sta_msg.subdata = std::string{ data.subdata(), data.subdata_len() };
        sta_updated_ = true;
    }
    sta_cv_.notify_all();

    print_info("TM_ROS: (TM_STA): res: (%s): %s", sm.sta_msg.subcmd.c_str(), sm.sta_msg.subdata.c_str());

    sm.sta_msg.header.stamp = rclcpp::Node::now();
    sm.sta_pub->publish(sm.sta_msg);
}
bool TmSctRos2::sct_func()
{
    TmSctCommunication &sct = sct_;
    int n;
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
            print_info("TM_ROS: (TM_SCT): CPERR");
            sct.err_data.set_CPError(pack.data.data(), pack.data.size());
            print_error(sct.err_data.error_code_str().c_str());

            // cpe response

            break;

        case TmPacket::Header::TMSCT:

            sct.err_data.error_code(TmCPError::Code::Ok);

            //TODO ? lock and copy for service response
            TmSctData::build_TmSctData(sct.sct_data, pack.data.data(), pack.data.size(), TmSctData::SrcType::Shallow);

            sct_msg();
            break;

        case TmPacket::Header::TMSTA:

            sct.err_data.error_code(TmCPError::Code::Ok);

            TmStaData::build_TmStaData(sct.sta_data, pack.data.data(), pack.data.size(), TmStaData::SrcType::Shallow);

            sta_msg();
            break;

        default:
            print_info("TM_ROS: (TM_SCT): invalid header");
            break;
        }
    }
    return true;
}
void TmSctRos2::sct_responsor()
{
    TmSctCommunication &sct = sct_;

    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    print_info("TM_ROS: sct_response thread begin");

    while (rclcpp::ok()) {
        //bool reconnect = false;
        if (!sct.recv_init()) {
            print_info("TM_ROS: (TM_SCT): is not connected");
        }
        while (rclcpp::ok() && sct.is_connected()) {
            if (!sct_func()) break;
        }
        sct.close_socket();

        // reconnect == true
        if (!rclcpp::ok()) break;
        if (sct_reconnect_timeval_ms_ <= 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        print_info("TM_ROS: (TM_SCT) reconnect in ");
        int cnt = 0;
        while (rclcpp::ok() && cnt < sct_reconnect_timeval_ms_) {
            if (cnt % 1000 == 0) {
                print_info("%.1f sec...", 0.001 * (sct_reconnect_timeval_ms_ - cnt));
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            ++cnt;
        }
        if (rclcpp::ok() && sct_reconnect_timeval_ms_ >= 0) {
            print_info("0 sec\nTM_ROS: (TM_SCT) connect(%d)...", sct_reconnect_timeout_ms_);
            sct.connect_socket(sct_reconnect_timeout_ms_);
        }
    }
    sct.close_socket();
    printf("TM_ROS: sct_response thread end\n");
}


bool TmSctRos2::connect_tmsct(
        const std::shared_ptr<tm_msgs::srv::ConnectTM::Request> req,
        std::shared_ptr<tm_msgs::srv::ConnectTM::Response> res)
{
    bool rb = true;
    int t_o = (int)(1000.0 * req->timeout);
    int t_v = (int)(1000.0 * req->timeval);
    if (req->connect) {
        print_info("TM_ROS: (re)connect(%d) TM_SCT", t_o);
        sct_.halt();
        rb = sct_.start(t_o);
    }
    if (req->reconnect) {
        sct_reconnect_timeout_ms_ = t_o;
        sct_reconnect_timeval_ms_ = t_v;
        print_info("TM_ROS: set SCT reconnect timeout %dms, timeval %dms", t_o, t_v);
    }
    else {
        // no reconnect
        sct_reconnect_timeval_ms_ = -1;
        print_info("TM_ROS: set SCT NOT reconnect");
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

bool TmSctRos2::ask_sta(
        const std::shared_ptr<tm_msgs::srv::AskSta::Request> req,
        std::shared_ptr<tm_msgs::srv::AskSta::Response> res)
{
    SctMsg &sm = sm_;
    bool rb = false;

    sta_mtx_.lock();
    sta_updated_ = false;
    sta_mtx_.unlock();

    rb = (sct_.send_script_str(req->subcmd, req->subdata) == iface_.RC_OK);

    {
        std::unique_lock<std::mutex> lck(sta_mtx_);
        if (rb && req->wait_time > 0.0) {
            if (!sta_updated_) {
                sta_cv_.wait_for(lck, std::chrono::duration<double>(req->wait_time));
            }
            if (!sta_updated_) {
                rb = false;
            }
            res->subcmd = sm.sta_msg.subcmd;
            res->subdata = sm.sta_msg.subdata;
        }
        sta_updated_ = false;
    }
    res->ok = rb;
    return rb;
}
