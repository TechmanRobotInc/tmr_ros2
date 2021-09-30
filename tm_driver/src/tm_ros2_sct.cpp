#include "tm_driver/tm_ros2_sct.h"

TmSctRos2::TmSctRos2(const rclcpp::NodeOptions &options, TmDriver &iface)
    : Node("tm_sct", options)
    , sct_(iface.sct)
    , iface_(iface)
{
    sct_.start_tm_sct(5000);

    sm_.sct_pub = create_publisher<tm_msgs::msg::SctResponse>("sct_response", 1);
    sm_.sta_pub = create_publisher<tm_msgs::msg::StaResponse>("sta_response", 1);

    sta_updated_ = false;

    sct_reconnect_timeout_ms_ = 1000;
    sct_reconnect_timeval_ms_ = 3000;
    checkListenNodeThread = std::thread(std::bind(&TmSctRos2::check_is_on_listen_node, this));
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
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),"TM_ROS: (TM_SCT) halt");		
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
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"),"TM_ROS: (TM_SCT): MSG : (" << sm.sct_msg.id << "): " << sm.sct_msg.script);
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"),"TM_ROS: (TM_SCT): ROS Node Data Error (" << (int)data.sct_has_error() << ")");
    }
    else {
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),"TM_ROS: (TM_SCT): MSG : (" << sm.sct_msg.id << "): " << sm.sct_msg.script);
    }

    sm.sct_msg.header.stamp = rclcpp::Node::now();
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

    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),"TM_ROS: (TM_STA): res: (" << sm.sta_msg.subcmd << "): " << sm.sta_msg.subdata);

    sm.sta_msg.header.stamp = rclcpp::Node::now();
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
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"),"TM_ROS: (TM_SCT) ROS Node Header CPERR" << (int)sct.tmSctErrData.error_code());
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
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"),"TM_ROS: (TM_SCT): invalid header");
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
            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),"TM_ROS: On listen node.");
            iface_.back_to_listen_node();
        } else{
            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),"TM_ROS: Not on listen node!");
        }
        checkIsOnListenNodeCondVar.wait(checkIsOnListenNodeLock);
    }
}

void TmSctRos2::sct_responsor()
{
    TmSctCommunication &sct = sct_;

    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),"TM_ROS: sct_response thread begin");

    while (rclcpp::ok()) {
        //bool reconnect = false;
        if (iface_.get_connect_recovery_guide()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        else   
        {
            if (!sct.recv_init()) {
                RCLCPP_DEBUG_STREAM(rclcpp::get_logger("rclcpp"),"TM_ROS: (TM_SCT): is not connected");
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
    
    sct.close_socket();
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),"TM_ROS: sct_response thread end\n");		
}

void TmSctRos2::sct_connect_recover()
{
    TmSctCommunication &sct = sct_;
    int timeInterval = 0;
    int lastTimeInterval=1000;
            	
    if (sct_reconnect_timeval_ms_ <= 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),"TM_ROS: (TM_SCT): Reconnecting...");

    uint64_t startTimeMs = TmCommunication::get_current_time_in_ms();
    while (rclcpp::ok() && timeInterval < sct_reconnect_timeval_ms_) {
        if ( lastTimeInterval/1000 != timeInterval/1000) {
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger("rclcpp"),"TM_SCT reconnect remain : " << (0.001 * (sct_reconnect_timeval_ms_ - timeInterval)) << " sec... ");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        lastTimeInterval = timeInterval;
        timeInterval = TmCommunication::get_current_time_in_ms() - startTimeMs;
    }
    if (rclcpp::ok() && sct_reconnect_timeval_ms_ >= 0) {
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("rclcpp"),"0 sec\nTM_ROS: (TM_SCT) connect(" << (int)sct_reconnect_timeout_ms_ << "ms)...");
        sct.connect_socket(sct_reconnect_timeout_ms_);
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
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),"TM_ROS: (re)connect(" << (int)t_o << ") TM_SCT");
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
            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),"TM_ROS: TM_SCT resume connection recovery");                     	
        }
        else
        {        	    	
            sct_reconnect_timeout_ms_ = t_o;
            sct_reconnect_timeval_ms_ = t_v;
        }
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),"TM_ROS: set TM_SCT reconnect timeout " << (int)sct_reconnect_timeout_ms_ << "ms, timeval " << (int)sct_reconnect_timeval_ms_ << "ms");
    }
    else {
        // no reconnect
        sct_reconnect_timeval_ms_ = -1;
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),"TM_ROS: set TM_SCT NOT reconnect");
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
