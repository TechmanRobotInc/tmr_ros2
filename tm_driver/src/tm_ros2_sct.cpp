#include "tm_driver/tm_ros2_sct.h"

TmSctRos2::TmSctRos2(rclcpp::Node::SharedPtr node, TmDriver &iface, bool is_fake)
    : node(node)
    , iface_(iface)
    , is_fake_(is_fake)
{

    jns_.clear();
    jns_.push_back("joint_1");
    jns_.push_back("joint_2");
    jns_.push_back("joint_3");
    jns_.push_back("joint_4");
    jns_.push_back("joint_5");
    jns_.push_back("joint_6");
    
    if (!is_fake_) {
        listenNodeConnection = std::make_unique<ListenNodeConnection>(iface_,
      std::bind(&TmSctRos2::sct_msg,this,std::placeholders::_1),
      std::bind(&TmSctRos2::sta_msg,this,std::placeholders::_1,std::placeholders::_2),
      is_fake_);
        sm_.sct_pub = node->create_publisher<tm_msgs::msg::SctResponse>("sct_response", 1);
        sm_.sta_pub = node->create_publisher<tm_msgs::msg::StaResponse>("sta_response", 1);
    }

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

TmSctRos2::~TmSctRos2(){
    print_info("TM_ROS: (Listen node) halt");		

    sta_updated_ = true;
        
}
void TmSctRos2::sct_msg(TmSctData data)
{
    SctAndStaMsg &sm = sm_;
    
    sm.sct_msg.id = data.script_id();
    sm.sct_msg.script = std::string{ data.script(), data.script_len() };

    listenNodeConnection->check_is_on_listen_node_from_script(sm.sct_msg.id, sm.sct_msg.script);

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

void TmSctRos2::sta_msg(std::string subcmd, std::string subdata)
{
    SctAndStaMsg &sm = sm_;
    
    sm.sta_msg.subcmd = subcmd;
    sm.sta_msg.subdata = subdata;

    sm.sta_msg.header.stamp = node->rclcpp::Node::now();
    sm.sta_pub->publish(sm.sta_msg);
}
bool TmSctRos2::connect_tmsct(
        const std::shared_ptr<tm_msgs::srv::ConnectTM::Request> req,
        std::shared_ptr<tm_msgs::srv::ConnectTM::Response> res)
{
    res->ok = listenNodeConnection->connect_tmsct(req->timeout, req->timeval, req->connect, req->reconnect);
    return res->ok;
}

bool TmSctRos2::send_script(
    const std::shared_ptr<tm_msgs::srv::SendScript::Request> req,
    std::shared_ptr<tm_msgs::srv::SendScript::Response> res)
{
    bool rb = listenNodeConnection->send_listen_node_script(req->id, req->script);
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
        std::shared_ptr<tm_msgs::srv::AskSta::Response> res){
    res->ok = listenNodeConnection->ask_sta_struct(req->subcmd, req->subdata, req->wait_time, res->subcmd, res->subdata);
    return res->ok;
}
