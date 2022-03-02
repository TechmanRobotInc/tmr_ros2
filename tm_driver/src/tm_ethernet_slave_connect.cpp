#include "tm_driver/tm_ethernet_slave_connect.h"

EthernetSlaveConnection::EthernetSlaveConnection(TmDriver& iface,std::function<void(void)> ethernet_cmd_come, bool stick_play) :
    svr_(iface.svr)
    , state_(iface.state)
    , sct_(iface.sct)
    , iface_(iface)
    , publish_svr(ethernet_cmd_come){

  
  bool rb = iface_.svr.start_tm_svr(5000);
  if (rb && stick_play) {
    svr_.send_stick_play();
  }
  
  
  run = true;
  
  getDataThread = std::thread(std::bind(&EthernetSlaveConnection::get_data_thread, this));
  
  pub_reconnect_timeout_ms_ = 1000;
  pub_reconnect_timeval_ms_ = 3000;
}
bool EthernetSlaveConnection::get_data_function()
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
    bool hasCompletePackage = false;
    for (auto &pack : pack_vec) {
        if (pack.type == TmPacket::Header::CPERR) {
            svr.tmSvrErrData.set_CPError(pack.data.data(), pack.data.size());
            print_error("TM_ROS: (Ethernet slave) ROS Node Header CPERR %d", (int)svr.tmSvrErrData.error_code());
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
                    //print_info("TM_ROS: (Ethernet slave): (%s) RESPONSE [%d]",
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
                    print_error("TM_ROS: (Ethernet slave): (%s): invalid mode (%d)",
                        svr.data.transaction_id().c_str(), (int)(svr.data.mode()));
                    break;
                }
            }else {   
                print_error("TM_ROS: (Ethernet slave): invalid data");
            }
            hasCompletePackage = true;
        } else if(pack.type == TmPacket::Header::EMPTY){
            if(hasCompletePackage){
                print_warn("warnning TM_ROS: (Ethernet slave): receive package header may not complete, but some data formate is correct, please check your network");
            } else{
                print_error("TM_ROS: (Ethernet slave): receive header EMPTY, invalid header");
            }
        } else if(pack.type == TmPacket::Header::PACKAGE_INCOMPLETE) {
            if(hasCompletePackage){
                print_warn("warnning TM_ROS: (Ethernet slave): receive package is not complete, but some data formate is correct, please check your network");
            }  else{
                print_warn("warnning TM_ROS: (Ethernet slave): receive package is not complete, please check your network");
            }
        } else if(pack.type == TmPacket::Header::TMSCT){
            print_error("TM_ROS: (Ethernet slave): receive header TMSCT, invalid header");
        } else if(pack.type == TmPacket::Header::TMSTA){
            print_error("TM_ROS: (Ethernet slave): receive header TMSTA, invalid header");
        } else if(pack.type == TmPacket::Header::OTHER){
            print_error("TM_ROS: (Ethernet slave): receive header OTHER, invalid header");
        } else {
            print_error("TM_ROS: (Ethernet slave): receive invalid header");
        }
    }
    if (fbs) {
        state_.set_receive_state(rc);
    }
    if(rc == TmCommRC::TIMEOUT){
      print_once("TM_ROS: (Ethernet slave): LINK TIMEOUT");
      return false;
    }
    return true;
}
void EthernetSlaveConnection::svr_connect_recover()
{
    TmSvrCommunication &svr = svr_;
    int timeInterval = 0;
    int lastTimeInterval = 1000;
    	    	
    if (pub_reconnect_timeval_ms_ <= 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    print_info("TM_ROS: (Ethernet slave): Reconnecting...");

    uint64_t startTimeMs = TmCommunication::get_current_time_in_ms();
    while (run && timeInterval < pub_reconnect_timeval_ms_) {
        if ( lastTimeInterval/1000 != timeInterval/1000) {
            print_debug("Ethernet slave reconnect remain : %.1f sec...", 0.001 * (pub_reconnect_timeval_ms_ - timeInterval));
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        lastTimeInterval = timeInterval;
        timeInterval = TmCommunication::get_current_time_in_ms() - startTimeMs;
    }    
    if (run && pub_reconnect_timeval_ms_ >= 0) {
        print_debug("0 sec\nTM_ROS: (Ethernet slave): connect(%dms)...", (int)pub_reconnect_timeout_ms_);
        svr.connect_socket("Ethernet slave",pub_reconnect_timeout_ms_);
    }
}
void EthernetSlaveConnection::cq_manage(){
    
    notConnectTimeInS = (TmCommunication::get_current_time_in_ms() - initialNotConnectTime)/1000;
    if(diconnectTimes == 0){
        return;
    }
    if(notConnectTimeInS>maxNotConnectTimeInS){
        maxNotConnectTimeInS = notConnectTimeInS;
    }
}
bool EthernetSlaveConnection::rc_halt(){  //Stop rescue connection
    if(maxTrialTimeInMinute == -1){
        return false;
    }
    if((int)(notConnectTimeInS/60) >= maxTrialTimeInMinute){
        print_debug("TM_ROS: (Ethernet slave): notConnectTimeInS = %d, maxTrialTimeInMinute = %d ", (int)notConnectTimeInS, (int)maxTrialTimeInMinute);
        return true;
    }
    return false;
}
void EthernetSlaveConnection::cq_monitor(){  //Connection quality
    diconnectTimes ++;
    initialNotConnectTime =  TmCommunication::get_current_time_in_ms();
}
void EthernetSlaveConnection::get_data_thread()
{
    TmSvrCommunication &svr = svr_;

    print_info("TM_ROS: get data thread begin");
    initialNotConnectTime =  TmCommunication::get_current_time_in_ms();
    while (run) {
        //bool reconnect = false;
        if (iface_.get_connect_recovery_guide()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        else   
        {	        	
            if (!svr.recv_init()) {
                print_debug("TM_ROS: (Ethernet slave): is not connected");
                cq_manage();
                if(rc_halt()) {
                    print_fatal("TM_ROS: (Ethernet slave): Ethernet slave connection stopped!");
                    if (diconnectTimes == 0)
                    {
                        print_warn("TM_ROS: (Ethernet slave): Please Check Wired Connected Settings");
                    }
                    else
                    {                    
                        print_fatal("TM_ROS: (Ethernet slave): LinkLost = %d , MaxLostTime(s) = %d", (int)diconnectTimes, (int)maxNotConnectTimeInS);
                    }
                    iface_.set_connect_recovery_guide(true);
                    svr.close_socket();
                }
            }
            if (!iface_.get_connect_recovery_guide()) {
                while (run && svr.is_connected()) {
                    if (!get_data_function()){
                        cq_monitor();
                        break;
                    }
                }
                        
                svr.close_socket();
                if (!run) break;
                svr_connect_recover();
            }
        } 
    }
    svr.close_socket();
    print_info("TM_ROS: publisher thread end\n");
}
void EthernetSlaveConnection::renew_all_data(){
    state_.update_tm_robot_publish_state();
  
}
void EthernetSlaveConnection::set_reconnect_time(int reconnect_timeout_ms,int reconnect_timeval_ms){
    pub_reconnect_timeout_ms_ = reconnect_timeout_ms;
    pub_reconnect_timeval_ms_ = reconnect_timeval_ms;
}
bool EthernetSlaveConnection::connect(int timeout){
    print_info("TM_ROS: (re)connect(%d) Ethernet slave", (int)timeout);
    svr_.halt();
    return svr_.start_tm_svr(timeout);
}
bool EthernetSlaveConnection::re_connect(int timeout_ms, int timeval_ms){
    bool rb = true;
    if (iface_.get_connect_recovery_guide()){
        pub_reconnect_timeout_ms_ = 1000;
        pub_reconnect_timeval_ms_ = 3000;
        initialNotConnectTime =  TmCommunication::get_current_time_in_ms();
        notConnectTimeInS = 0;
        diconnectTimes = 0;
        maxNotConnectTimeInS = 0;
        iface_.set_connect_recovery_guide(false);
        rb = svr_.start_tm_svr(5000);
        print_info("TM_ROS: Ethernet slave resume connection recovery");
    } else {
        pub_reconnect_timeout_ms_ = timeout_ms;
        pub_reconnect_timeval_ms_ = timeval_ms;
    }	
    print_info("TM_ROS: set Ethernet slave reconnect timeout %dms, timeval %dms", (int)pub_reconnect_timeout_ms_, (int)pub_reconnect_timeval_ms_);
    return rb;
}
void EthernetSlaveConnection::no_connect(){
    
    pub_reconnect_timeval_ms_ = -1;
    print_info("TM_ROS: set Ethernet slave NOT reconnect");
}
EthernetSlaveConnection::~EthernetSlaveConnection(){
    if (svr_.is_connected()) {}
    svr_.halt();
    run = false;
}
