#include "tm_driver/tm_listen_node_connect.h"

ListenNodeConnection::ListenNodeConnection
  (TmDriver &iface,std::function<void(TmSctData)> sct_msg, std::function<void(std::string, std::string)> sta_msg, bool is_fake_)
  :iface(iface), sct_(iface.sct),sct_msg(sct_msg),sta_msg(sta_msg)
  {
  isRun = true;
  if (!is_fake_) {
      sct_.start_tm_sct(5000);
  }
  listenNodeThread = std::thread(std::bind(&ListenNodeConnection::listen_node_connect, this));
  checkListenNodeThread = std::thread(std::bind(&ListenNodeConnection::check_is_on_listen_node, this));
  
}
void ListenNodeConnection::build_sta_cmd(){
    TmStaData &data = sct_.sta_data;
    {
        std::lock_guard<std::mutex> lck(sta_mtx_);
        staSubcmd = data.subcmd_str();
        staSubdata = std::string{ data.subdata(), data.subdata_len() };
        sta_updated_ = true;
    }
    sta_cv_.notify_all();

    print_info("TM_ROS: (TM_STA): res: (%s): %s", staSubcmd.c_str(), staSubdata.c_str());
    sta_msg(staSubcmd,staSubdata);
}
bool ListenNodeConnection::send_data(){
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

            sct_msg(sct_.sct_data);
            break;

        case TmPacket::Header::TMSTA:

            sct.tmSctErrData.error_code(TmCPError::Code::Ok);

            TmStaData::build_TmStaData(sct.sta_data, pack.data.data(), pack.data.size(), TmStaData::SrcType::Shallow);
            
            build_sta_cmd();
            break;

        default:
            print_error("TM_ROS: (Listen node): invalid header");
            break;
        }
    }
    return true;
}
void ListenNodeConnection::sct_connect_recover()
{
    TmSctCommunication &sct = sct_;
    int timeInterval = 0;
    int lastTimeInterval=1000;
            	
    if (sct_reconnect_timeval_ms_ <= 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    print_info("TM_ROS: (Listen node): Reconnecting...");

    uint64_t startTimeMs = TmCommunication::get_current_time_in_ms();
    while (isRun && timeInterval < sct_reconnect_timeval_ms_) {
        if ( lastTimeInterval/1000 != timeInterval/1000) {
            print_debug("Listen node reconnect remain : %.1f sec...", 0.001 * (sct_reconnect_timeval_ms_ - timeInterval));
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        lastTimeInterval = timeInterval;
        timeInterval = TmCommunication::get_current_time_in_ms() - startTimeMs;
    }
    if (isRun && sct_reconnect_timeval_ms_ >= 0) {
        print_debug("0 sec\nTM_ROS: (Listen node) connect(%d)...", (int)sct_reconnect_timeout_ms_);
        sct.connect_socket("Listen node",sct_reconnect_timeout_ms_);
    }
}
void ListenNodeConnection::listen_node_connect(){

    TmSctCommunication &sct = sct_;

    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    print_info("TM_ROS: sct_response thread begin");

    while (isRun) {
        //bool reconnect = false;
        if (iface.get_connect_recovery_guide()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        else   
        {
            if (!sct.recv_init()) {
                print_debug("TM_ROS: (Listen node): is not connected");
            }
            firstEnter = true;
            while (isRun && sct.is_connected() && iface.svr.is_connected()) {
                if(firstEnter){
                    checkIsOnListenNodeCondVar.notify_one();
                    firstEnter = false;
                }
                if (!send_data()) break;
            }
            sct.close_socket();
            if (!isRun) break;
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
void ListenNodeConnection::check_is_on_listen_node(){
    std::unique_lock<std::mutex> firstCheckIsOnListenNodeLock(firstCheckIsOnListenNodeMutex);
    std::unique_lock<std::mutex> checkIsOnListenNodeLock(checkIsOnListenNodeMutex);
    
    firstCheckIsOnListenNodeCondVar.wait(firstCheckIsOnListenNodeLock);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));

    while (isRun){
        std::string reSubcmd;
        std::string reSubdata;
        ask_sta_struct("00","",1,reSubcmd,reSubdata);
        bool isInListenNode = false;

        std::istringstream(reSubdata) >> std::boolalpha >> isInListenNode;
    
        if(isInListenNode){
            print_info("TM_ROS: On listen node.");
            iface.back_to_listen_node();
        } else{
            print_info("TM_ROS: Not on listen node!");
        }
        checkIsOnListenNodeCondVar.wait(checkIsOnListenNodeLock);
    }
}
bool ListenNodeConnection::connect_tmsct(int timeout, int timeval, bool connect, bool reconnect)
{
    bool rb = true;
    int t_o = (int)(1000.0 * timeout);
    int t_v = (int)(1000.0 * timeval);
    if (connect) {
        print_info("TM_ROS: (re)connect(%d) TM_SCT Listen node", (int)t_o);
        sct_.halt();
        rb = sct_.start_tm_sct(t_o);
    }
    if (reconnect) {
        if (iface.get_connect_recovery_guide())
        {        	
            sct_reconnect_timeout_ms_ = 1000;
            sct_reconnect_timeval_ms_ = 3000;
            iface.set_connect_recovery_guide(false);
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
    return rb;
}
bool ListenNodeConnection::send_listen_node_script(const std::string id, const std::string script){
    return (sct_.send_script_str(id, script) == iface.RC_OK);
}
bool ListenNodeConnection::ask_sta_struct(std::string subcmd, std::string subdata, double waitTime,std::string &reSubcmd, std::string &reSubdata){
    
    bool rb = false;

    sta_mtx_.lock();
    sta_updated_ = false;
    sta_mtx_.unlock();

    rb = (sct_.send_sta_request(subcmd, subdata) == iface.RC_OK);

    {
        std::unique_lock<std::mutex> lck(sta_mtx_);
        if (rb && waitTime > 0.0) {
            if (!sta_updated_) {
                sta_cv_.wait_for(lck, std::chrono::duration<double>(waitTime));
            }
            if (!sta_updated_) {
                rb = false;
            }
            reSubcmd = staSubcmd;
            reSubdata = staSubdata;
        }
        sta_updated_ = false;
    }

    return rb;
}
void ListenNodeConnection::check_is_on_listen_node_from_script(std::string id, std::string script){
    std::string idzero = "0";
    std::string ok = "OK";
    std::string errorString = "ERROR";
    if(idzero.compare(id)==0 && errorString.compare(script)!=0 &&  ok.compare(script)!=0){
        iface.back_to_listen_node();
    }
}
ListenNodeConnection::~ListenNodeConnection(){
    print_info("TM_ROS: (Listen node) halt");
    isRun = false;		
    sta_updated_ = true;
    sta_cv_.notify_all();
    firstCheckIsOnListenNodeCondVar.notify_all();
    checkIsOnListenNodeCondVar.notify_all();
    if (sct_.is_connected()) {}
    sct_.halt();
}
