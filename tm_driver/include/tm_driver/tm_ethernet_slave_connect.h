#include "tm_driver.h"

class EthernetSlaveConnection{
 private:
  TmSvrCommunication &svr_;
  TmRobotState &state_;

  TmSctCommunication &sct_;
  TmDriver &iface_;
  bool run;
  std::thread getDataThread;
  std::function<void(void)> publish_svr;
  int pub_reconnect_timeout_ms_;
  int pub_reconnect_timeval_ms_;
  int diconnectTimes = 0;
  uint64_t initialNotConnectTime = 0;
  uint64_t notConnectTimeInS = 0;
  int maxTrialTimeInMinute = -1;
  uint64_t maxNotConnectTimeInS = 0;

  bool get_data_function();
  void get_data_thread();
  void svr_connect_recover();
  void cq_manage();
  bool rc_halt();
  void cq_monitor();
public:
  EthernetSlaveConnection(TmDriver& iface,std::function<void(void)> ethernet_cmd_come,bool stick_play);
  ~EthernetSlaveConnection();
  void renew_all_data();
  void set_reconnect_time(int reconnect_timeout_ms,int reconnect_timeval_ms);
  bool connect(int timeout);
  bool re_connect(int timeout_ms, int timeval_ms);
  void no_connect();
};

