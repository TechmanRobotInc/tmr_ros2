#pragma once
#include "tmr_communication.h"
#include "tmr_robot_state.h"

namespace tmr
{

class TmSvrCommunication : public Communication
{
private:
	std::condition_variable *_cv = nullptr;
	std::thread _recv_thread;
	bool _keep_thread_alive = false;
	bool _has_thread = false;

private:
	bool _updated = false;

	int _reconnect_timeout_ms = 1000;
	int _reconnect_timeval_ms = 3000;

public:
	TmCPError err_data{ TmCPError::Code::Ok };
	TmSvrData data;
	RobotState state;

private:
	std::mutex _mtx;

public:
	explicit TmSvrCommunication(const std::string &ip,
		int recv_buf_len, std::condition_variable *cv = nullptr);
	~TmSvrCommunication();

	bool start(int timeout_ms);
	void halt();

	void set_reconnect_timeout(int timeout_ms)
	{ _reconnect_timeout_ms = timeout_ms; }
	void set_reconnect_timeval(int timeval_ms)
	{ _reconnect_timeval_ms = timeval_ms; }

	CommRC send_content(const std::string &id, TmSvrData::Mode mode, const std::string &content);
	CommRC send_content_str(const std::string &id, const std::string &content);

	CommRC send_stick_play();
	//TmCommRC send_stop_cmd();

	bool state_updated() { return _updated; }
	void reset_state_update() { _updated = false; }

	TmSvrData::ErrorCode error_code() { return data.error_code(); }
	std::string error_content()
	{
		if (data.content() && data.content_len() > 0) {
			return std::string(data.content(), data.content_len());
		}
		else return std::string();
	}
private:
	void thread_function();
	void reconnect_function();
public:
	CommRC tmsvr_function();
};

}