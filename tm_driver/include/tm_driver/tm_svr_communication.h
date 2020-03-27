#pragma once
#include "tm_communication.h"
#include "tm_robot_state.h"

class TmSvrCommunication : public TmCommunication
{
private:
	std::condition_variable *_cv = nullptr;
	std::thread _recv_thread;
	bool _keep_thread_alive = false;
	bool _has_thread = false;

private:
	bool _updated = false;

public:
	TmCPError err_data{ TmCPError::Code::Ok };
	TmSvrData data;
	TmRobotState state;

public:
	explicit TmSvrCommunication(const std::string &ip,
		int recv_buf_len, std::condition_variable *cv = nullptr);
	~TmSvrCommunication();

	bool start(int timeout_ms);
	void halt();

	TmCommRC send_content_str(const std::string &id, const std::string &content);
	TmCommRC send_play_cmd();
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
public:
	TmCommRC tmsvr_function();
};
