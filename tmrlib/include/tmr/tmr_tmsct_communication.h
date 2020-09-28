#pragma once
#include "tmr_communication.h"

namespace tmr
{

class TmSctCommunication : public Communication
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
	TmSctData sct_data;
	TmStaData sta_data;

private:
	std::mutex _mtx;
public:
	std::mutex mtx_cpe;
	std::mutex mtx_sct;
	std::mutex mtx_sta;

	//std::string _sct_res_id;
	//std::string _sct_res_script;

	//std::string _sta_res_subcmd_str;
	//std::string _sta_res_subdata;

public:
	explicit TmSctCommunication(const std::string &ip,
		int recv_buf_len, std::condition_variable *cv = nullptr);
	~TmSctCommunication();

	bool start(int timeout_ms = 0);
	void halt();

	void set_reconnect_timeout(int timeout_ms)
	{ _reconnect_timeout_ms = timeout_ms; }
	void set_reconnect_timeval(int timeval_ms)
	{ _reconnect_timeval_ms = timeval_ms; }

	CommRC send_script_str(const std::string &id, const std::string &script);
	CommRC send_script_str_silent(const std::string &id, const std::string &script);
	CommRC send_script_exit();

	CommRC send_sta_request(const std::string &subcmd, const std::string &subdata);

public:
	TmCPError::Code cperr_code() { return err_data.error_code(); }
	std::string sct_response(std::string &id)
	{
		id = sct_data.script_id();
		return std::string{ sct_data.script(), sct_data.script_len() };
	}
	std::string sta_response(std::string &cmd)
	{
		cmd = sta_data.subcmd_str();
		return std::string{ sta_data.subdata(), sta_data.subdata_len() };
	}

public:
	std::string mtx_sct_response(std::string &id);
	std::string mtx_sta_response(std::string &cmd);

private:
	void thread_function();
	void reconnect_function();
public:
	CommRC tmsct_function();
private:
	void tmsta_function();
};

}