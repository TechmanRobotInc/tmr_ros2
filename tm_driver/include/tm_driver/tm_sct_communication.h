#pragma once
#include "tm_communication.h"

class TmSctCommunication : public TmCommunication
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
	TmCPError tmSctErrData{ TmCPError::Code::Ok };
	TmSctData sct_data;
	TmStaData sta_data;

private:
	std::mutex _mtx;
	std::mutex mtx_sct;
	std::mutex mtx_sta;
    
	bool &isOnListenNode;

	//std::string _sct_res_id;
	//std::string _sct_res_script;

	//std::string _sta_res_subcmd_str;
	//std::string _sta_res_subdata;

public:
	explicit TmSctCommunication(const std::string &ip,
		int recv_buf_len,bool &isOnListenNode, std::condition_variable *cv = nullptr);
	~TmSctCommunication();

	bool start_tm_sct(int timeout_ms = 0);
	void halt();

	void set_reconnect_timeout(int timeout_ms)
	{ _reconnect_timeout_ms = timeout_ms; }
	void set_reconnect_timeval(int timeval_ms)
	{ _reconnect_timeval_ms = timeval_ms; }
    void check_script_is_exit(std::string script);
	TmCommRC send_script_str(const std::string &id, const std::string &script);
	TmCommRC send_script_str_silent(const std::string &id, const std::string &script);
	TmCommRC send_script_exit();

	TmCommRC send_sta_request(const std::string &subcmd, const std::string &subdata);

public:
	TmCPError::Code cperr_code() { return tmSctErrData.error_code(); }
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
	void mtx_sct_lock() { mtx_sct.lock(); }
	void mtx_sct_unlock() { mtx_sct.unlock(); }

	void mtx_sta_lock() { mtx_sta.lock(); }
	void mtx_sta_unlock() { mtx_sta.unlock(); }

	std::string mtx_sct_response(std::string &id);
	std::string mtx_sta_response(std::string &cmd);

private:
	void tm_sct_thread_function();
	void reconnect_function();
public:
	TmCommRC tmsct_function();
private:
	void tmsta_function();
};
