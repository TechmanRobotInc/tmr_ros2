#ifdef NO_INCLUDE_DIR
#include "tmr_tmsct_communication.h"
#include "tmr_print.h"
#else
#include "tmr/tmr_tmsct_communication.h"
#include "tmr/tmr_print.h"
#endif

#include <functional>

namespace tmr
{

//
// TmSctCommunication
//

TmSctCommunication::TmSctCommunication(const std::string &ip,
	int recv_buf_len, std::condition_variable *cv)
	: Communication(ip.c_str(), 5890, recv_buf_len)
{
	if (cv) {
		_cv = cv;
		_has_thread = true;
	}
}
TmSctCommunication::~TmSctCommunication()
{
	halt();
}

bool TmSctCommunication::start(int timeout_ms)
{
	halt();
	tmr_INFO_STREAM("TM_SCT: start");

	bool rb = Connect(timeout_ms);
	//if (!rb) return rb; // ? start thread anyway

	if (_has_thread) {
		// start thread
		_recv_thread = std::thread(std::bind(&TmSctCommunication::thread_function, this));
	}
	return rb;
}
void TmSctCommunication::halt()
{
	tmr_INFO_STREAM("TM_SCT: halt");
	if (_has_thread) {
		_keep_thread_alive = false;
		if (_recv_thread.joinable()) {
			_recv_thread.join();
		}
		_updated = true;
		_cv->notify_all();
	}
	if (is_connected()) {
		Close();
	}
}

CommRC TmSctCommunication::send_script_str(const std::string &id, const std::string &script)
{
	std::string sct = script;
	TmSctData cmd{ id, sct.data(), sct.size(), TmSctData::SrcType::Shallow };
	TmPacket pack{ cmd };
	return send_packet_all(pack);
}
CommRC TmSctCommunication::send_script_str_silent(const std::string &id, const std::string &script)
{
	std::string sct = script;
	TmSctData cmd{ id, sct.data(), sct.size(), TmSctData::SrcType::Shallow };
	TmPacket pack{ cmd };
	return send_packet_silent_all(pack);
}
CommRC TmSctCommunication::send_script_exit()
{
	return send_script_str("Exit", "ScriptExit()");
}

CommRC TmSctCommunication::send_sta_request(const std::string &subcmd, const std::string &subdata)
{
	std::string data = subdata;
	TmStaData req{ subcmd, data.data(), data.size(), TmStaData::SrcType::Shallow };
	TmPacket pack{ req };
	return send_packet_all(pack);
}

std::string TmSctCommunication::mtx_sct_response(std::string &id)
{
	std::string rs;
	{
		std::lock_guard<std::mutex> lck(mtx_sct);
		id = sct_data.script_id();
		rs = std::string{ sct_data.script(), sct_data.script_len() };
	}
	return rs;
}
std::string TmSctCommunication::mtx_sta_response(std::string &cmd)
{
	std::string rs;
	{
		std::lock_guard<std::mutex> lck(mtx_sta);
		cmd = sta_data.subcmd_str();
		rs = std::string{ sta_data.subdata(), sta_data.subdata_len() };
	}
	return rs;
}

void TmSctCommunication::thread_function()
{
	tmr_INFO_STREAM("TM_SCT: thread begin");
	_keep_thread_alive = true;
	while (_keep_thread_alive) {
		bool reconnect = false;
		if (!recv_init()) {
			tmr_INFO_STREAM("TM_SCT: is not connected");
		}
		while (_keep_thread_alive && is_connected() && !reconnect) {
			CommRC rc = tmsct_function();
			{
				std::lock_guard<std::mutex> lck(_mtx);
				_updated = true;
			}
			_cv->notify_all();

			switch (rc) {
			case CommRC::ERR:
			case CommRC::NOTREADY:
			case CommRC::NOTCONNECT:
				tmr_INFO_STREAM("TM_SCT: rc=" << int(rc));
				reconnect = true;
				break;
			default: break;
			}
		}
		Close();
		reconnect_function();
	}
	Close();
	tmr_INFO_STREAM("TM_SCT: thread end");
}
void TmSctCommunication::reconnect_function()
{
	if (!_keep_thread_alive) return;
	if (_reconnect_timeval_ms <= 0) {
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
	tmr_INFO_STREAM("TM_SCT: reconnect in ");
	int cnt = 0;
	while (_keep_thread_alive && cnt < _reconnect_timeval_ms) {
		if (cnt % 1000 == 0) {
			tmr_INFO_STREAM(0.001 * (_reconnect_timeval_ms - cnt) << " sec...");
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		++cnt;
	}
	if (_keep_thread_alive && _reconnect_timeval_ms >= 0) {
		tmr_INFO_STREAM("0 sec\nTM_SCT: connect(" << _reconnect_timeout_ms << "ms)...");
		Connect(_reconnect_timeout_ms);
	}
}
CommRC TmSctCommunication::tmsct_function()
{
	CommRC rc;
	int n;
	rc = recv_spin_once(1000, &n);
	if (rc != CommRC::OK) {
		return rc;
	}
	std::vector<TmPacket> &pack_vec = packet_list();

	//TmCPError err_data_tmp;
	TmSctData sct_data_tmp;
	TmStaData sta_data_tmp;

	for (auto &pack : pack_vec) {
		switch (pack.type) {
		case TmPacket::Header::CPERR:
			tmr_INFO_STREAM("TM_SCT: CPERR");
			err_data.set_CPError(pack.data.data(), pack.data.size());
			tmr_ERROR_STREAM(err_data.error_code_str());
			break;

		case TmPacket::Header::TMSCT:
			//tmr_INFO_STREAM("TM_SCT: TMSCT");
			err_data.error_code(TmCPError::Code::Ok);

			/*TmSctData::build_TmSctData(sct_data, pack.data.data(), pack.data.size(), TmSctData::SrcType::Shallow);
			{
				std::lock_guard<std::mutex> lck(mtx_sct);
				_sct_res_id = sct_data.script_id();
				_sct_res_script = std::string{ sct_data.script(), sct_data.script_len() };
			}
			if (sct_data.has_error()) {
				tmr_ERROR_STREAM("TM_SCT: err: (" << _sct_res_id << "): " << _sct_res_script);
			}
			else {
				tmr_INFO_STREAM("TM_SCT: res: (" << _sct_res_id << "): " << _sct_res_script);
			}*/

			TmSctData::build_TmSctData(sct_data_tmp, pack.data.data(), pack.data.size(), TmSctData::SrcType::Shallow);
			{
				std::lock_guard<std::mutex> lck(mtx_sct);
				TmSctData::build_TmSctData(sct_data, sct_data_tmp, TmSctData::SrcType::Deep);
			}
			if (sct_data.has_error())
				tmr_INFO_STREAM("TM_SCT: err: (" << sct_data.script_id() << "): " << sct_data.script());
			else
				tmr_INFO_STREAM("TM_SCT: res: (" << sct_data.script_id() << "): " << sct_data.script());

			break;

		case TmPacket::Header::TMSTA:
			//tmr_INFO_STREAM("TM_SCT: TMSTA");
			err_data.error_code(TmCPError::Code::Ok);

			TmStaData::build_TmStaData(sta_data_tmp, pack.data.data(), pack.data.size(), TmStaData::SrcType::Shallow);
			{
				std::lock_guard<std::mutex> lck(mtx_sta);
				TmStaData::build_TmStaData(sta_data, sta_data_tmp, TmStaData::SrcType::Deep);
			}
			tmr_INFO_STREAM("TM_STA: res: (" << sta_data.subcmd_str() << "): " << sta_data.subdata());

			tmsta_function();
			break;

		default:
			tmr_WARN_STREAM("TM_SCT: invalid header");
			break;
		}
	}
	return rc;
}
void TmSctCommunication::tmsta_function()
{
	switch (sta_data.subcmd()) {
	case 0:
		break;
	case 1:
		break;
	}
}

}