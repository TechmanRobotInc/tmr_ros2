#ifdef NO_INCLUDE_DIR
#include "tm_sct_communication.h"
#include "tm_print.h"
#else
#include "tm_driver/tm_sct_communication.h"
#include "tm_driver/tm_print.h"
#endif

#include <functional>

//
// TmSctCommunication
//

TmSctCommunication::TmSctCommunication(const std::string &ip,
	int recv_buf_len, std::condition_variable *cv)
	: TmCommunication(ip.c_str(), 5890, recv_buf_len)
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
	print_info("TM_SCT: start");

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
	if (_has_thread) {
		_keep_thread_alive = false;
		if (_recv_thread.joinable()) {
			_recv_thread.join();
		}
	}
	if (is_connected()) {
		print_info("TM_SCT: halt");
		//if (_has_thread) {
		//	_keep_thread_alive = false;
		//	if (_recv_thread.joinable()) {
		//		_recv_thread.join();
		//	}
		//}
		Close();
	}
}

TmCommRC TmSctCommunication::send_script_str(const std::string &id, const std::string &script)
{
	std::string sct = script;
	TmSctData cmd{ id, sct.data(), sct.size(), TmSctData::SrcType::Move };
	TmPacket pack{ cmd };
	return send_packet_all(pack);
}
TmCommRC TmSctCommunication::send_script_exit_cmd()
{
	return send_script_str("0", "ScriptExit()");
}

void TmSctCommunication::thread_function()
{
	print_info("TM_SCT: thread begin");
	_keep_thread_alive = true;
	while (_keep_thread_alive) {
		bool reconnect = false;
		while (_keep_thread_alive && is_connected() && !reconnect) {
			TmCommRC rc = tmsct_function();
			switch (rc) {
			case TmCommRC::ERR:
			case TmCommRC::NOTREADY:
			case TmCommRC::NOTCONNECT:
				print_info("TM_SCT: rc=%d", int(rc));
				reconnect = true;
				break;
			default: break;
			}
		}
		Close();
		if (_keep_thread_alive) {
			print_info("TM_SCT: reconnect in ");
		}
		int cnt = 5;
		while (_keep_thread_alive && cnt > 0) {
			print_info("%d sec...", cnt);
			std::this_thread::sleep_for(std::chrono::seconds(1));
			--cnt;
		}
		if (_keep_thread_alive) {
			print_info("TM_SCT: connect...");
			Connect(1000);
		}
	}
	Close();
	print_info("TM_SCT: thread end");
}
TmCommRC TmSctCommunication::tmsct_function()
{
	TmCommRC rc;
	int n;
	rc = recv_spin_once(1000, &n);
	if (rc != TmCommRC::OK) {
		return rc;
	}
	std::vector<TmPacket> &pack_vec = packet_list();

	for (auto &pack : pack_vec) {
		switch (pack.type) {
		case TmPacket::Header::CPERR:
			print_info("TM_SCT: CPERR");
			err_data.set_CPError(pack.data.data(), pack.data.size());
			print_error(err_data.error_code_str().c_str());
			break;
		case TmPacket::Header::TMSCT:
			//print_info("TM_SCT: TMSCT");
			err_data.error_code(TmCPError::Code::Ok);

			TmSctData::build_TmSctData(sct_data, pack.data.data(), pack.data.size(), TmSctData::SrcType::Move);
			
			mtx_sct_lock();
			_sct_response = std::string{ sct_data.script(), sct_data.script_len() };
			mtx_sct_unlock();
			if (sct_data.has_error()) {
				print_error("TM_SCT: err: %s", _sct_response.c_str());
			}
			else {
				print_info("TM_SCT: res: %s", _sct_response.c_str());
			}
			break;
		case TmPacket::Header::TMSTA:
			print_info("TM_SCT: TMSTA");
			break;
		default:
			print_info("TM_SCT: invalid header");
			break;
		}
	}
	return rc;
}