#ifdef NO_INCLUDE_DIR
#include "tm_svr_communication.h"
#include "tm_print.h"
#else
#include "tm_driver/tm_svr_communication.h"
#include "tm_driver/tm_print.h"
#endif

#include <functional>

//
// TmSvrCommunication
//

TmSvrCommunication::TmSvrCommunication(const std::string &ip,
	int recv_buf_len, std::condition_variable *cv)
	: TmCommunication(ip.c_str(), 5891, recv_buf_len)
{
	if (cv) {
		_cv = cv;
		_has_thread = true;
	}
}
TmSvrCommunication::~TmSvrCommunication()
{
	halt();
}

bool TmSvrCommunication::start(int timeout_ms)
{
	if (socket_description() == 6188)
	{
		print_info("TM_SVR: start (fake)");
		if (_has_thread) {
			// start thread
			_recv_thread = std::thread(std::bind(&TmSvrCommunication::thread_function, this));
		}
		return true;
	}

	halt();
	print_info("TM_SVR: start");

	bool rb = Connect(timeout_ms);
	//if (!rb) return rb; // ? start thread anyway

	if (_has_thread) {
		// start thread
		_recv_thread = std::thread(std::bind(&TmSvrCommunication::thread_function, this));
	}
	return rb;
}
void TmSvrCommunication::halt()
{
	if (socket_description() == 6188)
	{
		print_info("TM_SVR: halt (fake)");
		if (_has_thread) {
			_keep_thread_alive = false;
			if (_recv_thread.joinable()) {
				_recv_thread.join();
			}
		}
		return;
	}
	if (_has_thread) {
		_keep_thread_alive = false;
		if (_recv_thread.joinable()) {
			_recv_thread.join();
		}
	}
	if (is_connected()) {
		print_info("TM_SVR: halt");
		//if (_has_thread) {
		//	_keep_thread_alive = false;
		//	if (_recv_thread.joinable()) {
		//		_recv_thread.join();
		//	}
		//}
		Close();
	}
}

TmCommRC TmSvrCommunication::send_content_str(const std::string &id, const std::string &content)
{
	std::string cntt = content;
	TmSvrData cmd{ id, TmSvrData::Mode::STRING, cntt.data(), cntt.size(), TmSvrData::SrcType::Move };
	TmPacket pack{ cmd };
	return send_packet_all(pack);
}
TmCommRC TmSvrCommunication::send_play_cmd()
{
	return send_content_str("0", "Stick_PlayPause=1\r\n");
}

void TmSvrCommunication::thread_function()
{
	print_info("TM_SVR: thread begin");
	_keep_thread_alive = true;
	while (_keep_thread_alive) {
		bool reconnect = false;
		while (_keep_thread_alive && is_connected() && !reconnect) {
			TmCommRC rc = tmsvr_function();
			_updated = true;
			_cv->notify_all();

			switch (rc) {
			case TmCommRC::ERR:
			case TmCommRC::NOTREADY:
			case TmCommRC::NOTCONNECT:
				print_info("TM_SVR: rc=%d", int(rc));
				reconnect = true;
				break;
			default: break;
			}
		}
		Close();
		print_info("TM_SVR: reconnect in ");
		int cnt = 5;
		while (_keep_thread_alive && cnt > 0) {
			print_info("%d sec...", cnt);
			std::this_thread::sleep_for(std::chrono::seconds(1));
			--cnt;
		}
		if (_keep_thread_alive) {
			print_info("TM_SVR: connect...");
			Connect(1000);
		}
	}
	Close();
	print_info("TM_SVR: thread end");
}
TmCommRC TmSvrCommunication::tmsvr_function()
{
	TmCommRC rc;
	int n;
	rc = recv_spin_once(1000, &n);
	if (rc != TmCommRC::OK) {
		return rc;
	}
	std::vector<TmPacket> &pack_vec = packet_list();

	for (auto &pack : pack_vec) {
		if (pack.type == TmPacket::Header::CPERR) {
			print_info("TM_SVR: CPERR");
			err_data.set_CPError(pack.data.data(), pack.data.size());
			print_error(err_data.error_code_str().c_str());
		}
		else if (pack.type == TmPacket::Header::TMSVR) {
			
			err_data.error_code(TmCPError::Code::Ok);

			TmSvrData::build_TmSvrData(data, pack.data.data(), pack.data.size(), TmSvrData::SrcType::Move);
			
			if (data.is_valid()) {
				switch (data.mode()) {
				case TmSvrData::Mode::RESPONSE:
					print_info("TM_SVR: response (%d)", int(data.error_code()));
					break;
				case TmSvrData::Mode::BINARY:
					state.mtx_deserialize(data.content(), data.content_len());
					break;
				default:
					print_info("TM_SVR: invalid mode (%d)", int(data.mode()));
					break;
				}
			}
			else {
				print_info("TM_SVR: invalid data");
			}
		}
		else {
			print_info("TM_SVR: invalid header");
		}
	}
	return rc;
}