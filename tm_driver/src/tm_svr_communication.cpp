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
	print_info("Ethernet slave communication: TmSvrCommunication");
	if (cv) {
		_cv = cv;
		_has_thread = true;
	}
}

TmSvrCommunication::~TmSvrCommunication()
{
	halt();
}

bool TmSvrCommunication::start_tm_svr(int timeout_ms)
{
	if (socket_description() == 6188)
	{
	    print_info("Ethernet slave communication: start (fake)");
		if (_has_thread) {
			// start thread
			_recv_thread = std::thread(std::bind(&TmSvrCommunication::tm_svr_thread_function, this));
		}
		return true;
	}

	halt();
	print_info("Ethernet slave communication: start");

	bool rb = connect_socket("Ethernet slave communication",timeout_ms);
	//if (!rb) return rb; // ? start thread anyway

	if (_has_thread) {
		// start thread
		_recv_thread = std::thread(std::bind(&TmSvrCommunication::tm_svr_thread_function, this));
	}
	return rb;
}

void TmSvrCommunication::halt()
{
	if (socket_description() == 6188)
	{
		print_info("Ethernet slave communication: halt (fake)");
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
		_updated = true;
		_cv->notify_all();
	}
	if (is_connected()) {
		print_info("Ethernet slave communication: halt");
		close_socket();
	}

	//_cv->notify_all();
	
}

TmCommRC TmSvrCommunication::send_content(const std::string &id, TmSvrData::Mode mode, const std::string &content)
{
	std::string cntt = content;
	TmSvrData cmd{ id, mode, cntt.data(), cntt.size(), TmSvrData::SrcType::Shallow };
	TmPacket pack{ cmd };
	return send_packet_all(pack);
}

TmCommRC TmSvrCommunication::send_content_str(const std::string &id, const std::string &content)
{
	std::string cntt = content;
	TmSvrData cmd{ id, TmSvrData::Mode::STRING, cntt.data(), cntt.size(), TmSvrData::SrcType::Shallow };
	TmPacket pack{ cmd };
	return send_packet_all(pack);
}

TmCommRC TmSvrCommunication::send_stick_play()
{
	return send_content_str("Play", "Stick_PlayPause=1");
}

void TmSvrCommunication::tm_svr_thread_function()
{
	print_info("Ethernet slave communication: thread begin");
	_keep_thread_alive = true;
	while (_keep_thread_alive) {
		bool reconnect = false;
		if (!recv_init()) {
			print_info("Ethernet slave communication: is not connected");
		}
		while (_keep_thread_alive && is_connected() && !reconnect) {
			TmCommRC rc = tmsvr_function();
			{
				std::lock_guard<std::mutex> lck(_mtx);
				_updated = true;
			}
			_cv->notify_all();

			switch (rc) {
			case TmCommRC::ERR:
			case TmCommRC::NOTREADY:
			case TmCommRC::NOTCONNECT:
			case TmCommRC::TIMEOUT:
				print_info("Ethernet slave communication: rc=%d", int(rc));
				reconnect = true;
				break;
			default: break;
			}
		}
		close_socket();
		reconnect_function();
	}
	close_socket();
	_cv->notify_all();
	print_info("Ethernet slave communication: thread end");
}

void TmSvrCommunication::reconnect_function()
{
	if (!_keep_thread_alive) return;
	if (_reconnect_timeval_ms <= 0) {
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
	print_info("Ethernet slave communication: Reconnecting.. ");
	int cnt = 0;
	while (_keep_thread_alive && cnt < _reconnect_timeval_ms) {
		if (cnt % 500 == 0) {
			print_debug("%.1f sec...", 0.001 * (_reconnect_timeval_ms - cnt));
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		++cnt;
	}
	if (_keep_thread_alive && _reconnect_timeval_ms >= 0) {
		print_debug("0 sec\nEthernet slave communication: connect(%dms)...", (int)_reconnect_timeout_ms);
		connect_socket("ethernet slave re-connection",_reconnect_timeout_ms);
	}
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
			tmSvrErrData.set_CPError(pack.data.data(), pack.data.size());
            print_error("Ethernet slave communication: CPERR %s",tmSvrErrData.error_code_str().c_str());
		}
		else if (pack.type == TmPacket::Header::TMSVR) {
			
			tmSvrErrData.error_code(TmCPError::Code::Ok);

			TmSvrData::build_TmSvrData(data, pack.data.data(), pack.data.size(), TmSvrData::SrcType::Shallow);
			
			if (data.is_valid()) {
				switch (data.mode()) {
				case TmSvrData::Mode::RESPONSE:
					print_info("Ethernet slave communication: RESPONSE (%s): [%d]: %s", data.transaction_id().c_str(),
						(int)(data.error_code()), std::string(data.content(), data.content_len()).c_str());
					break;
				case TmSvrData::Mode::BINARY:
					state.mtx_deserialize(data.content(), data.content_len());
					break;
				case TmSvrData::Mode::READ_STRING:
					print_info("Ethernet slave communication: READ_STRING (%s): %s", data.transaction_id().c_str(),
						std::string(data.content(), data.content_len()).c_str());
					break;
				default:
					print_error("Ethernet slave communication: (%s): invalid mode (%d)", data.transaction_id().c_str(), (int)(data.mode()));
					break;
				}
			}
			else {
				print_error("Ethernet slave communication: invalid data");
			}
		}
		else {
			print_error("Ethernet slave communication: invalid header");
		}
	}
	return rc;
}
