#ifdef NO_INCLUDE_DIR
#include "tmr_tmsvr_communication.h"
#include "tmr_print.h"
#else
#include "tmr/tmr_tmsvr_communication.h"
#include "tmr/tmr_print.h"
#endif

#include <functional>

namespace tmr
{

//
// TmSvrCommunication
//

TmSvrCommunication::TmSvrCommunication(const std::string &ip,
	int recv_buf_len, std::condition_variable *cv)
	: Communication(ip.c_str(), 5891, recv_buf_len)
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
		tmr_INFO_STREAM("TM_SVR: start (fake)");
		if (_has_thread) {
			// start thread
			_recv_thread = std::thread(std::bind(&TmSvrCommunication::thread_function, this));
		}
		return true;
	}

	halt();
	tmr_INFO_STREAM("TM_SVR: start");

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
		tmr_INFO_STREAM("TM_SVR: halt (fake)");
		if (_has_thread) {
			_keep_thread_alive = false;
			if (_recv_thread.joinable()) {
				_recv_thread.join();
			}
		}
		return;
	}
	tmr_INFO_STREAM("TM_SVR: halt");
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

CommRC TmSvrCommunication::send_content(const std::string &id, TmSvrData::Mode mode, const std::string &content)
{
	std::string cntt = content;
	TmSvrData cmd{ id, mode, cntt.data(), cntt.size(), TmSvrData::SrcType::Shallow };
	TmPacket pack{ cmd };
	return send_packet_all(pack);
}
CommRC TmSvrCommunication::send_content_str(const std::string &id, const std::string &content)
{
	std::string cntt = content;
	TmSvrData cmd{ id, TmSvrData::Mode::STRING, cntt.data(), cntt.size(), TmSvrData::SrcType::Shallow };
	TmPacket pack{ cmd };
	return send_packet_all(pack);
}
CommRC TmSvrCommunication::send_stick_play()
{
	return send_content_str("Play", "Stick_PlayPause=1");
}

void TmSvrCommunication::thread_function()
{
	tmr_INFO_STREAM("TM_SVR: thread begin");
	_keep_thread_alive = true;
	while (_keep_thread_alive) {
		bool reconnect = false;
		if (!recv_init()) {
			tmr_INFO_STREAM("TM_SVR: is not connected");
		}
		while (_keep_thread_alive && is_connected() && !reconnect) {
			CommRC rc = tmsvr_function();
			{
				std::lock_guard<std::mutex> lck(_mtx);
				_updated = true;
			}
			_cv->notify_all();

			switch (rc) {
			case CommRC::ERR:
			case CommRC::NOTREADY:
			case CommRC::NOTCONNECT:
				tmr_INFO_STREAM("TM_SVR: rc=" << int(rc));
				reconnect = true;
				break;
			default: break;
			}
		}
		Close();
		reconnect_function();
	}
	Close();
	_cv->notify_all();
	tmr_INFO_STREAM("TM_SVR: thread end");
}
void TmSvrCommunication::reconnect_function()
{
	if (!_keep_thread_alive) return;
	if (_reconnect_timeval_ms <= 0) {
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
	tmr_INFO_STREAM("TM_SVR: reconnect in ");
	int cnt = 0;
	while (_keep_thread_alive && cnt < _reconnect_timeval_ms) {
		if (cnt % 500 == 0) {
			tmr_INFO_STREAM(0.001 * (_reconnect_timeval_ms - cnt) << " sec...");
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		++cnt;
	}
	if (_keep_thread_alive && _reconnect_timeval_ms >= 0) {
		tmr_INFO_STREAM("0 sec\nTM_SVR: connect(" << _reconnect_timeout_ms << "ms)...");
		Connect(_reconnect_timeout_ms);
	}
}
CommRC TmSvrCommunication::tmsvr_function()
{
	CommRC rc;
	int n;
	rc = recv_spin_once(1000, &n);
	if (rc != CommRC::OK) {
		return rc;
	}
	std::vector<TmPacket> &pack_vec = packet_list();

	for (auto &pack : pack_vec) {
		if (pack.type == TmPacket::Header::CPERR) {
			tmr_INFO_STREAM("TM_SVR: CPERR");
			err_data.set_CPError(pack.data.data(), pack.data.size());
			tmr_ERROR_STREAM(err_data.error_code_str());
		}
		else if (pack.type == TmPacket::Header::TMSVR) {
			
			err_data.error_code(TmCPError::Code::Ok);

			TmSvrData::build_TmSvrData(data, pack.data.data(), pack.data.size(), TmSvrData::SrcType::Shallow);
			
			if (data.is_valid()) {
				switch (data.mode()) {
				case TmSvrData::Mode::RESPONSE:
					tmr_INFO_STREAM("TM_SVR: RESPONSE (" << data.transaction_id() << "): [" <<
						(int)(data.error_code()) << "]: " << std::string(data.content(), data.content_len()));
					break;
				case TmSvrData::Mode::BINARY:
					state.mtx_deserialize(data.content(), data.content_len());
					break;
				case TmSvrData::Mode::READ_STRING:
					tmr_INFO_STREAM("TM_SVR: READ_STRING (" << data.transaction_id() << "): " <<
						std::string(data.content(), data.content_len()));
					break;
				default:
					tmr_WARN_STREAM("TM_SVR: (" << data.transaction_id() << "): invalid mode (" << (int)(data.mode()) << ")");
					break;
				}
			}
			else {
				tmr_WARN_STREAM("TM_SVR: invalid data");
			}
		}
		else {
			tmr_WARN_STREAM("TM_SVR: invalid header");
		}
	}
	return rc;
}

}