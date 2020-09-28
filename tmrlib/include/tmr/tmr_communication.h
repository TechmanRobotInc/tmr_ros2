#pragma once
#include "tmr_tmpacket.h"
//#include "tmr_robot_state.h"

#include <chrono>
#include <thread>
#include <condition_variable>


namespace tmr
{

enum class CommRC {
	ERR = -1,
	OK = 0,
	TIMEOUT,
	NOTREADY,
	NOTCONNECT,
	NOTSENDALL,
	NOVALIDPACK,
};

class CommRecv;

class Communication
{
private:
	CommRecv *_recv;
	char *_ip;
	unsigned short _port;
	int _recv_buf_len;
	int _sockfd;
	int _optflag;
	CommRC _recv_rc;
	bool _recv_ready;
	//bool _is_connected;

private:
	//TmPacket _packet;
	std::vector<TmPacket> _packet_list;

public:
	explicit Communication(const char *ip, unsigned short port, int recv_buf_len);
	virtual ~Communication();

	int socket_description() { return _sockfd; }
	int socket_description(int sockfd) { _sockfd = sockfd; return _sockfd; }

	bool is_connected() { return (_sockfd > 0); }

	bool Connect(int timeout_ms = 0);

	void Close();

	CommRC send_bytes(const char *bytes, int len, int *n = NULL);

	CommRC send_bytes_all(const char *bytes, int len, int *n = NULL);
	
	CommRC send_packet(TmPacket &packet, int *n = nullptr);
	CommRC send_packet_all(TmPacket &packet, int *n = nullptr);
	CommRC send_packet_(TmPacket &packet, int *n = nullptr);

	CommRC send_packet_silent(TmPacket &packet, int *n = nullptr);
	CommRC send_packet_silent_all(TmPacket &packet, int *n = nullptr);
	CommRC send_packet_silent_(TmPacket &packet, int *n = nullptr);

	bool recv_init();

	CommRC recv_spin_once(int timeval_ms, int *n = NULL);

	CommRC recv_rc() { return _recv_rc; }

	std::vector<TmPacket> &packet_list() { return _packet_list; }

	TmPacket &packet() { return _packet_list.back(); }

private:
	int connect_with_timeout(int sockfd, const char *ip, unsigned short port, int timeout_ms);

};

}