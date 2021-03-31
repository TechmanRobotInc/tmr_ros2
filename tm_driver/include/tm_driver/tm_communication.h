#pragma once
#include "tm_packet.h"
//#include "tm_robot_state.h"

#include <chrono>
#include <thread>
#include <condition_variable>


enum class TmCommRC {
	ERR = -1,
	OK = 0,
	TIMEOUT,
	NOTREADY,
	NOTCONNECT,
	NOTSENDALL,
	NOVALIDPACK,
};

class TmCommRecv;

class TmCommunication
{
private:
	TmCommRecv *_recv;
	char *_ip;
	unsigned short _port;
	int _recv_buf_len;
	int _sockfd;
	int socketFile;
	int _optflag;
	TmCommRC _recv_rc;
	bool _recv_ready;
	//bool _is_connected;

private:
	//TmPacket _packet;
	std::vector<TmPacket> _packet_list;

public:
	explicit TmCommunication(const char *ip, unsigned short port, int recv_buf_len);
	virtual ~TmCommunication();

	int socket_description() { return _sockfd; }
	int socket_description(int sockfd) { _sockfd = sockfd; return _sockfd; }

	bool is_connected() { return (_sockfd > 0); }

	bool connect_socket(int timeout_ms = 0);

	void close_socket();

	TmCommRC send_bytes(const char *bytes, int len, int *n = NULL);

	TmCommRC send_bytes_all(const char *bytes, int len, int *n = NULL);
	
	TmCommRC send_packet(TmPacket &packet, int *n = nullptr);

	TmCommRC send_packet_all(TmPacket &packet, int *n = nullptr);

	TmCommRC send_packet_(TmPacket &packet, int *n = nullptr);

	bool recv_init();

	TmCommRC recv_spin_once(int timeval_ms, int *n = NULL);

	TmCommRC recv_rc() { return _recv_rc; }

	std::vector<TmPacket> &packet_list() { return _packet_list; }

	TmPacket &packet() { return _packet_list.back(); }

private:
	int connect_with_timeout(int sockfd, const char *ip, unsigned short port, int timeout_ms);

};
