#ifdef NO_INCLUDE_DIR
#include "tm_communication.h"
#include "tm_print.h"
#else
#include "tm_driver/tm_communication.h"
#include "tm_driver/tm_print.h"
#endif

#include <functional>

#ifdef _WIN32
// windows socket
#pragma comment (lib, "Ws2_32.lib")
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#endif

//
// TmSBuffer
//

class TmSBuffer
{
private:
	std::vector<char> _bytes;

public:
	TmSBuffer()
	{
		print_debug("TmSBuffer::TmSBuffer");
	}
	~TmSBuffer()
	{
		print_debug("TmSBuffer::~TmSBuffer");
	}

	int length() const
	{
		return _bytes.size();
	}
	char *data()
	{
		return _bytes.data();
	}
	int append(const char *bdata, int blen)
	{
		if (blen <= 0) return 0;

		size_t old_size = _bytes.size();
		size_t new_size = old_size + blen;
		_bytes.resize(new_size);
		for (size_t i = 0; i < size_t(blen); ++i) {
			_bytes[old_size + i] = bdata[i];
		}
		//print_debug("TmSBuffer::append %d bytes", blen);
		return blen;
	}
	void pop_front(int len = 1)
	{
		// commit extract
		if (len <= 0) return;

		if (size_t(len) < _bytes.size()) {
			std::vector<char> tmp{ _bytes.begin() + len, _bytes.end() };
			_bytes.clear();
			_bytes.insert(_bytes.end(), tmp.begin(), tmp.end());
		}
		else {
			len = int(_bytes.size());
			_bytes.clear();
		}
		//print_debug("TmSBuffer::pop_front %d bytes", len);
	}
	void clear()
	{
		_bytes.clear();
	}
};

//
// TmSvrCommRecv
//

class TmCommRecv
{
private:
	TmSBuffer _sbuf;
	char *_recv_buf = NULL;
	int _recv_buf_len = 0;
	int _sockfd = -1;
	fd_set _masterfs;
	fd_set _readfs;
	int _rn = 0;
	TmCommRC _rc = TmCommRC::OK;

public:
	explicit TmCommRecv(int recv_buf_len)
	{
		print_debug("TmCommRecv::TmCommRecv");

		if (recv_buf_len < 512) recv_buf_len = 512;

		_recv_buf = new char[recv_buf_len];
		_recv_buf_len = recv_buf_len;

		memset(_recv_buf, 0, _recv_buf_len);
	}
	~TmCommRecv()
	{
		print_debug("TmCommRecv::~TmCommRecv");
		delete _recv_buf;
	}

	bool setup(int sockfd);

	TmCommRC spin_once(int timeval_ms, int *n = NULL);

	void commit_spin_once() { _sbuf.pop_front(_rn); }

	TmSBuffer &buffer() { return _sbuf; }
};

bool TmCommRecv::setup(int sockfd)
{
	if (sockfd <= 0) return false;

	_sbuf.clear();
	_sockfd = sockfd;

	FD_ZERO(&_masterfs);
	// fake
	if (sockfd != 6188) {
		FD_SET(sockfd, &_masterfs);
	}
	_rc = TmCommRC::OK;
	return true;
}

size_t _recv_fake_svr_pack_data(char *buf)
{
	static long long cnt = 0;
	std::this_thread::sleep_for(std::chrono::milliseconds(100));

	float angle[6] = { 0.0f, 0.0f, 90.0f, 0.0f, 90.0f, 0.0f };
	float pose[6] = { 420.0f, -120.0f, 360.0f, 180.0f, 0.0f, 90.0f };
	FakeTmSvrPacket svr_pack;
	FakeTmSvrPacket::build_content(svr_pack.content, angle, pose);
	TmSvrData::build_TmSvrData(svr_pack.data, "0", TmSvrData::Mode::BINARY,
		svr_pack.content.data(), svr_pack.content.size(), TmSvrData::SrcType::Shallow);
	TmSvrData::build_bytes(svr_pack.packet.data, svr_pack.data);
	svr_pack.packet.setup_header(TmPacket::Header::TMSVR);
	std::vector<char> pack_byte;
	TmPacket::build_bytes(pack_byte, svr_pack.packet);
	size_t n = pack_byte.size();
	for (size_t i = 0; i < n; ++i) {
		buf[i] = pack_byte[i];
	}
	if (cnt % 10 == 1) {
		for (size_t j = 1; j < 7; ++j) {
			for (size_t i = 0; i < n; ++i) {
				buf[j * n + i] = pack_byte[i];
			}
		}
		n *= 7;
	}
	++cnt;
	return n;
}

TmCommRC TmCommRecv::spin_once(int timeval_ms, int *n)
{
	TmCommRC rc = TmCommRC::OK;
	int nb = 0;
	int rv = 0;
	timeval tv;

	// fake
	if (_sockfd == 6188) {
		nb = _recv_fake_svr_pack_data(_recv_buf);
		_sbuf.append(_recv_buf, nb);

		if (n) *n = nb;
		_rn = nb;
		_rc = rc;
		return rc;
	}

	if (timeval_ms < 8) timeval_ms = 8;

	tv.tv_sec = (timeval_ms / 1000);
	tv.tv_usec = (timeval_ms % 1000) * 1000;

	_readfs = _masterfs; // re-init

	rv = select(_sockfd + 1, &_readfs, NULL, NULL, &tv);

	if (n) *n = 0;
	
	if (rv < 0) {
		rc = TmCommRC::ERR;
	}
	else if (rv == 0) {
		rc = TmCommRC::TIMEOUT;
	}
	else if (FD_ISSET(_sockfd, &_readfs)) {
		nb = recv(_sockfd, _recv_buf, _recv_buf_len, 0);

		if (nb < 0) {
			// error
			rc = TmCommRC::ERR;
		}
		else if (nb == 0) {
			// sever is closed
			rc = TmCommRC::NOTCONNECT;
		}
		else {
			// recv n bytes
			_sbuf.append(_recv_buf, nb);

			if (n) *n = nb;
		}
	}
	else {
		rc = TmCommRC::NOTREADY;
	}
	_rn = nb;
	_rc = rc;
	return rc;
}

//
// TmCommunication
//

TmCommunication::TmCommunication(const char *ip, unsigned short port, int recv_buf_len)
	: _recv(nullptr)
	, _ip(NULL)
	, _port(port)
	, _recv_buf_len(recv_buf_len)
	, _sockfd(-1)
	, _optflag(1)
	, _recv_rc(TmCommRC::OK)
	, _recv_ready(false)
{
	print_info("TmCommunication::TmCommunication");

	_recv = new TmCommRecv(recv_buf_len);

	size_t len = strlen(ip);

	_ip = new char[len + 1];
	memcpy(_ip, ip, len);
	_ip[len] = '\0';

#ifdef _WIN32
	// Initialize Winsock
	WSADATA wsaData;
	int iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
	if (iResult != 0) {
		//
	}
#endif
}

TmCommunication::~TmCommunication()
{
	print_info("TmCommunication::~TmCommunication");

	delete _ip;
	delete _recv;

#ifdef _WIN32
	// cleanup
	WSACleanup();
#endif
}

int TmCommunication::connect_with_timeout(int sockfd, const char *ip, unsigned short port, int timeout_ms)
{
	int rv = 0;
	int flags = 0;
	int err = 0;
	int err_len = 0;
	sockaddr_in addr;
	timeval tv;
	fd_set wset;

	print_info("TM_COM: ip:=%s", ip);

	addr.sin_family = AF_INET;
	addr.sin_port = htons(port);
	inet_pton(AF_INET, ip, &(addr.sin_addr));

	tv.tv_sec = (timeout_ms / 1000);
	tv.tv_usec = (timeout_ms % 1000) * 1000;

	FD_ZERO(&wset);
	FD_SET(sockfd, &wset);

#ifndef _WIN32
	//Get Flag of Fcntl
	if ((flags = fcntl(sockfd, F_GETFL, 0)) < 0 ) {
		print_warn("TM_COM: The flag of fcntl is not ok");
		return -1;
	}
#endif

	rv = connect(sockfd, (sockaddr *)&addr, 16);
	print_info("TM_COM: rv:=%d", rv);

	if (rv < 0) {
		if (errno != EINPROGRESS) return -1;
	}
	if (rv == 0) {
		print_info("TM_COM: Connection is ok");
		return rv;
	}
	else {
		//Wait for Connect OK by checking Write buffer
		if ((rv = select(sockfd + 1, NULL, &wset, NULL, &tv)) < 0) {
			return rv;
		}
		if (rv == 0) {
			print_warn("TM_COM: Connection timeout");
			//errno = ETIMEDOUT;
			return -1;
		}
		if (FD_ISSET(sockfd, &wset)) {
#ifdef _WIN32
			if (getsockopt(sockfd, SOL_SOCKET, SO_ERROR, (char*)&err, &err_len) < 0) {
#else
			if (getsockopt(sockfd, SOL_SOCKET, SO_ERROR, &err, (socklen_t *)&err_len) < 0) {
#endif
				print_error("TM_COM: Get socketopt SO_ERROR FAIL");
				errno = err;
				return -1;
			}
		}
		else {
			print_error("TM_COM: Connection is not ready");
			return -1;
		}
		if (err != 0) {
			errno = err;
			print_error("TM_COM: Connection error");
			return -1;
		}
	}
	return rv;
}

bool TmCommunication::connect_socket(int timeout_ms)
{
	if (_sockfd > 0) return true;

	if (timeout_ms < 0) timeout_ms = 0;

#ifdef _WIN32
	addrinfo hints;
	ZeroMemory(&hints, sizeof(hints));
	hints.ai_family = AF_INET;
	hints.ai_socktype = SOCK_STREAM;
	hints.ai_protocol = IPPROTO_HOPOPTS;

	socketFile = socket(hints.ai_family, hints.ai_socktype, hints.ai_protocol);
#else
	socketFile = socket(AF_INET, SOCK_STREAM, 0);
#endif
    _sockfd = socketFile;
	if (_sockfd < 0) {
		print_error("TM_COM: Error socket");
		return false;
	}

	setsockopt(_sockfd, IPPROTO_TCP, TCP_NODELAY, (char*)&_optflag, sizeof(_optflag));
#ifndef _WIN32
	setsockopt(_sockfd, IPPROTO_TCP, TCP_QUICKACK, (char*)&_optflag, sizeof(_optflag));
#endif
	setsockopt(_sockfd, SOL_SOCKET, SO_REUSEADDR, (char*)&_optflag, sizeof(_optflag));
	struct timeval timeout;      
    timeout.tv_sec = timeout_ms/1000;
    timeout.tv_usec = 0;

    if (setsockopt (_sockfd, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout,sizeof(timeout)) < 0){
        print_error("setsockopt failed\n");
	}
        

    if (setsockopt (_sockfd, SOL_SOCKET, SO_SNDTIMEO, (char *)&timeout,sizeof(timeout)) < 0){
		print_error("setsockopt failed\n");
	}
        

	if (connect_with_timeout(_sockfd, _ip, _port, timeout_ms) == 0) {
		print_info("TM_COM: O_NONBLOCK connection is ok");
	}
	else {
		print_info("TM_COM: O_NONBLOCK connection is fail");
		_sockfd = -1;
	}
	if (_sockfd > 0) {
		print_info("TM_COM: TM robot is connected. sockfd:=%d", _sockfd);
		//_is_connected = true;
		return true;
	}
	else {
		return false;
	}
}

void TmCommunication::close_socket()
{
	// reset
	_recv_rc = TmCommRC::OK;
	_recv_ready = false;

#ifdef _WIN32
	closesocket((SOCKET)socketFile);
#else
	close(socketFile);
#endif
	_sockfd = -1;
}

TmCommRC TmCommunication::send_bytes(const char *bytes, int len, int *n)
{
	TmCommRC rc = TmCommRC::OK;

	if (n) *n = 0;
	
	if (len <= 0) return TmCommRC::OK;
	if (_sockfd < 0) return TmCommRC::NOTREADY;

	int nb = send(_sockfd, bytes, len, 0);

	if (nb < 0) {
		rc = TmCommRC::ERR;
	}
	else if (nb < len) {
		rc = TmCommRC::NOTSENDALL;

		if (n) *n = nb;
	}
	return rc;
}

TmCommRC TmCommunication::send_bytes_all(const char *bytes, int len, int *n)
{
	TmCommRC rc = TmCommRC::OK;

	if (n) *n = 0;

	if (len <= 0) return TmCommRC::OK;
	if (_sockfd < 0) return TmCommRC::NOTREADY;

	int ntotal = 0;
	int nb = 0;
	int nleft = len;

	while (ntotal < len) {
		nb = send(_sockfd, bytes, nleft, 0);
		if (nb < 0) {
			rc = TmCommRC::ERR;
			break;
		}
		ntotal += nb;
		nleft -= nb;
	}
	if (n) *n = ntotal;
	return rc;
}

TmCommRC TmCommunication::send_packet(TmPacket &packet, int *n)
{
	std::vector<char> bytes;
	TmPacket::build_bytes(bytes, packet);
	print_info(TmPacket::string_from_bytes(bytes).c_str());
	return send_bytes(bytes.data(), bytes.size(), n);
}
TmCommRC TmCommunication::send_packet_all(TmPacket &packet, int *n)
{
	std::vector<char> bytes;
	TmPacket::build_bytes(bytes, packet);
	print_info(TmPacket::string_from_bytes(bytes).c_str());
	return send_bytes_all(bytes.data(), bytes.size(), n);
}
TmCommRC TmCommunication::send_packet_(TmPacket &packet, int *n)
{
	std::vector<char> bytes;
	TmPacket::build_bytes(bytes, packet);
	print_info(TmPacket::string_from_bytes(bytes).c_str());
	if (bytes.size() > 0x1000)
		return send_bytes_all(bytes.data(), bytes.size(), n);
	else
		return send_bytes(bytes.data(), bytes.size(), n);
}

bool TmCommunication::recv_init()
{
	_recv_ready = _recv->setup(_sockfd);
	return _recv_ready;
}

TmCommRC TmCommunication::recv_spin_once(int timeval_ms, int *n)
{
	TmCommRC rc = TmCommRC::OK;

	if (n) *n = 0;

	//if (_sockfd <= 0) return TmCommRC::NOTCONNECT;

	// first init.
	/*if (!_recv_ready) {
		if (_recv->setup(_sockfd))
			_recv_ready = true;
		else
			return TmCommRC::NOTREADY;
	}*/

	// spin once
	int nb = 0;
	rc = _recv->spin_once(timeval_ms, &nb);
	
	if (n) *n = nb;

	// error handling
	if (rc != TmCommRC::OK) {
		_recv_rc = rc;
		return rc;
	}

	// find packet
	int loop_cnt = 0;
	int pack_cnt = 0;
	int blen = 0;
	char *bdata = NULL;
	size_t size = 0;
	size_t len = 0;
	bool ncs = false;
	bool ok = false;

	while (loop_cnt < 10 || pack_cnt < 10) {

		blen = _recv->buffer().length();
		if (blen < 9) {
			break;
		}
		bdata = _recv->buffer().data();

		//print_debug("TmCommunication::recv_spin_once: %d, %d", bdata, loop_cnt);

		++size;
		_packet_list.resize(size);

		len = TmPacket::build_packet_from_bytes(_packet_list.back(), bdata, blen);

		ncs = packet().is_checksum_failed();
		ok = packet().is_valid();

		if (ok || ncs) {
			_recv->buffer().pop_front(len);
		}
		if (ok) {
			++pack_cnt;
		}
		else {
			if (size > 1) {
				_packet_list.resize(size - 1);
			}
			//if (pack_cnt != 0) break;
			break;
		}
		++loop_cnt;
	}
	if (pack_cnt == 0) {
		rc = TmCommRC::NOVALIDPACK;
	}
	_recv_rc = rc;
	return rc;
}
