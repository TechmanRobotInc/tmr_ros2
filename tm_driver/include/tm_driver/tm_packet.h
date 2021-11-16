#pragma once

#include <vector>
#include <string>
#include <type_traits>
#include <sstream>
#include <iomanip>
#include "tm_print.h"
//#include <unordered_map>


// header_type + bytes data  => packet bytes
// header_type + string data => packet bytes

// packet bytes => header_type + bytes data
// packet bytes => header_type + string data

class TmCPError;
class TmSctData;
class TmStaData;
class TmSvrData;

class TmPacket
{
public:
	static const char P_HEAD;
	static const char P_END1;
	static const char P_END2;
	static const char P_SEPR;
	static const char P_CSUM;

	static const std::string HDR_CPERR;
	static const std::string HDR_TMSCT;
	static const std::string HDR_TMSTA;
	static const std::string HDR_TMSVR;
	static const std::string PACKAGE_INCOMPLETE;

	enum class Header
	{
		EMPTY,
		CPERR,
		TMSCT,
		TMSTA,
		TMSVR,
		PACKAGE_INCOMPLETE,
		OTHER
	};

public:
	Header type{ Header::EMPTY };
	std::string header;
	std::vector<char> data;

private:
	mutable size_t _size = 0;
	mutable char _cs = 0;
	mutable bool _is_cs_failed = false;
	mutable bool _is_valid = false;

public:
	TmPacket() {}

	explicit TmPacket(Header type)
	{
		setup_header(type);
	}
	explicit TmPacket(Header type, const std::vector<char> &data)
		: data(data)
	{
		setup_header(type);
	}
	explicit TmPacket(Header type, const std::string &data)
		: data{ std::begin(data), std::end(data) }
	{
		setup_header(type);
	}
	explicit TmPacket(const std::string &hdr, const std::vector<char> &data)
		: data(data)
	{
		setup_header(hdr);
	}
	explicit TmPacket(const std::string &hdr, const std::string &data)
		: data{ std::begin(data), std::end(data) }
	{
		setup_header(hdr);
	}
	explicit TmPacket(const TmSvrData &data)
	{
		build_packet(*this, data);
	}
	explicit TmPacket(const TmSctData &data)
	{
		build_packet(*this, data);
	}
	explicit TmPacket(const TmStaData &data)
	{
		build_packet(*this, data);
	}

	void setup_header(Header type);

	void setup_header(const std::string &hdr);

	size_t bytes_size() { return _size; }

	char checksum() { return _cs; }

	bool is_checksum_failed() { return _is_cs_failed; }

	bool is_valid() { return _is_valid; }

	void reset()
	{
		setup_header(Header::EMPTY);
		data.clear();
		_size = 0;
		_cs = 0;
		_is_cs_failed = false;
		_is_valid = false;
	}
    void set_as_not_finish_data(){
        setup_header(Header::PACKAGE_INCOMPLETE);
		data.clear();
		_size = 0;
		_cs = 0;
		_is_cs_failed = false;
		_is_valid = false;
	}
public:
	static std::vector<char> bytes_from_string(const std::string &s)
	{
		return std::vector<char>{ std::begin(s), std::end(s) };
	}
	static std::string string_from_bytes(const std::vector<char> &b)
	{
		return std::string{ std::begin(b), std::end(b) };
	}

	static std::string string_from_hex_uint8(unsigned char num)
	{
		std::stringstream ss;
		ss << std::setfill('0') << std::setw(2) << std::hex << int(num);
		return ss.str();
	}
	static unsigned char hex_uint8_from_string(const std::string &s)
	{
		if (s.size() != 2) return 0;
		int val;
		std::stringstream ss;
		ss << std::hex << s;
		ss >> val;
		return (unsigned char)(val);
	}

	static char checksum_xor(const char *data, size_t size);
	static char checksum_xor(const std::vector<char> &data)
	{
		return checksum_xor(data.data(), data.size());
	}
	static char checksum_xor(const std::string &data)
	{
		return checksum_xor(data.data(), data.size());
	}

	static void build_bytes(std::vector<char> &bytes, const TmPacket &packet);
	static void build_bytes(std::string &bytes, const TmPacket &packet);
	static std::vector<char> bytes_vec_from_packet(const TmPacket &packet)
	{
		std::vector<char> bytes;
		build_bytes(bytes, packet);
		return bytes;
	}
	static std::string bytes_str_from_packet(const TmPacket &packet)
	{
		std::string bytes;
		build_bytes(bytes, packet);
		return bytes;
	}
	
	static size_t build_packet_from_bytes(TmPacket &packet, const char *bytes, size_t size);
	static void build_packet(TmPacket &packet, const char *bytes, size_t size)
	{
		build_packet_from_bytes(packet, bytes, size);
	}
	static void build_packet(TmPacket &packet, const std::vector<char> &bytes)
	{
		build_packet_from_bytes(packet, bytes.data(), bytes.size());
	}
	static void build_packet(TmPacket &packet, const std::string &bytes)
	{
		build_packet_from_bytes(packet, bytes.data(), bytes.size());
	}
	static TmPacket packet_from_bytes(const char *bytes, size_t size)
	{
		TmPacket packet;
		build_packet(packet, bytes, size);
		return packet;
	}
	static TmPacket packet_from_bytes(const std::vector<char> &bytes)
	{
		TmPacket packet;
		build_packet(packet, bytes);
		return packet;
	}
	static TmPacket packet_from_bytes(const std::string &bytes)
	{
		TmPacket packet;
		build_packet(packet, bytes);
		return packet;
	}

	static size_t find_packet_bytes_begin_index(const char *bytes, size_t max_len, size_t *pack_size = NULL);
	static const char *find_packet_bytes(const char *bytes, size_t max_len, size_t *pack_size = NULL)
	{
		const char *p = NULL;
		int ind_b = find_packet_bytes_begin_index(bytes, max_len, pack_size);
		if (ind_b < 0) {
			return p;
		}
		p = bytes;
		return p + ind_b;
	}

	static void build_packet(TmPacket &packet, const TmSvrData &data);
	static void build_packet(TmPacket &packet, const TmSctData &data);
	static void build_packet(TmPacket &packet, const TmStaData &data);
};

//
// TMSVR
//

class TmSvrData
{
public:
	enum SrcType { Shallow, Deep };

	enum class Mode : char {
		RESPONSE = 0,
		BINARY,
		STRING,
		JSON,
		READ_BINARY = 11,
		READ_STRING,
		READ_JSON,
		UNKNOW
	};
	enum class ErrorCode {
		Ok,
		NotSupport,
		WritePermission,
		InvalidData,
		NotExist,
		ReadOnly,
		ModeError,
		ValueError,

		Other
	};
private:
	std::string _transaction_id;
	Mode _mode = Mode::RESPONSE;
	std::string _content_str;
	const char *_content = NULL;
	size_t _len = 0;
	size_t _size = 0;

	ErrorCode _err_code = ErrorCode::Ok;

	bool _is_valid = false;
	bool _is_copy = false;

public:
	TmSvrData() {}
	explicit TmSvrData(const TmSvrData &other, SrcType type)
	{
		build_TmSvrData(*this, other, type);
	}
	explicit TmSvrData(const std::string &id, Mode mode, const char *content, size_t len, SrcType type)
	{
		build_TmSvrData(*this, id, mode, content, len, type);
	}
	explicit TmSvrData(const char *data, size_t size, SrcType type)
	{
		build_TmSvrData(*this, data, size, type);
	}
	~TmSvrData()
	{
		clear_content(*this);
	}

	std::string transaction_id() { return _transaction_id; }
	Mode mode() { return _mode; }
	std::string content_str() { return _content_str; }
	const char *content() { return _content; }
	size_t content_len() { return _len; }
	size_t data_size() { return _size; }
	ErrorCode error_code() { return _err_code; }
	bool is_valid() { return _is_valid; }

private:
	ErrorCode _error_code(const char *buf);

public:
	static void clear_content(TmSvrData &data);

	static void build_TmSvrData(TmSvrData &data, const TmSvrData &other, SrcType type);
	static void build_TmSvrData(TmSvrData &data, const std::string &id, Mode mode, const char *content, size_t len, SrcType type);
	static void build_TmSvrData(TmSvrData &data, const char *bytes, size_t size, SrcType type);
	
	static void build_bytes(std::vector<char> &bytes, const TmSvrData &data);

	//static void build_content_map(const char *content, size_t size);
};

class FakeTmSvrPacket
{
public:
	std::vector<char> content;
	TmSvrData data;
	TmPacket packet;

	static void build_content(std::vector<char> &content, float *angle, float *pose);
};

//
// TMSCT
//

class TmSctData
{
public:
	enum SrcType { Shallow, Deep };

private:
	std::string _script_id;
	std::string _script_str;
	const char *_script;
	size_t _len = 0;
	size_t _size = 0;

	bool _is_ok = false;
	bool _sctDataHasError = false;

	bool _is_valid = false;
	bool _is_copy = false;

public:
	TmSctData() {}
	explicit TmSctData(const TmSctData &other, SrcType type)
	{
		build_TmSctData(*this, other, type);
	}
	explicit TmSctData(const std::string &id, const char *script, size_t len, SrcType type)
	{
		build_TmSctData(*this, id, script, len, type);
	}
	explicit TmSctData(const char *data, size_t size, SrcType type)
	{
		build_TmSctData(*this, data, size, type);
	}
	~TmSctData()
	{
		clear_script(*this);
	}

	std::string script_id() { return _script_id; }
	std::string script_str() { return _script_str; }
	const char *script() { return _script; }
	size_t script_len() { return _len; }
	size_t data_size() { return _size; }
	bool is_ok() { return _is_ok; }
	bool sct_has_error() { return _sctDataHasError; }
	bool is_valid() { return _is_valid; }
	void set_sct_data_has_error(bool err_status);

public:
	static void clear_script(TmSctData &data);

	static void build_TmSctData(TmSctData &data, const TmSctData &other, SrcType type);
	static void build_TmSctData(TmSctData &data, const std::string &id, const char *script, size_t len, SrcType type);
	static void build_TmSctData(TmSctData &data, const char *bytes, size_t size, SrcType type);

	static void build_bytes(std::vector<char> &bytes, const TmSctData &data);
};

//
// TMSTA
//

class TmStaData
{
public:
	enum SrcType { Shallow, Deep };

private:
	unsigned char _subcmd;
	std::string _subcmd_str;
	std::string _subdata_str;
	const char *_subdata;
	size_t _len = 0;
	size_t _size = 0;

	bool _is_valid = false;
	bool _is_copy = false;

public:
	TmStaData() {}
	explicit TmStaData(const TmStaData &other, SrcType type)
	{
		build_TmStaData(*this, other, type);
	}
	explicit TmStaData(const std::string &sub_cmd, const char *sub_data, size_t len, SrcType type)
	{
		build_TmStaData(*this, sub_cmd, sub_data, len, type);
	}
	explicit TmStaData(const unsigned char sub_cmd, const char *sub_data, size_t len, SrcType type)
	{
		build_TmStaData(*this, sub_cmd, sub_data, len, type);
	}
	explicit TmStaData(const char *data, size_t size, SrcType type)
	{
		build_TmStaData(*this, data, size, type);
	}
	~TmStaData()
	{
		clear_subdata(*this);
	}

	unsigned char subcmd() { return _subcmd; }
	std::string subcmd_str() { return _subcmd_str; }
	const char *subdata() { return _subdata; }
	size_t subdata_len() { return _len; }
	size_t data_size() { return _size; }
	bool is_valid() { return _is_valid; }

public:
	static void clear_subdata(TmStaData &data);

	static void build_TmStaData(TmStaData &data, const TmStaData &other, SrcType type);
	static void build_TmStaData(TmStaData &data, const std::string &sub_cmd, const char *sub_data, size_t len, SrcType type);
	static void build_TmStaData(TmStaData &data, unsigned char sub_cmd, const char *sub_data, size_t len, SrcType type);
	static void build_TmStaData(TmStaData &data, const char *bytes, size_t size, SrcType type);

	static void build_bytes(std::vector<char> &bytes, const TmStaData &data);
};

//
// CPERR
//

class TmCPError
{
public:
	enum class Code : unsigned char
	{
		Ok,
		PacketErr,
		ChecksumErr,
		HeaderErr,
		DataErr,
		NoExtSctMode = 0xf1,
		Other = 0xff
	};
private:
	Code _err_code;
	std::string _err_code_str;

public:
	TmCPError() {}
	explicit TmCPError(Code ec)
	{
		set_CPError(ec);
	}
	explicit TmCPError(const char *bytes, size_t size)
	{
		set_CPError(bytes, size);
	}

	Code error_code() { return _err_code; }
	Code error_code(Code ec)
	{
		_err_code = ec;
		return _err_code;
	}
	std::string error_code_str() { return _err_code_str; }

	void set_CPError(Code ec);
	void set_CPError(const char *bytes, size_t size);
};
