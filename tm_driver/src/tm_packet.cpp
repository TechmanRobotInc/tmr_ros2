#ifdef NO_INCLUDE_DIR
#include "tm_packet.h"
//#include "tm_print.h"
#else
#include "tm_driver/tm_packet.h"
//#include "tm_driver/tm_print.h"
#endif

#include <cstring>

//
// TmPacket
//

const char TmPacket::P_HEAD = 0x24; // $
const char TmPacket::P_END1 = 0x0D; // \r
const char TmPacket::P_END2 = 0x0A; // \n
const char TmPacket::P_SEPR = 0x2C; // ,
const char TmPacket::P_CSUM = 0x2A; // *

const std::string TmPacket::HDR_CPERR = "CPERR";
const std::string TmPacket::HDR_TMSCT = "TMSCT";
const std::string TmPacket::HDR_TMSTA = "TMSTA";
const std::string TmPacket::HDR_TMSVR = "TMSVR";
const std::string TmPacket::PACKAGE_INCOMPLETE = "PACKAGE_INCOMPLETE";

void TmPacket::setup_header(Header type)
{
	this->type = type;
	switch (type) {
	case Header::EMPTY: header.clear(); break;
	case Header::CPERR: header = HDR_CPERR; break;
	case Header::TMSCT: header = HDR_TMSCT; break;
	case Header::TMSTA: header = HDR_TMSTA; break;
	case Header::TMSVR: header = HDR_TMSVR; break;
	case Header::PACKAGE_INCOMPLETE : header = PACKAGE_INCOMPLETE; break;
	case Header::OTHER: break;
	}
}
void TmPacket::setup_header(const std::string &hdr)
{
	header = hdr;
	if (header.size() == 0) {
		type = Header::EMPTY;
		return;
	}
	if (header.compare(HDR_CPERR) == 0) {
		type = Header::CPERR;
	}
	else if (header.compare(HDR_TMSCT) == 0) {
		type = Header::TMSCT;
	}
	else if (header.compare(HDR_TMSTA) == 0) {
		type = Header::TMSTA;
	}
	else if (header.compare(HDR_TMSVR) == 0) {
		type = Header::TMSVR;
	}
	else {
		type = Header::OTHER;
	}
}

char TmPacket::checksum_xor(const char *data, size_t size)
{
	char cs = 0x00;
	for (size_t i = 0; i < size; ++i) { cs ^= char(data[i]); }
	return cs;
}

void TmPacket::build_bytes(std::vector<char> &bytes, const TmPacket &packet)
{
	char cs = 0x00;
	if (bytes.size() != 0) {
		bytes.clear();
	}
	// Header
	bytes.push_back(P_HEAD);
	bytes.insert(bytes.end(), std::begin(packet.header), std::end(packet.header));
	bytes.push_back(P_SEPR);
	// Length
	size_t length = packet.data.size();
	std::string slen = std::to_string(length);
	bytes.insert(bytes.end(), std::begin(slen), std::end(slen));
	bytes.push_back(P_SEPR);
	// Data
	bytes.insert(bytes.end(), std::begin(packet.data), std::end(packet.data));
	bytes.push_back(P_SEPR);
	// Checksum
	cs = checksum_xor(bytes.data() + 1, bytes.size() - 1);
	//cs = checksum_xor(bytes, 1, -1);
	bytes.push_back(P_CSUM);
	std::string shex = string_from_hex_uint8(cs);
	bytes.insert(bytes.end(), std::begin(shex), std::end(shex));
	// End
	bytes.push_back(P_END1);
	bytes.push_back(P_END2);
	//return cs;
	packet._size = bytes.size();
	packet._cs = cs;
	packet._is_cs_failed = false;
	packet._is_valid = true;
}
void TmPacket::build_bytes(std::string &bytes, const TmPacket &packet)
{
	char cs = 0x00;
	if (bytes.size() != 0) {
		bytes.clear();
	}
	// Header
	bytes.push_back(P_HEAD);
	bytes.append(packet.header);
	bytes.push_back(P_SEPR);
	// Length
	size_t length = packet.data.size();
	bytes.append(std::to_string(length));
	bytes.push_back(P_SEPR);
	// Data
	bytes.insert(bytes.end(), std::begin(packet.data), std::end(packet.data));
	bytes.push_back(P_SEPR);
	// Checksum
	cs = checksum_xor(bytes.data() + 1, bytes.size() - 1);
	//cs = checksum_xor(bytes, 1, -1);
	bytes.push_back(P_CSUM);
	bytes.append(string_from_hex_uint8(cs));
	// End
	bytes.push_back(P_END1);
	bytes.push_back(P_END2);
	//return cs;
	packet._size = bytes.size();
	packet._cs = cs;
	packet._is_cs_failed = false;
	packet._is_valid = true;
}

size_t TmPacket::build_packet_from_bytes(TmPacket &packet, const char *bytes, size_t size)
{
	size_t ind_e = 1, ind_b = 1;
	size_t length = 0;
	char cs = 0;
	bool is_valid = true;

	if (size < 9) {
		is_valid = false;
	}
	if (bytes[0] != P_HEAD) {
		is_valid = false;
	}
	if (!is_valid) {
        print_warn("package header is not $");
		packet.reset();
		packet._size = ind_e;
		return ind_e;
	}//goto end;

	//size_t ind_b = 1, ind_e = 1;

	// find end of header (first P_SEPR)
	while (ind_e < size && bytes[ind_e] != P_SEPR) {
		++ind_e;
	}
	// didn't find header
	if (ind_e + 8 > size) {
		//is_valid = false; 
		print_warn("didn't find header");
		packet.reset();
		packet._size = ind_e;
		return ind_e;
		//goto end;
	}
	// setup header
	if (ind_e > 1) {
		std::string hdr{ bytes + ind_b, ind_e - ind_b };
		packet.setup_header(hdr);
	}
	else {
		packet.setup_header(Header::EMPTY);
	}
	++ind_e;
	ind_b = ind_e;

	// check header

	// find end of length
	while (ind_e < size && bytes[ind_e] != P_SEPR) {
		++ind_e;
	}
	// didn't find length
	if (ind_e + 7 > size) {
		print_warn("didn't find package length");
		packet.set_as_not_finish_data();
		packet._size = ind_e;
		return ind_e;
		//is_valid = false; goto end;
	}
	// get length
	if (ind_e > ind_b) {
		std::string len{ bytes + ind_b, ind_e - ind_b };
		length = std::stoi(len);
	}
	++ind_e;
	ind_b = ind_e;

	// check length
	if (ind_e + length + 6 > size || bytes[ind_e + length] != P_SEPR) {
		//is_valid = false; goto end;
		print_warn("package length not valid");
		packet.set_as_not_finish_data();
		packet._size = ind_e;
		return ind_e;
	}

	// find and save data, and checksum
	packet.data.clear();
	packet.data.resize(length);

	for (size_t i = 0; i < length; ++i) {
		packet.data[i] = bytes[ind_b + i];
		//cs ^= bytes[ind_b + i];
	}
	cs = checksum_xor(bytes + 1, ind_e + length);
	ind_e += length + 1;
	ind_b = ind_e;
	
	// check checksum (P_CSUM)
	if (bytes[ind_e] != P_CSUM) {
		//is_valid = false; goto end;
		print_warn("check sum not valid");
		packet.set_as_not_finish_data();
		packet._size = ind_e;
		return ind_e;
	}
	++ind_e;
	ind_b = ind_e;

	// check checksum 
	{
		int val;
		std::stringstream ss;
		ss << std::hex << bytes[ind_e] << bytes[ind_e + 1];
		ss >> val;
		packet._cs = (char)(val);
		if (cs != packet._cs) {
			packet._is_cs_failed = true;
			is_valid = false;
		}
	}
	ind_e += 2;
	ind_b = ind_e;

	// check end
	if (bytes[ind_e] != P_END1 || bytes[ind_e + 1] != P_END2) {
		++ind_e;
		print_warn("package end not valid");
		packet.set_as_not_finish_data();
		packet._size = ind_e;
		return ind_e;
		//is_valid = false; goto end;
	}
	ind_e += 2;
	ind_b = ind_e;

	packet._size = ind_e;
	packet._is_valid = true;
//end:
	if (!is_valid) {
		print_warn("package not valid");
		//packet.reset();
		packet.set_as_not_finish_data();
		packet._size = ind_e;
	}
	return ind_e;
}

size_t TmPacket::find_packet_bytes_begin_index(const char *bytes, size_t max_len, size_t *pack_size)
{
	size_t ind_b = 0;
	size_t ind_e = 0;
	size_t count_sepr = 0;

	if (pack_size) {
		*pack_size = 0;
	}
	if (max_len < 9) return -1;

	// find head (P_HEAD)
	while (ind_e < max_len && bytes[ind_e] != P_HEAD) {
		++ind_e;
	}
	if (ind_e + 9 > max_len) return -1;

	ind_b = ind_e;

	// find checksum (P_CSUM)
	while (1) {
		while (ind_e < max_len && bytes[ind_e] != P_CSUM) {
			if (bytes[ind_e] == P_SEPR) {
				++count_sepr;
			}
			++ind_e;
		}
		if (ind_e + 5 > max_len) return -1;

		// check \r\n
		if (bytes[ind_e + 3] == P_END1 && bytes[ind_e + 4] == P_END2)
			break;

		++ind_e;
	}
	if (count_sepr < 3) return -1;

	ind_e += 5;

	if (pack_size) {
		*pack_size = ind_e - ind_b;
	}
	return ind_b;
}

//
// TmSvrData
//

TmSvrData::ErrorCode TmSvrData::_error_code(const char *buf)
{
	char cc[3] = { buf[0], buf[1], '\0' };
	int ic = std::atoi(cc);
	if (ic < (int)(ErrorCode::Other)) {
		return ErrorCode(ic);
	}
	else {
		return ErrorCode::Other;
	}
}

void TmSvrData::clear_content(TmSvrData &data)
{
	/*if (data._is_copy && data._content) {
		delete data._content;
		data._content = nullptr;
	}*/
	if (data._is_copy) {
		data._content_str.clear();
	}
	data._content = nullptr;
	data._size -= data._len;
	data._len = 0;
}

void TmSvrData::build_TmSvrData(TmSvrData &data, const TmSvrData &other, SrcType type)
{
	data._is_copy = (type != SrcType::Shallow);
	data._transaction_id = other._transaction_id;
	data._mode = other._mode;
	if (data._is_copy) {
		data._content_str = std::string{ other._content, other._len };
		data._content = data._content_str.data();
	}
	else {
		data._content_str.clear();
		data._content = other._content;
	}
	data._err_code = other._err_code;
	data._len = other._len;
	data._size = other._size;
	data._is_valid = other._is_valid;
}
void TmSvrData::build_TmSvrData(TmSvrData &data, const std::string &id, Mode mode, const char *content, size_t len, SrcType type)
{
	data._is_copy = (type != SrcType::Shallow);
	data._is_valid = false;
	data._transaction_id = id;
	data._mode = mode;
	if (data._is_copy) {
		/*char *pch = new char[len];
		memcpy(pch, content, len);
		data._content = pch;*/
		data._content_str = std::string{ content, len };
		data._content = data._content_str.data();
	}
	else {
		data._content_str.clear();
		data._content = content;
	}
	data._len = len;

	// mode 0/1/2/3/ AND 11/12/13
	if (data._mode < Mode::READ_BINARY)
		data._size = len + id.size() + 3;
	else
		data._size = len + id.size() + 4;

	// mode 0~255 ?

	data._err_code = ErrorCode::Ok;
	data._is_valid = true;
}

void TmSvrData::build_TmSvrData(TmSvrData &data, const char *bytes, size_t size, SrcType type)
{
	data._is_copy = (type != SrcType::Shallow);
	data._is_valid = false;

	if (!bytes) {
		return;
	}

	//size_t ind_b = 0;
	size_t ind_e = 0;

	// find end of transaction ID (P_SEPR)
	while (ind_e < size && bytes[ind_e] != TmPacket::P_SEPR) {
		++ind_e;
	}
	if (ind_e + 2 > size) {
		return;
	}

	data._transaction_id = std::string{ bytes, ind_e };

	++ind_e;
	//ind_b = ind_e;

	// mode 0/1/2/3/ (deprecated)

	/*char amode[] = { bytes[ind_e], '\0' };
	char cmode = std::atoi(amode);
	if (cmode > char(Mode::UNKNOW) || bytes[ind_e] < char(Mode::RESPONSE)) {
		return;
	}
	data._mode = Mode(cmode);
	++ind_e;
	if (bytes[ind_e] != TmPacket::P_SEPR) {
		return;
	}
	++ind_e;*/

	// mode 0/1/2/3/ AND 11/12/13

	char amode[] = { bytes[ind_e], '\0', '\0' };
	char cmode = 0;
	if (bytes[ind_e + 1] == TmPacket::P_SEPR) {
		++ind_e;
	}
	else if (bytes[ind_e + 2] == TmPacket::P_SEPR) {
		amode[1] = bytes[ind_e + 1];
		ind_e += 2;
	}
	else {
		return;
	}
	cmode = std::atoi(amode);
	if (cmode > (char)(Mode::UNKNOW)) {
		return;
	}
	++ind_e;
	data._mode = Mode(cmode);

	// mode 0~255 ?

	// find end of mode (P_SEPR)
	/*size_t ind_b = ind_e;
	while (ind_e < size && bytes[ind_e] != TmPacket::P_SEPR) {
		++ind_e;
	}
	std::string smode = std::string{ bytes + ind_b, ind_e - ind_b };
	char cmode = std::atoi(smode.c_str());
	if (cmode > (char)(Mode::UNKNOW)) {
		return;
	}
	++ind_e;
	data._mode = Mode(cmode);
	*/

	if (data._mode == Mode::RESPONSE) {
		if (ind_e + 3 > size) {
			return;
		}
		data._err_code = data._error_code(bytes + ind_e);

		ind_e += 2;

		if (bytes[ind_e] != TmPacket::P_SEPR) {
			return;
		}
		
		++ind_e;
	}
	else {
		data._err_code = ErrorCode::Ok;
	}

	data._len = size - ind_e;

	if (data._is_copy) {
		/*char *pch = new char[data._len];
		memcpy(pch, bytes + ind_e, data._len);
		data._content = pch;*/
		data._content_str = std::string{ bytes + ind_e, data._len };
		data._content = data._content_str.data();
	}
	else {
		data._content_str.clear();
		data._content = bytes + ind_e;
	}
	data._size = size;
	data._is_valid = true;
}

void TmSvrData::build_bytes(std::vector<char> &bytes, const TmSvrData &data)
{
	if (bytes.size() != 0) {
		bytes.clear();
	}
	bytes.insert(bytes.end(), std::begin(data._transaction_id), std::end(data._transaction_id));
	bytes.push_back(TmPacket::P_SEPR);
	std::string smode = std::to_string((int)(data._mode));
	bytes.insert(bytes.end(), std::begin(smode), std::end(smode));
	bytes.push_back(TmPacket::P_SEPR);
	size_t ind_b = bytes.size();
	size_t ind_e = ind_b;
	size_t new_size = ind_e + data._len;
	bytes.resize(new_size);
	for (; ind_e < new_size; ++ind_e) {
		bytes[ind_e] = data._content[ind_e - ind_b];
	}
}

void TmPacket::build_packet(TmPacket &packet, const TmSvrData &data)
{
	packet.setup_header(TmPacket::Header::TMSVR);
	TmSvrData::build_bytes(packet.data, data);
}


void FakeTmSvrPacket::build_content(std::vector<char> &content, float *angle, float *pose)
{
	if (content.size() != 0) {
		content.clear();
	}
	union {
		char ca[2];
		unsigned short us;
	} ulen;
	std::string item_name = "Robot_Link";
	ulen.us = (unsigned short)item_name.size();
	content.push_back(ulen.ca[0]);
	content.push_back(ulen.ca[1]);
	content.insert(content.end(), std::begin(item_name), std::end(item_name));
	ulen.us = (unsigned short)1;
	content.push_back(ulen.ca[0]);
	content.push_back(ulen.ca[1]);
	content.push_back(1);
	item_name = "Robot_Error";
	ulen.us = (unsigned short)item_name.size();
	content.push_back(ulen.ca[0]);
	content.push_back(ulen.ca[1]);
	content.insert(content.end(), std::begin(item_name), std::end(item_name));
	ulen.us = (unsigned short)1;
	content.push_back(ulen.ca[0]);
	content.push_back(ulen.ca[1]);
	content.push_back(0);
	item_name = "Project_Run";
	ulen.us = (unsigned short)item_name.size();
	content.push_back(ulen.ca[0]);
	content.push_back(ulen.ca[1]);
	content.insert(content.end(), std::begin(item_name), std::end(item_name));
	ulen.us = (unsigned short)1;
	content.push_back(ulen.ca[0]);
	content.push_back(ulen.ca[1]);
	content.push_back(0);
	item_name = "Project_Pause";
	ulen.us = (unsigned short)item_name.size();
	content.push_back(ulen.ca[0]);
	content.push_back(ulen.ca[1]);
	content.insert(content.end(), std::begin(item_name), std::end(item_name));
	ulen.us = (unsigned short)1;
	content.push_back(ulen.ca[0]);
	content.push_back(ulen.ca[1]);
	content.push_back(0);
	item_name = "Joint_Angle";
	ulen.us = (unsigned short)item_name.size();
	content.push_back(ulen.ca[0]);
	content.push_back(ulen.ca[1]);
	content.insert(content.end(), std::begin(item_name), std::end(item_name));
	ulen.us = (unsigned short)24;
	content.push_back(ulen.ca[0]);
	content.push_back(ulen.ca[1]);
	char fdata[24];
	memcpy(fdata, angle, 24);
	for (int i = 0; i < 24; ++i) {
		content.push_back(fdata[i]);
	}
	item_name = "Coord_Base_Tool";
	ulen.us = (unsigned short)item_name.size();
	content.push_back(ulen.ca[0]);
	content.push_back(ulen.ca[1]);
	content.insert(content.end(), std::begin(item_name), std::end(item_name));
	ulen.us = (unsigned short)24;
	content.push_back(ulen.ca[0]);
	content.push_back(ulen.ca[1]);
	memcpy(fdata, pose, 24);
	for (int i = 0; i < 24; ++i) {
		content.push_back(fdata[i]);
	}
	item_name = "Stick_PlayPause";
	ulen.us = (unsigned short)item_name.size();
	content.push_back(ulen.ca[0]);
	content.push_back(ulen.ca[1]);
	content.insert(content.end(), std::begin(item_name), std::end(item_name));
	ulen.us = (unsigned short)1;
	content.push_back(ulen.ca[0]);
	content.push_back(ulen.ca[1]);
	content.push_back(0);
	item_name = "Error_Code";
	ulen.us = (unsigned short)item_name.size();
	content.push_back(ulen.ca[0]);
	content.push_back(ulen.ca[1]);
	content.insert(content.end(), std::begin(item_name), std::end(item_name));
	ulen.us = (unsigned short)4;
	content.push_back(ulen.ca[0]);
	content.push_back(ulen.ca[1]);
	int ival = 0;
	memcpy(fdata, &ival, 4);
	for (int i = 0; i < 4; ++i) {
		content.push_back(fdata[i]);
	}
	item_name = "Error_Content";
	ulen.us = (unsigned short)item_name.size();
	content.push_back(ulen.ca[0]);
	content.push_back(ulen.ca[1]);
	content.insert(content.end(), std::begin(item_name), std::end(item_name));
	ulen.us = (unsigned short)0;
	content.push_back(ulen.ca[0]);
	content.push_back(ulen.ca[1]);
}

//
// TMSCT
//

void TmSctData::clear_script(TmSctData &data)
{
	/*if (data._is_copy && data._script) {
		delete data._script;
		data._script = nullptr;
	}*/
	if (data._is_copy) {
		data._script_str.clear();
	}
	data._script = nullptr;
	data._size -= data._len;
	data._len = 0;
}

void TmSctData::build_TmSctData(TmSctData &data, const TmSctData &other, SrcType type)
{
	data._is_copy = (type != SrcType::Shallow);
	data._script_id = other._script_id;
	if (data._is_copy) {
		data._script_str = std::string{ other._script, other._len };
		data._script = data._script_str.data();
	}
	else {
		data._script_str.clear();
		data._script = other._script;
	}
	data._len = other._len;
	data._size = other._size;
	data._is_valid = other._is_valid;
}
void TmSctData::build_TmSctData(TmSctData &data, const std::string &id, const char *script, size_t len, SrcType type)
{
	data._is_copy = (type != SrcType::Shallow);
	data._is_valid = false;
	data._script_id = id;
	if (data._is_copy) {
		data._script_str = std::string{ script, len };
		data._script = data._script_str.data();
	}
	else {
		data._script_str.clear();
		data._script = script;
	}
	data._len = len;
	data._size = len + id.size() + 1;
	data._is_valid = true;
}
void TmSctData::set_sct_data_has_error(bool err_status){
	this->_sctDataHasError = err_status;
}
void TmSctData::build_TmSctData(TmSctData &data, const char *bytes, size_t size, SrcType type)
{
	data._is_copy = (type != SrcType::Shallow);
	data._is_valid = false;
	data._sctDataHasError = false;
	data._is_ok = false;
	//size_t ind_b = 0;
	size_t ind_e = 0;
	// find end of script ID (P_SEPR)
	while (ind_e < size && bytes[ind_e] != TmPacket::P_SEPR) {
		++ind_e;
	}
	if (ind_e + 1 > size) {
		return;
	}
	data._script_id = std::string{ bytes, ind_e };
	
	++ind_e;

	data._len = size - ind_e;

	if (data._is_copy) {
		/*char *pch = new char[data._len];
		memcpy(pch, bytes + ind_e, data._len);
		data._script = pch;*/
		data._script_str = std::string{ bytes + ind_e, data._len };
		data._script = data._script_str.data();
	}
	else {
		data._script_str.clear();
		data._script = bytes + ind_e;
	}
	data._size = size;

	if (strncmp(data._script, "OK", 2) == 0) {
		data._is_ok = true;
	}
	else if (strncmp(data._script, "ERROR", 5) == 0) {
		data._sctDataHasError = true;
	}	
	data._is_valid = true;
}

void TmSctData::build_bytes(std::vector<char> &bytes, const TmSctData &data)
{
	if (bytes.size() != 0) {
		bytes.clear();
	}
	bytes.insert(bytes.end(), std::begin(data._script_id), std::end(data._script_id));
	bytes.push_back(TmPacket::P_SEPR);
	size_t ind_b = bytes.size();
	size_t ind_e = ind_b;
	size_t new_size = ind_e + data._len;
	bytes.resize(new_size);
	for (; ind_e < new_size; ++ind_e) {
		bytes[ind_e] = data._script[ind_e - ind_b];
	}
}

void TmPacket::build_packet(TmPacket &packet, const TmSctData &data)
{
	packet.setup_header(TmPacket::Header::TMSCT);
	TmSctData::build_bytes(packet.data, data);
}

//
// TMSTA
//

void TmStaData::clear_subdata(TmStaData &data)
{
	if (data._is_copy) {
		data._subcmd_str.clear();
	}
	data._subdata = nullptr;
	data._size -= data._len;
	data._len = 0;
}

void TmStaData::build_TmStaData(TmStaData &data, const TmStaData &other, SrcType type)
{
	data._is_copy = (type != SrcType::Shallow);
	data._subcmd = other._subcmd;
	data._subcmd_str = other._subcmd_str;
	if (data._is_copy) {
		data._subdata_str = std::string{ other._subdata, other._len };
		data._subdata = data._subdata_str.data();
	}
	else {
		data._subdata_str.clear();
		data._subdata = other._subdata;
	}
	data._len = other._len;
	data._size = other._size;
	data._is_valid = other._is_valid;
}
void TmStaData::build_TmStaData(TmStaData &data, const std::string &sub_cmd, const char *sub_data, size_t len, SrcType type)
{
	data._is_copy = (type != SrcType::Shallow);
	data._is_valid = false;
	data._subcmd = TmPacket::hex_uint8_from_string(sub_cmd);
	data._subcmd_str = sub_cmd;
	if (data._is_copy) {
		data._subdata_str = std::string{ sub_data, len };
		data._subdata = data._subdata_str.data();
	}
	else {
		data._subdata_str.clear();
		data._subdata = sub_data;
	}
	data._len = len;
	data._size = len + sub_cmd.size() + 1;
	data._is_valid = true;
}
void TmStaData::build_TmStaData(TmStaData &data, unsigned char sub_cmd, const char *sub_data, size_t len, SrcType type)
{
	data._is_copy = (type != SrcType::Shallow);
	data._is_valid = false;
	data._subcmd = sub_cmd;
	data._subcmd_str = TmPacket::string_from_hex_uint8(sub_cmd);	
	if (data._is_copy) {
		data._subdata_str = std::string{ sub_data, len };
		data._subdata = data._subdata_str.data();
	}
	else {
		data._subdata_str.clear();
		data._subdata = sub_data;
	}
	data._len = len;
	data._size = len + data._subcmd_str.size() + 1;
	data._is_valid = true;
}
void TmStaData::build_TmStaData(TmStaData &data, const char *bytes, size_t size, SrcType type)
{
	data._is_copy = (type != SrcType::Shallow);
	data._is_valid = false;

	size_t ind_e = 0;
	// find end of SubCmd (P_SEPR)
	while (ind_e < size && bytes[ind_e] != TmPacket::P_SEPR) {
		++ind_e;
	}
	if (ind_e + 1 > size) {
		return;
	}
	data._subcmd_str = std::string{ bytes, ind_e };
	data._subcmd = TmPacket::hex_uint8_from_string(data._subcmd_str);

	++ind_e;

	data._len = size - ind_e;

	if (data._is_copy) {
		/*char *pch = new char[data._len];
		memcpy(pch, bytes + ind_e, data._len);
		data._subdata = pch;*/
		data._subdata_str = std::string{ bytes + ind_e, data._len };
		data._subdata = data._subdata_str.data();
	}
	else {
		data._subdata_str.clear();
		data._subdata = bytes + ind_e;
	}
	data._size = size;

	data._is_valid = true;
}

void TmStaData::build_bytes(std::vector<char> &bytes, const TmStaData &data)
{
	if (bytes.size() != 0) {
		bytes.clear();
	}
	bytes.insert(bytes.end(), std::begin(data._subcmd_str), std::end(data._subcmd_str));
	bytes.push_back(TmPacket::P_SEPR);
	size_t ind_b = bytes.size();
	size_t ind_e = ind_b;
	size_t new_size = ind_e + data._len;
	bytes.resize(new_size);
	for (; ind_e < new_size; ++ind_e) {
		bytes[ind_e] = data._subdata[ind_e - ind_b];
	}
}

void TmPacket::build_packet(TmPacket &packet, const TmStaData &data)
{
	packet.setup_header(TmPacket::Header::TMSTA);
	TmStaData::build_bytes(packet.data, data);
}

//
// CPERR
//

void TmCPError::set_CPError(Code ec)
{
	_err_code = ec;
	_err_code_str = TmPacket::string_from_hex_uint8((unsigned char)(ec));
}
void TmCPError::set_CPError(const char *bytes, size_t size)
{
	if (size != 2) {
		_err_code = Code::Other;
		_err_code_str.clear();
		return;
	}
	_err_code_str = std::string{ bytes, size };

	_err_code = Code(TmPacket::hex_uint8_from_string(_err_code_str));
}
