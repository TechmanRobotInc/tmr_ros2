#ifdef NO_INCLUDE_DIR
#include "tm_robot_state.h"
#include "tm_print.h"
#else
#include "tm_driver/tm_robot_state.h"
#include "tm_driver/tm_print.h"
#endif

#include <memory>
#include <cstring>
#include <sstream>
#include <iostream>

#include <map>

class TmDataTable
{
public:
	struct Item {
		void *dst;
		bool required;
		bool checked;
		enum { NOT_REQUIRE = 0 ,REQUIRED = 1 };
		Item() : dst(nullptr), required(true), checked(false) {};
		Item(void *d) : dst(d), required(true), checked(false) {};
		Item(void *d, bool r) : dst(d), required(r), checked(false) {};
	};
private:
	std::map<std::string, Item> _item_map;

public:
	TmDataTable(TmRobotState *rs)
	{
		print_debug("Create DataTable");

		_item_map.clear();
		//_item_map[""] = { Item:, &rs- };
		_item_map["Robot_Link"         ] = { &rs->_is_linked_ };
		_item_map["Robot_Error"        ] = { &rs->_has_error_ };
		_item_map["Project_Run"        ] = { &rs->_is_proj_running_ };
		_item_map["Project_Pause"      ] = { &rs->_is_proj_paused_ };
		_item_map["Safeguard_A"        ] = { &rs->_is_safeguard_A_triggered_};
		_item_map["ESTOP"              ] = { &rs->_is_ESTOP_pressed_ };
		_item_map["Camera_Light"       ] = { &rs->_camera_light_ };
		_item_map["Error_Code"         ] = { &rs->_error_code_ };
		_item_map["Joint_Angle"        ] = { &rs->_joint_angle_ };
		_item_map["Coord_Robot_Flange" ] = { &rs->_flange_pose_ };
		_item_map["Coord_Robot_Tool"   ] = { &rs->_tool_pose_ };
		_item_map["TCP_Force"          ] = { &rs->_tcp_force_vec_ };
		_item_map["TCP_Force3D"        ] = { &rs->_tcp_force_ };
		_item_map["TCP_Speed"          ] = { &rs->_tcp_speed_vec_ };
		_item_map["TCP_Speed3D"        ] = { &rs->_tcp_speed_ };
		_item_map["Joint_Speed"        ] = { &rs->_joint_speed_ };
		_item_map["Joint_Torque"       ] = { &rs->_joint_torque_ };
		_item_map["Joint_Torque_Average"] = { &rs->_joint_torque_average_ ,Item::NOT_REQUIRE};
		_item_map["Joint_Torque_Min"   ] = { &rs->_joint_torque_min_ ,Item::NOT_REQUIRE};
		_item_map["Joint_Torque_Max"   ] = { &rs->_joint_torque_max_ ,Item::NOT_REQUIRE};
		_item_map["Project_Speed"      ] = { &rs->_proj_speed_ };
		_item_map["MA_Mode"            ] = { &rs->_ma_mode_ };
		_item_map["Robot_Light"        ] = { &rs->_robot_light_ };
		_item_map["Ctrl_DO0"           ] = { &rs->_ctrller_DO_[ 0] };
		_item_map["Ctrl_DO1"           ] = { &rs->_ctrller_DO_[ 1] };
		_item_map["Ctrl_DO2"           ] = { &rs->_ctrller_DO_[ 2] };
		_item_map["Ctrl_DO3"           ] = { &rs->_ctrller_DO_[ 3] };
		_item_map["Ctrl_DO4"           ] = { &rs->_ctrller_DO_[ 4] };
		_item_map["Ctrl_DO5"           ] = { &rs->_ctrller_DO_[ 5] };
		_item_map["Ctrl_DO6"           ] = { &rs->_ctrller_DO_[ 6] };
		_item_map["Ctrl_DO7"           ] = { &rs->_ctrller_DO_[ 7] };
		_item_map["Ctrl_DO8"           ] = { &rs->_ctrller_DO_[ 8],Item::NOT_REQUIRE };
		_item_map["Ctrl_DO9"           ] = { &rs->_ctrller_DO_[ 9],Item::NOT_REQUIRE };
		_item_map["Ctrl_DO10"          ] = { &rs->_ctrller_DO_[10],Item::NOT_REQUIRE };
		_item_map["Ctrl_DO11"          ] = { &rs->_ctrller_DO_[11],Item::NOT_REQUIRE };
		_item_map["Ctrl_DO12"          ] = { &rs->_ctrller_DO_[12],Item::NOT_REQUIRE };
		_item_map["Ctrl_DO13"          ] = { &rs->_ctrller_DO_[13],Item::NOT_REQUIRE };
		_item_map["Ctrl_DO14"          ] = { &rs->_ctrller_DO_[14],Item::NOT_REQUIRE };
		_item_map["Ctrl_DO15"          ] = { &rs->_ctrller_DO_[15],Item::NOT_REQUIRE };
		_item_map["Ctrl_DI0"           ] = { &rs->_ctrller_DI_[ 0] };
		_item_map["Ctrl_DI1"           ] = { &rs->_ctrller_DI_[ 1] };
		_item_map["Ctrl_DI2"           ] = { &rs->_ctrller_DI_[ 2] };
		_item_map["Ctrl_DI3"           ] = { &rs->_ctrller_DI_[ 3] };
		_item_map["Ctrl_DI4"           ] = { &rs->_ctrller_DI_[ 4] };
		_item_map["Ctrl_DI5"           ] = { &rs->_ctrller_DI_[ 5] };
		_item_map["Ctrl_DI6"           ] = { &rs->_ctrller_DI_[ 6] };
		_item_map["Ctrl_DI7"           ] = { &rs->_ctrller_DI_[ 7] };
		_item_map["Ctrl_DI8"           ] = { &rs->_ctrller_DI_[ 8],Item::NOT_REQUIRE };
		_item_map["Ctrl_DI9"           ] = { &rs->_ctrller_DI_[ 9],Item::NOT_REQUIRE };
		_item_map["Ctrl_DI10"          ] = { &rs->_ctrller_DI_[10],Item::NOT_REQUIRE };
		_item_map["Ctrl_DI11"          ] = { &rs->_ctrller_DI_[11],Item::NOT_REQUIRE };
		_item_map["Ctrl_DI12"          ] = { &rs->_ctrller_DI_[12],Item::NOT_REQUIRE };
		_item_map["Ctrl_DI13"          ] = { &rs->_ctrller_DI_[13],Item::NOT_REQUIRE };
		_item_map["Ctrl_DI14"          ] = { &rs->_ctrller_DI_[14],Item::NOT_REQUIRE };
		_item_map["Ctrl_DI15"          ] = { &rs->_ctrller_DI_[15],Item::NOT_REQUIRE };
		_item_map["Ctrl_AO0"           ] = { &rs->_ctrller_AO_[ 0] };
		_item_map["Ctrl_AO1"           ] = { &rs->_ctrller_AO_[ 1] ,Item::NOT_REQUIRE};
		_item_map["Ctrl_AI0"           ] = { &rs->_ctrller_AI_[ 0] };
		_item_map["Ctrl_AI1"           ] = { &rs->_ctrller_AI_[ 1],Item::NOT_REQUIRE };
		_item_map["End_DO0"            ] = { &rs->_ee_DO_[0] };
		_item_map["End_DO1"            ] = { &rs->_ee_DO_[1] };
		_item_map["End_DO2"            ] = { &rs->_ee_DO_[2] };
		_item_map["End_DO3"            ] = { &rs->_ee_DO_[3] };
		_item_map["End_DI0"            ] = { &rs->_ee_DI_[0] };
		_item_map["End_DI1"            ] = { &rs->_ee_DI_[1] };
		_item_map["End_DI2"            ] = { &rs->_ee_DI_[2] };
		_item_map["End_DI3"            ] = { &rs->_ee_DI_[3],Item::NOT_REQUIRE };
		_item_map["End_AO0"            ] = { &rs->_ee_AO_[0],Item::NOT_REQUIRE };
		_item_map["End_AO1"            ] = { &rs->_ee_AO_[1],Item::NOT_REQUIRE };
		_item_map["End_AI0"            ] = { &rs->_ee_AI_[0] };
		_item_map["End_AI1"            ] = { &rs->_ee_AI_[1],Item::NOT_REQUIRE };
	}
	std::map<std::string, Item>  & get() { return _item_map; }
	std::map<std::string, Item>::iterator find(const std::string &name) { return _item_map.find(name); }
	std::map<std::string, Item>::iterator end() { return _item_map.end(); }
};

TmRobotState::TmRobotState()
{
	print_debug("TmRobotState::TmRobotState");

	_data_table = new TmDataTable(this);

	_joint_angle.assign(DOF, 0.0);
	_flange_pose.assign(6, 0.0);
	_tool_pose.assign(6, 0.0);
	_tcp_force_vec.assign(3, 0.0);
	_tcp_speed_vec.assign(6, 0.0);
	_joint_speed.assign(DOF, 0.0);
	_joint_torque.assign(DOF, 0.0);
	_joint_torque_average.assign(DOF, 0.0);
	_joint_torque_min.assign(DOF, 0.0);
	_joint_torque_max.assign(DOF, 0.0);
	_tcp_frame.assign(6, 0.0);
	_tcp_cog.assign(6, 0.0);

	_ctrller_DO.assign(16, false);
	_ctrller_DI.assign(16, false);
	_ctrller_AO.assign(2, 0.0);
	_ctrller_AI.assign(2, 0.0);
	_ee_DO.assign(4, false);
	_ee_DI.assign(4, false);
	_ee_AO.assign(2, 0.0);
	_ee_AI.assign(2, 0.0);

	_f_deserialize_item[0] = std::bind(&TmRobotState::_deserialize_skip,
		std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
	_f_deserialize_item[1] = std::bind(&TmRobotState::_deserialize_copy_wo_check,
		std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);

	_f_deserialize = std::bind(&TmRobotState::_deserialize_first_time, this,
		std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
}
TmRobotState::~TmRobotState()
{
	print_debug("TmRobotState::~TmRobotState");
	delete _data_table;
}

std::vector<double> TmRobotState::mtx_flange_pose()
{
	std::vector<double> rv(_flange_pose.size());
	mtx_lock();
	rv = _flange_pose;
	mtx_unlock();
	return rv;
}
std::vector<double> TmRobotState::mtx_joint_angle()
{
	std::vector<double> rv(_joint_angle.size());
	mtx_lock();
	rv = _joint_angle;
	mtx_unlock();
	return rv;
}
std::vector<double> TmRobotState::mtx_tool_pose()
{
	std::vector<double> rv(_tool_pose.size());
	mtx_lock();
	rv = _tool_pose;
	mtx_unlock();
	return rv;
}
std::string TmRobotState::mtx_error_content()
{
	std::string rv;
	mtx_lock();
	rv = _error_content;
	mtx_unlock();
	return rv;
}
std::vector<unsigned char> TmRobotState::mtx_ctrller_DO()
{
	std::vector<unsigned char> rv;
	mtx_lock();
	rv = _ctrller_DO;
	mtx_unlock();
	return rv;
}
std::vector<unsigned char> TmRobotState::mtx_ctrller_DI()
{
	std::vector<unsigned char> rv;
	mtx_lock();
	rv = _ctrller_DI;
	mtx_unlock();
	return rv;
}
std::vector<float> TmRobotState::mtx_ctrller_AO()
{
	std::vector<float_t> rv;
	mtx_lock();
	rv = _ctrller_AO;
	mtx_unlock();
	return rv;
}
std::vector<float> TmRobotState::mtx_ctrller_AI()
{
	std::vector<float> rv;
	mtx_lock();
	rv = _ctrller_AO;
	mtx_unlock();
	return rv;
}
std::vector<unsigned char> TmRobotState::mtx_ee_DO()
{
	std::vector<unsigned char> rv;
	mtx_lock();
	rv = _ee_DO;
	mtx_unlock();
	return rv;
}
std::vector<unsigned char> TmRobotState::mtx_ee_DI()
{
	std::vector<unsigned char> rv;
	mtx_lock();
	rv = _ee_DI;
	mtx_unlock();
	return rv;
}
std::vector<float> TmRobotState::mtx_ee_AI()
{
	std::vector<float> rv;
	mtx_lock();
	rv = _ee_AI;
	mtx_unlock();
	return rv;
}

size_t TmRobotState::_deserialize_get_name(std::string &name, const char *data, size_t offset)
{
	size_t boffset = offset;
	unsigned short uslen; // 2 bytes

	// item name length
	memcpy(&uslen, data + boffset, 2);
	boffset += 2;
	// item name
	name = std::string{data + boffset, uslen};
	boffset += uslen;
	// skip item
	memcpy(&uslen, data + boffset, 2);
	boffset += 2 + uslen;
	return boffset;
}
size_t TmRobotState::_deserialize_skip(void *dst, const char *data, size_t offset)
{
	size_t boffset = offset;
	unsigned short uslen; // 2 bytes

	// skip item name
	memcpy(&uslen, data + boffset, 2);
	boffset += 2 + uslen;
	// skip item
	memcpy(&uslen, data + boffset, 2);
	boffset += 2 + uslen;

	if (dst) {}
	return boffset;
}
size_t TmRobotState::_deserialize_copy_wo_check(void *dst, const char *data, size_t offset)
{
	size_t boffset = offset;
	size_t bsize = 2;
	unsigned short uslen; // 2 bytes

	// skip item name
	memcpy(&uslen, data + boffset, bsize);
	boffset += bsize + uslen;
	// item data length
	memcpy(&uslen, data + boffset, bsize);
	boffset += bsize;
	// item data
	bsize = uslen;
	memcpy(dst, data + boffset, bsize);
	boffset += bsize;
	return boffset;
}

size_t TmRobotState::_deserialize_first_time(const char *data, size_t size, bool lock)
{
	size_t boffset = 0;
	size_t count = 0;
	size_t check_count = 0;
	size_t skip_count = 0;
	unsigned short uslen = 0; // 2 bytes
	std::string item_name;

	print_info("TM Flow DataTable Checked Item: ");
	_item_updates.clear();
	//_f_deserialize_item.clear();

	while (boffset < size && count < 100) {
		//boffset = _deserialize_get_name(item_name, data, boffset);
		// item name length
		memcpy(&uslen, data + boffset, 2);
		boffset += 2;
		// item name
		item_name = std::string{data + boffset, uslen};
		boffset += uslen;

		ItemUpdate update{ nullptr, ItemUpdate::SKIP };
		//std::function<size_t (void *, const char *, size_t)> func;
		auto iter = _data_table->find(item_name);
		if (iter != _data_table->end()) {
			update.dst = iter->second.dst;
			update.func = ItemUpdate::UPDATE;
			//func = std::bind(&RobotState::_deserialize_copy_wo_check, iter->second.dst,
			//	std::placeholders::_2, std::placeholders::_3);
			iter->second.checked = true;
			std::string msg = "- " + item_name + " - checked";
			print_info(msg.c_str());
			++check_count;
		}
		else {
			//func = std::bind(&RobotState::_deserialize_skip, nullptr,
			//	std::placeholders::_2, std::placeholders::_3);
			std::string msg = "- " + item_name + " - skipped";
			print_info(msg.c_str());
			++skip_count;
		}
		_item_updates.push_back({ update.dst, update.func });
		//_f_deserialize_item.push_back(func);

		// item data length
		memcpy(&uslen, data + boffset, 2);
		boffset += 2;
		if (update.func == ItemUpdate::SKIP) {
			// skip item
			boffset += uslen;
		}
		else {
			// item data
			memcpy(update.dst, data + boffset, uslen);
			boffset += uslen;
		}
		++count;
	}
	
	std::string msg = "Total " + std::to_string(_item_updates.size()) + " item," +
	std::to_string(check_count) + " checked, " + std::to_string(skip_count) + " skipped";
	print_info(msg.c_str());
	isDataTableCorrect = true;

	_deserialize_update(lock);

	for (auto iter : _data_table->get()) {
		if (iter.second.required && !iter.second.checked) {
			std::string msg = "Required item" + iter.first + " is NOT checked";
			isDataTableCorrect = false;
			print_error(msg.c_str());
		}
	}

	if(isDataTableCorrect){
	  print_info("data table is correct!");
	} else{
          print_error("data table is not correct");
	}

	_f_deserialize = std::bind(&TmRobotState::_deserialize, this,
		std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);

	return boffset;
}
size_t TmRobotState::_deserialize(const char *data, size_t size, bool use_mtx)
{
	size_t boffset = 0;

	for (auto &update : _item_updates) {
		boffset = _f_deserialize_item[update.func](update.dst, data, boffset);
	}


	_deserialize_update(use_mtx);

	if (boffset > size) {
	}
	return boffset;
}
void TmRobotState::_deserialize_update(bool lock) {
	// ---------------
	// update together
	// ---------------

	//const char c0 = 0;

	_is_linked = _is_linked_;
	_has_error = _has_error_;
	_is_proj_running = _is_proj_running_;
	_is_proj_paused = _is_proj_paused_;

	_is_safeguard_A_triggered = _is_safeguard_A_triggered_;
	_is_ESTOP_pressed = _is_ESTOP_pressed_;
	_camera_light = _camera_light_;
	_error_code = _error_code_;

	_proj_speed = _proj_speed_;
	_ma_mode = _ma_mode_;
	//_stick_play_pause = _stick_play_pause_;

	_robot_light = _robot_light_;

	if (lock) mtx_lock();
	{
		_joint_angle = rads(_joint_angle_, DOF);

		si_pose(_flange_pose, _flange_pose_, 6);

		si_pose(_tool_pose, _tool_pose_, 6);

		for (int i = 0; i < 3; ++i) { _tcp_force_vec[i] = double(_tcp_force_vec_[i]); }

		_tcp_force = double(_tcp_force_);

		si_pose(_tcp_speed_vec, _tcp_speed_vec_, 6);

		_joint_speed = rads(_joint_speed_, DOF);

		_joint_torque = meters(_joint_torque_, DOF);
		
		_joint_torque_average = meters(_joint_torque_average_, DOF);
		
        _joint_torque_min = meters(_joint_torque_min_, DOF);
		
		_joint_torque_max = meters(_joint_torque_max_, DOF);
		// IO

		for (int i = 0; i < 8; ++i) { _ctrller_DO[i] = _ctrller_DO_[i]; }
		for (int i = 0; i < 8; ++i) { _ctrller_DI[i] = _ctrller_DI_[i]; }
		for (int i = 0; i < 1; ++i) { _ctrller_AO[i] = _ctrller_AO_[i]; }
		for (int i = 0; i < 2; ++i) { _ctrller_AI[i] = _ctrller_AI_[i]; }
		for (int i = 0; i < 4; ++i) { _ee_DO[i] = _ee_DO_[i]; }
		for (int i = 0; i < 4; ++i) { _ee_DI[i] = _ee_DI_[i]; }
		//for (int i = 0; i < 1; ++i) { _ee_AO[i] = _ee_AO_[i]; }
		for (int i = 0; i < 1; ++i) { _ee_AI[i] = _ee_AI_[i]; }
	}
	if (lock) mtx_unlock();
}

void TmRobotState::print()
{
	mtx_lock();
	std::cout << "Robot_Link=" << _is_linked << "\n";
	std::cout << "Robot_Error=" << _has_error << "\n";
	std::cout << "Project_Run=" << _is_proj_running << "\n";
	std::cout << "Project_Pause=" << _is_proj_running << "\n";
	std::cout << "Safetyguard_A=" << _is_safeguard_A_triggered << "\n";
	std::cout << "ESTOP=" << _is_ESTOP_pressed << "\n";

	std::cout << "Joint_Angle={";
	for (auto &val : _joint_angle) { std::cout << val << ", "; }
	std::cout << "}\n";

	std::cout << "Coord_Robot_Tool0={";
	for (auto &val : _flange_pose) { std::cout << val << ", "; }
	std::cout << "}\n";

	std::cout << "Coord_Robot_Tool={";
	for (auto &val : _tool_pose) { std::cout << val << ", "; }
	std::cout << "}\n";

	std::cout << "Error_Code=" << _error_code << "\n";
	std::cout << "Error_Content=" << _error_content << "\n";
	mtx_unlock();
}
