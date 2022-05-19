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
		_item_map["Robot_Link"         ] = { &rs->tmRobotStateDataFromEthernet.is_linked };
		_item_map["Robot_Error"        ] = { &rs->tmRobotStateDataFromEthernet.has_error };
		_item_map["Project_Run"        ] = { &rs->tmRobotStateDataFromEthernet.is_proj_running };
		_item_map["Project_Pause"      ] = { &rs->tmRobotStateDataFromEthernet.is_proj_paused };
		_item_map["Safeguard_A"        ] = { &rs->tmRobotStateDataFromEthernet.is_safeguard_A_triggered};
		_item_map["ESTOP"              ] = { &rs->tmRobotStateDataFromEthernet.is_ESTOP_pressed };
		_item_map["Camera_Light"       ] = { &rs->tmRobotStateDataFromEthernet.camera_light };
		_item_map["Error_Code"         ] = { &rs->tmRobotStateDataFromEthernet.error_code };
		_item_map["Joint_Angle"        ] = { &rs->tmRobotStateDataFromEthernet.joint_angle };
		_item_map["Coord_Robot_Flange" ] = { &rs->tmRobotStateDataFromEthernet.flange_pose };
		_item_map["Coord_Robot_Tool"   ] = { &rs->tmRobotStateDataFromEthernet.tool_pose };
		_item_map["TCP_Force"          ] = { &rs->tmRobotStateDataFromEthernet.tcp_force_vec };
		_item_map["TCP_Force3D"        ] = { &rs->tmRobotStateDataFromEthernet.tcp_force };
		_item_map["TCP_Speed"          ] = { &rs->tmRobotStateDataFromEthernet.tcp_speed_vec };
		_item_map["TCP_Speed3D"        ] = { &rs->tmRobotStateDataFromEthernet.tcp_speed };
		_item_map["Joint_Speed"        ] = { &rs->tmRobotStateDataFromEthernet.joint_speed };
		_item_map["Joint_Torque"       ] = { &rs->tmRobotStateDataFromEthernet.joint_torque };
		_item_map["Joint_Torque_Average"] = { &rs->tmRobotStateDataFromEthernet.joint_torque_average ,Item::NOT_REQUIRE};
		_item_map["Joint_Torque_Min"   ] = { &rs->tmRobotStateDataFromEthernet.joint_torque_min ,Item::NOT_REQUIRE};
		_item_map["Joint_Torque_Max"   ] = { &rs->tmRobotStateDataFromEthernet.joint_torque_max ,Item::NOT_REQUIRE};
		_item_map["Project_Speed"      ] = { &rs->tmRobotStateDataFromEthernet.proj_speed };
		_item_map["MA_Mode"            ] = { &rs->tmRobotStateDataFromEthernet.ma_mode };
		_item_map["Robot_Light"        ] = { &rs->tmRobotStateDataFromEthernet.robot_light };
		_item_map["Ctrl_DO0"           ] = { &rs->tmRobotStateDataFromEthernet.ctrller_DO[ 0] };
		_item_map["Ctrl_DO1"           ] = { &rs->tmRobotStateDataFromEthernet.ctrller_DO[ 1] };
		_item_map["Ctrl_DO2"           ] = { &rs->tmRobotStateDataFromEthernet.ctrller_DO[ 2] };
		_item_map["Ctrl_DO3"           ] = { &rs->tmRobotStateDataFromEthernet.ctrller_DO[ 3] };
		_item_map["Ctrl_DO4"           ] = { &rs->tmRobotStateDataFromEthernet.ctrller_DO[ 4] };
		_item_map["Ctrl_DO5"           ] = { &rs->tmRobotStateDataFromEthernet.ctrller_DO[ 5] };
		_item_map["Ctrl_DO6"           ] = { &rs->tmRobotStateDataFromEthernet.ctrller_DO[ 6] };
		_item_map["Ctrl_DO7"           ] = { &rs->tmRobotStateDataFromEthernet.ctrller_DO[ 7] };
		_item_map["Ctrl_DO8"           ] = { &rs->tmRobotStateDataFromEthernet.ctrller_DO[ 8],Item::NOT_REQUIRE };
		_item_map["Ctrl_DO9"           ] = { &rs->tmRobotStateDataFromEthernet.ctrller_DO[ 9],Item::NOT_REQUIRE };
		_item_map["Ctrl_DO10"          ] = { &rs->tmRobotStateDataFromEthernet.ctrller_DO[10],Item::NOT_REQUIRE };
		_item_map["Ctrl_DO11"          ] = { &rs->tmRobotStateDataFromEthernet.ctrller_DO[11],Item::NOT_REQUIRE };
		_item_map["Ctrl_DO12"          ] = { &rs->tmRobotStateDataFromEthernet.ctrller_DO[12],Item::NOT_REQUIRE };
		_item_map["Ctrl_DO13"          ] = { &rs->tmRobotStateDataFromEthernet.ctrller_DO[13],Item::NOT_REQUIRE };
		_item_map["Ctrl_DO14"          ] = { &rs->tmRobotStateDataFromEthernet.ctrller_DO[14],Item::NOT_REQUIRE };
		_item_map["Ctrl_DO15"          ] = { &rs->tmRobotStateDataFromEthernet.ctrller_DO[15],Item::NOT_REQUIRE };
		_item_map["Ctrl_DI0"           ] = { &rs->tmRobotStateDataFromEthernet.ctrller_DI[ 0] };
		_item_map["Ctrl_DI1"           ] = { &rs->tmRobotStateDataFromEthernet.ctrller_DI[ 1] };
		_item_map["Ctrl_DI2"           ] = { &rs->tmRobotStateDataFromEthernet.ctrller_DI[ 2] };
		_item_map["Ctrl_DI3"           ] = { &rs->tmRobotStateDataFromEthernet.ctrller_DI[ 3] };
		_item_map["Ctrl_DI4"           ] = { &rs->tmRobotStateDataFromEthernet.ctrller_DI[ 4] };
		_item_map["Ctrl_DI5"           ] = { &rs->tmRobotStateDataFromEthernet.ctrller_DI[ 5] };
		_item_map["Ctrl_DI6"           ] = { &rs->tmRobotStateDataFromEthernet.ctrller_DI[ 6] };
		_item_map["Ctrl_DI7"           ] = { &rs->tmRobotStateDataFromEthernet.ctrller_DI[ 7] };
		_item_map["Ctrl_DI8"           ] = { &rs->tmRobotStateDataFromEthernet.ctrller_DI[ 8],Item::NOT_REQUIRE };
		_item_map["Ctrl_DI9"           ] = { &rs->tmRobotStateDataFromEthernet.ctrller_DI[ 9],Item::NOT_REQUIRE };
		_item_map["Ctrl_DI10"          ] = { &rs->tmRobotStateDataFromEthernet.ctrller_DI[10],Item::NOT_REQUIRE };
		_item_map["Ctrl_DI11"          ] = { &rs->tmRobotStateDataFromEthernet.ctrller_DI[11],Item::NOT_REQUIRE };
		_item_map["Ctrl_DI12"          ] = { &rs->tmRobotStateDataFromEthernet.ctrller_DI[12],Item::NOT_REQUIRE };
		_item_map["Ctrl_DI13"          ] = { &rs->tmRobotStateDataFromEthernet.ctrller_DI[13],Item::NOT_REQUIRE };
		_item_map["Ctrl_DI14"          ] = { &rs->tmRobotStateDataFromEthernet.ctrller_DI[14],Item::NOT_REQUIRE };
		_item_map["Ctrl_DI15"          ] = { &rs->tmRobotStateDataFromEthernet.ctrller_DI[15],Item::NOT_REQUIRE };
		_item_map["Ctrl_AO0"           ] = { &rs->tmRobotStateDataFromEthernet.ctrller_AO[ 0] };
		_item_map["Ctrl_AO1"           ] = { &rs->tmRobotStateDataFromEthernet.ctrller_AO[ 1] ,Item::NOT_REQUIRE};
		_item_map["Ctrl_AI0"           ] = { &rs->tmRobotStateDataFromEthernet.ctrller_AI[ 0] };
		_item_map["Ctrl_AI1"           ] = { &rs->tmRobotStateDataFromEthernet.ctrller_AI[ 1],Item::NOT_REQUIRE };
		_item_map["End_DO0"            ] = { &rs->tmRobotStateDataFromEthernet.ee_DO[0] };
		_item_map["End_DO1"            ] = { &rs->tmRobotStateDataFromEthernet.ee_DO[1] };
		_item_map["End_DO2"            ] = { &rs->tmRobotStateDataFromEthernet.ee_DO[2] };
		_item_map["End_DO3"            ] = { &rs->tmRobotStateDataFromEthernet.ee_DO[3] };
		_item_map["End_DI0"            ] = { &rs->tmRobotStateDataFromEthernet.ee_DI[0] };
		_item_map["End_DI1"            ] = { &rs->tmRobotStateDataFromEthernet.ee_DI[1] };
		_item_map["End_DI2"            ] = { &rs->tmRobotStateDataFromEthernet.ee_DI[2] };
		_item_map["End_DI3"            ] = { &rs->tmRobotStateDataFromEthernet.ee_DI[3],Item::NOT_REQUIRE };
		_item_map["End_AO0"            ] = { &rs->tmRobotStateDataFromEthernet.ee_AO[0],Item::NOT_REQUIRE };
		_item_map["End_AO1"            ] = { &rs->tmRobotStateDataFromEthernet.ee_AO[1],Item::NOT_REQUIRE };
		_item_map["End_AI0"            ] = { &rs->tmRobotStateDataFromEthernet.ee_AI[0] };
		_item_map["End_AI1"            ] = { &rs->tmRobotStateDataFromEthernet.ee_AI[1],Item::NOT_REQUIRE };
	}
	std::map<std::string, Item>  & get() { return _item_map; }
	std::map<std::string, Item>::iterator find(const std::string &name) { return _item_map.find(name); }
	std::map<std::string, Item>::iterator end() { return _item_map.end(); }
};

TmRobotState::TmRobotState()
{
	print_debug("TmRobotState::TmRobotState");

	_data_table = new TmDataTable(this);

	_f_deserialize_item[0] = std::bind(&TmRobotState::_deserialize_skip,
		std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
	_f_deserialize_item[1] = std::bind(&TmRobotState::_deserialize_copy_wo_check,
		std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);

	_f_deserialize = std::bind(&TmRobotState::_deserialize_first_time, this,
		std::placeholders::_1, std::placeholders::_2);
}

TmRobotState::~TmRobotState()
{
	print_debug("TmRobotState::~TmRobotState");
	delete _data_table;
}

void TmRobotState::set_fake_joint_states(const std::vector<double> &pos, const std::vector<double> &vel, const std::vector<double> &tor)
{
	for (size_t i = 0; i < 6; ++i) {
		tmRobotStateDataFromEthernet.joint_angle[i] = pos[i] * (180.0 / M_PI);
		tmRobotStateDataFromEthernet.joint_speed[i] = vel[i];
		tmRobotStateDataFromEthernet.joint_torque[i] = tor[i];
	}
	multiThreadCache.set_catch_data(tmRobotStateDataFromEthernet);
}

std::vector<double> TmRobotState::mtx_tcp_force_vec()
{
	std::vector<double> rv(tcp_force_vec().size());
	std::lock_guard<std::mutex> lck(mtx);
	rv = tcp_force_vec();
	return rv;
}

std::vector<double> TmRobotState::mtx_tcp_speed_vec()
{
	std::vector<double> rv(tcp_speed_vec().size());
	std::lock_guard<std::mutex> lck(mtx);
	rv = tcp_speed_vec();
	return rv;
}

std::vector<double> TmRobotState::mtx_joint_speed()
{
	std::vector<double> rv(joint_speed().size());
	std::lock_guard<std::mutex> lck(mtx);
	rv = joint_speed();
	return rv;
}

std::vector<double> TmRobotState::mtx_joint_torque()
{
	std::vector<double> rv(joint_torque().size());
	std::lock_guard<std::mutex> lck(mtx);
	rv = joint_torque();
	return rv;
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
	//size_t bsize = 2;
	unsigned short uslen; // 2 bytes

	// skip item name
	memcpy(&uslen, data + boffset, 2);
	boffset += 2 + uslen;
	// item data length
	memcpy(&uslen, data + boffset, 2);
	boffset += 2;
	// item data
	//bsize = uslen;
	memcpy(dst, data + boffset, uslen);
	boffset += uslen;
	return boffset;
}

size_t TmRobotState::_deserialize_first_time(const char *data, size_t size)
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
			print_debug(msg.c_str());
			++check_count;
		}
		else {
			//func = std::bind(&RobotState::_deserialize_skip, nullptr,
			//	std::placeholders::_2, std::placeholders::_3);
			std::string msg = "- " + item_name + " - skipped";
			print_debug(msg.c_str());
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

	multiThreadCache.set_catch_data(tmRobotStateDataFromEthernet);

	for (auto iter : _data_table->get()) {
		if (iter.second.required && !iter.second.checked) {
			isDataTableCorrect = false;
			std::string msg = "Required item" + iter.first + " is NOT checked";
			print_error(msg.c_str());
		}
	}

	if(isDataTableCorrect){
	  print_info("data table is correct!");
	} else{
	  print_error("data table is not correct!");
	}

	_f_deserialize = std::bind(&TmRobotState::_deserialize, this,
		std::placeholders::_1, std::placeholders::_2);

	return boffset;
}

size_t TmRobotState::_deserialize(const char *data, size_t size)
{
	size_t boffset = 0;

	for (auto &update : _item_updates) {
		boffset = _f_deserialize_item[update.func](update.dst, data, boffset);
	}

	multiThreadCache.set_catch_data(tmRobotStateDataFromEthernet);

	if (boffset > size) {
	}
	return boffset;
}

void TmRobotState::update_tm_robot_publish_state(){
	tmRobotStateDataToPublish = multiThreadCache.get_catch_data();
}

void TmRobotState::set_receive_state(TmCommRC state){
	_receive_state = state;
}
