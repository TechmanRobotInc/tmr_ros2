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


TmRobotState::TmRobotState()
{
	print_debug("TmRobotState::TmRobotState");

	_joint_angle.assign(DOF, 0.0);
	_flange_pose.assign(6, 0.0);
	_tool_pose.assign(6, 0.0);
	_tcp_force_vec.assign(3, 0.0);
	_tcp_speed_vec.assign(6, 0.0);
	_joint_speed.assign(DOF, 0.0);
	_joint_torque.assign(DOF, 0.0);
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
}
TmRobotState::~TmRobotState()
{
	print_debug("TmRobotState::~TmRobotState");
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

size_t TmRobotState::_deserialize(const char *data, size_t size, bool use_mtx)
{
	size_t boffset = 0;
	size_t bsize = 0;
	unsigned short uslen; // 2 bytes
	/*union {
		size_t size;
		char bytes[2];
	} ulen;*/

	// Robot_Link (2 + 10 + 2 + 1 = 15)
	boffset = _deserialize_copy_wo_check(&_is_linked_, data, boffset); // 15

	// Robot_Error (2 + 11 + 2 + 1 = 16)
	boffset = _deserialize_copy_wo_check(&_has_error_, data, boffset); // 31

	// Project_Edit

	// Project_Run (2 + 11 + 2 + 1 = 16)
	boffset = _deserialize_copy_wo_check(&_is_proj_running_, data, boffset); // 47
	
	// Project_Pause
	boffset = _deserialize_copy_wo_check(&_is_proj_paused_, data, boffset);

	// Safeguard_A
	boffset = _deserialize_copy_wo_check(&_is_safeguard_A_triggered_, data, boffset);

	// ESTOP
	boffset = _deserialize_copy_wo_check(&_is_ESTOP_pressed_, data, boffset);

	// Camera_Light
	boffset = _deserialize_copy_wo_check(&_camera_light_, data, boffset);

	// Robot_Model

	// Error_Code (2 + 10 + 2 + 4 = 18)
	boffset = _deserialize_copy_wo_check(&_error_code_, data, boffset);

	// Error_Content
	/*if (use_mtx) mtx_lock();
	{
		bsize = 2;
		memcpy(&uslen, data + boffset, bsize);
		boffset += bsize + uslen; // = 13
		memcpy(&uslen, data + boffset, bsize);
		boffset += bsize;
		if (uslen > 0) {
			bsize = uslen;
			{
				std::stringstream ss;
				for (size_t i = 0; i < bsize; ++i) {
					ss << *(data + boffset + i);
				}
				_error_content = ss.str();
			}
			boffset += bsize;
		}
	}
	if (use_mtx) mtx_unlock();*/

	// Error_Time

	// Coord_Base_Flange
	//boffset = _deserialize_copy_wo_check(&_flange_pose_, data, boffset);

	// Joint_Angle
	boffset = _deserialize_copy_wo_check(&_joint_angle_, data, boffset);

	// Coord_Base_Tool
	//boffset = _deserialize_copy_wo_check(&_tool_pose_, data, boffset);

	// Coord_Robot_Flange
	boffset = _deserialize_copy_wo_check(&_flange_pose_, data, boffset);

	// Coord_Robot_Tool
	boffset = _deserialize_copy_wo_check(&_tool_pose_, data, boffset);

	// TCP_Force 12
	boffset = _deserialize_copy_wo_check(&_tcp_force_vec_, data, boffset);

	// TCP_Force3D 4
	boffset = _deserialize_copy_wo_check(&_tcp_force_, data, boffset);

	// TCP_Speed 24
	boffset = _deserialize_copy_wo_check(&_tcp_speed_vec_, data, boffset);

	// TCP_Speed3D 4
	boffset = _deserialize_copy_wo_check(&_tcp_speed_, data, boffset);

	// Joint_Speed 24
	boffset = _deserialize_copy_wo_check(&_joint_speed_, data, boffset);

	// Joint_Torque 24
	boffset = _deserialize_copy_wo_check(&_joint_torque_, data, boffset);

	// TCP_Name

	// TCP_Value
	//boffset = _deserialize_copy_wo_check(&_tcp_frame_, data, boffset);

	// TCP_Mass
	//boffset = _deserialize_copy_wo_check(&_tcp_mass_, data, boffset);

	// TCP_MCF
	//boffset = _deserialize_copy_wo_check(&_tcp_cog_, data, boffset);

	// Project_Name

	// Project_Speed 1
	boffset = _deserialize_copy_wo_check(&_proj_speed_, data, boffset);

	// MA_Mode 4
	boffset = _deserialize_copy_wo_check(&_ma_mode_, data, boffset);

	// Stick_PlayPause
	//boffset = _deserialize_copy_wo_check(&_stick_play_pause_, data, boffset);

	// Robot Light 4
	boffset = _deserialize_copy_wo_check(&_robot_light_, data, boffset);

	// Ctrl_DOx
	for (int i = 0; i < 8; ++i) {
		boffset = _deserialize_copy_wo_check(&_ctrller_DO_[i], data, boffset);
	}
	// Ctrl_DIx
	for (int i = 0; i < 8; ++i) {
		boffset = _deserialize_copy_wo_check(&_ctrller_DI_[i], data, boffset);
	}
	// Ctrl_AOx
	for (int i = 0; i < 1; ++i) {
		boffset = _deserialize_copy_wo_check(&_ctrller_AO_[i], data, boffset);
	}
	// Ctrl_AIx
	for (int i = 0; i < 2; ++i) {
		boffset = _deserialize_copy_wo_check(&_ctrller_AI_[i], data, boffset);
	}
	// End_DOx
	for (int i = 0; i < 4; ++i) {
		boffset = _deserialize_copy_wo_check(&_ee_DO_[i], data, boffset);
	}
	// End_DIx
	for (int i = 0; i < 3; ++i) {
		boffset = _deserialize_copy_wo_check(&_ee_DI_[i], data, boffset);
	}
	// End_AOx
	//for (int i = 0; i < 1; ++i) {
	//	boffset = _deserialize_copy_wo_check(&_ee_AO_[i], data, boffset);
	//}
	// End_AIx
	for (int i = 0; i < 1; ++i) {
		boffset = _deserialize_copy_wo_check(&_ee_AI_[i], data, boffset);
	}

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

	if (use_mtx) mtx_lock();
	{
		_joint_angle = rads(_joint_angle_, DOF);

		si_pose(_flange_pose, _flange_pose_, 6);

		si_pose(_tool_pose, _tool_pose_, 6);

		for (int i = 0; i < 3; ++i) { _tcp_force_vec[i] = double(_tcp_force_vec_[i]); }

		_tcp_force = double(_tcp_force_);

		si_pose(_tcp_speed_vec, _tcp_speed_vec_, 6);

		_joint_speed = rads(_joint_speed_, DOF);

		_joint_torque = meters(_joint_torque_, DOF);

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
	if (use_mtx) mtx_unlock();

	return boffset;
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
