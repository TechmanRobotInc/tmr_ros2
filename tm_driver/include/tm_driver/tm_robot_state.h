#pragma once

#define _USE_MATH_DEFINES
#include <math.h>

#include <vector>
#include <string>
#include <mutex>
#include <functional>
#include "tm_driver_utilities.h"

struct TmRobotStateData
{
	unsigned char is_linked;
	unsigned char has_error;
	unsigned char is_proj_running;
	unsigned char is_proj_paused;
	unsigned char is_safeguard_A_triggered;
	unsigned char is_ESTOP_pressed;
	unsigned char camera_light;
	int error_code;
	float joint_angle[6]={0};
	float flange_pose[6];
	float tool_pose[6];
	float tcp_frame[6];
	float tcp_mass;
	float tcp_cog[6];
	float tcp_force_vec[3];
	float tcp_force;
	float tcp_speed_vec[6];
	float tcp_speed;
	float joint_speed[6];
	float joint_torque[6];
	float joint_torque_average[6] = {0};
	float joint_torque_min[6] = {0};
	float joint_torque_max[6] = {0};
	int proj_speed;
	int ma_mode;
	char stick_play_pause;
	int robot_light;
	unsigned char ctrller_DO[16];
	unsigned char ctrller_DI[16];
	float ctrller_AO[2];
	float ctrller_AI[2];
	unsigned char ee_DO[4];
	unsigned char ee_DI[4];
	float ee_AO[2];
	float ee_AI[2];
};

class TmDataTable;

class TmRobotState
{
	friend class TmDataTable;
private:
	TmDataTable *_data_table;

public:
	enum { DOF = 6 };

private:
	std::mutex mtx;
	MultiThreadCache<TmRobotStateData> multiThreadCache;

private:
	char _buf_[256];
	TmRobotStateData tmRobotStateDataFromEthernet;
	bool isDataTableCorrect = false; 

private:
    TmRobotStateData tmRobotStateDataToPublish;
	
    TmCommRC _receive_state;

private:
	std::function<size_t (void *, const char *, size_t)> _f_deserialize_item[2];
	std::function<size_t (const char *, size_t, bool)> _f_deserialize;
	struct ItemUpdate {
		void *dst;
		size_t func;
		enum { SKIP, UPDATE };
	};
	std::vector<ItemUpdate> _item_updates;

public:
	TmRobotState();
	~TmRobotState();

public:
	unsigned char is_linked() { return tmRobotStateDataToPublish.is_linked; }
	unsigned char has_error() { return tmRobotStateDataToPublish.has_error; }
	bool is_data_table_correct() { return isDataTableCorrect; }
	unsigned char is_project_running() { return tmRobotStateDataToPublish.is_proj_running; }
	unsigned char is_project_paused() { return tmRobotStateDataToPublish.is_proj_paused; }
	unsigned char is_safeguard_A() { return tmRobotStateDataToPublish.is_safeguard_A_triggered; }
	unsigned char is_EStop() { return tmRobotStateDataToPublish.is_ESTOP_pressed; }
	unsigned char camera_light() { return tmRobotStateDataToPublish.camera_light; } // R/W
	int error_code() { return tmRobotStateDataToPublish.error_code; }
	std::string error_content() { return ""; }

	std::vector<double> flange_pose() { 
		std::vector<double>  flangePose;
		flangePose.assign(6, 0.0);
		si_pose(flangePose, tmRobotStateDataToPublish.flange_pose, 6);
		return flangePose;
	}
	std::vector<double> joint_angle(){
		std::vector<double>  jointAngle;
		jointAngle.assign(6, 0.0);
		jointAngle = rads(tmRobotStateDataToPublish.joint_angle, 6);
		return jointAngle;
	}
	std::vector<double> tool_pose(){
		std::vector<double>  toolPose;
		toolPose.assign(6, 0.0);
		si_pose(toolPose, tmRobotStateDataToPublish.tool_pose, 6);
		return toolPose;
	}
	std::vector<double> tcp_force_vec(){
		std::vector<double>  tcpForceVec;
		tcpForceVec.assign(3, 0.0);
		for (int i = 0; i < 3; ++i) { tcpForceVec[i] = double(tmRobotStateDataToPublish.tcp_force_vec[i]); }
		return tcpForceVec;
	}
	double tcp_force() { 
		return tmRobotStateDataToPublish.tcp_force; 
	}
	std::vector<double> tcp_speed_vec(){
		std::vector<double>  tcpSpeedVec;
		tcpSpeedVec.assign(6, 0.0);
		si_pose(tcpSpeedVec, tmRobotStateDataToPublish.tcp_speed_vec, 6);
		return tcpSpeedVec;
	}
	double tcp_speed() { return tmRobotStateDataToPublish.tcp_speed; }
	std::vector<double> joint_speed(){
		std::vector<double>  jointSpeed;
		jointSpeed.assign(6, 0.0);
		jointSpeed = rads( tmRobotStateDataToPublish.joint_speed, 6);
		return jointSpeed;
	} 
	std::vector<double> joint_torque(){
		std::vector<double>  jointTorque;
		jointTorque.assign(6, 0.0);
		jointTorque = meters( tmRobotStateDataToPublish.joint_torque, 6);
		return jointTorque;
	}
	std::vector<double> joint_torque_average(){
		std::vector<double>  jointTorqueAverage;
		jointTorqueAverage.assign(6, 0.0);
		jointTorqueAverage = meters( tmRobotStateDataToPublish.joint_torque_average, 6);
		return jointTorqueAverage;
	}
	std::vector<double> joint_torque_min() {
		std::vector<double>  jointTorqueMin;
		jointTorqueMin.assign(6, 0.0);
		jointTorqueMin = meters( tmRobotStateDataToPublish.joint_torque_min, 6);
		return jointTorqueMin;
	}
	std::vector<double> joint_torque_max() {
		std::vector<double>  jointTorqueMax;
		jointTorqueMax.assign(6, 0.0);
		jointTorqueMax = meters( tmRobotStateDataToPublish.joint_torque_max, 6);
		return jointTorqueMax;
	}

	int project_speed() { return tmRobotStateDataToPublish.proj_speed; }
	int ma_mode() { return tmRobotStateDataToPublish.ma_mode; }
	unsigned char stick_play_pause() { return tmRobotStateDataToPublish.stick_play_pause; } // R/W
	int robot_light() { return tmRobotStateDataToPublish.robot_light; }

	std::vector<unsigned char> ctrller_DO(){
		std::vector<unsigned char>  ctrllerDO;
		ctrllerDO.assign(8, 0.0);
		for (int i = 0; i < 8; ++i) { ctrllerDO[i] = tmRobotStateDataToPublish.ctrller_DO[i]; }
		return ctrllerDO;
	}
	std::vector<unsigned char> ctrller_DI() {
		std::vector<unsigned char>  ctrllerDI;
		ctrllerDI.assign(8, 0.0);
		for (int i = 0; i < 8; ++i) { ctrllerDI[i] = tmRobotStateDataToPublish.ctrller_DI[i]; }
	    return ctrllerDI; 
	}
	std::vector<float> ctrller_AO(){
		std::vector<float>  ctrllerAO;
		ctrllerAO.assign(1, 0.0);
		for (int i = 0; i < 1; ++i) { ctrllerAO[i] = tmRobotStateDataToPublish.ctrller_AO[i]; }
	    return ctrllerAO; 
	}
	std::vector<float> ctrller_AI(){
        std::vector<float>  ctrllerAI;
		ctrllerAI.assign(2, 0.0);
		for (int i = 0; i < 2; ++i) { ctrllerAI[i] = tmRobotStateDataToPublish.ctrller_AI[i]; }
	    return ctrllerAI; 
	}
	std::vector<unsigned char> ee_DO(){
		std::vector<unsigned char>  eeDO;
		eeDO.assign(4, 0.0);
		for (int i = 0; i < 4; ++i) { eeDO[i] = tmRobotStateDataToPublish.ee_DO[i]; }
	    return eeDO; 
	}
	std::vector<unsigned char> ee_DI(){
		std::vector<unsigned char>  eeDI;
		eeDI.assign(4, 0.0);
		for (int i = 0; i < 4; ++i) { eeDI[i] = tmRobotStateDataToPublish.ee_DI[i]; }
	    return eeDI; 
	}	
	std::vector<float> ee_AI(){
        std::vector<float>  eeAI;
		eeAI.assign(1, 0.0);
		for (int i = 0; i < 1; ++i) { eeAI[i] = tmRobotStateDataToPublish.ee_AI[i]; }
	    return eeAI; 
	}
	
    TmCommRC get_receive_state(){return _receive_state;}

	void set_fake_joint_states(
		const std::vector<double> &pos,
		const std::vector<double> &vel,
		const std::vector<double> &tor);

public:
	void mtx_lock() { mtx.lock(); }
	void mtx_unlock() { mtx.unlock(); }

	std::vector<double> mtx_tcp_force_vec();
	std::vector<double> mtx_tcp_speed_vec();

	std::vector<double> mtx_joint_speed();
	std::vector<double> mtx_joint_torque();

	void mtx_set_joint_states(
		const std::vector<double> &pos,
		const std::vector<double> &vel,
		const std::vector<double> &tor)
	{
		std::lock_guard<std::mutex> lck(mtx);
		set_fake_joint_states(pos, vel, tor);
	}

private:
	static double meter(double mm)
	{
		return 0.001 * mm;
	}
	static double rad(double ang)
	{
		return (M_PI / 180.0) * ang;
	}
	template<typename T>
	static std::vector<double> meters(const T *mms, size_t size)
	{
		std::vector<double> rv(size);
		for (size_t i = 0; i < size; ++i) {
			rv[i] = meter(double(mms[i]));
		}
		return rv;
	}
	template<typename T>
	static std::vector<double> rads(const T *degs, size_t size)
	{
		std::vector<double> rv(size);
		for (size_t i = 0; i < size; ++i) {
			rv[i] = rad(double(degs[i]));
		}
		return rv;
	}
	template<typename T>
	static void si_pose(std::vector<double> &dst, T *src, size_t size = 6)
	{
		for (size_t i = 0; i < 3; ++i) { dst[i] = meter(double(src[i])); }
		for (size_t i = 3; i < size; ++i) { dst[i] = rad(double(src[i])); }
	}

private:
	static size_t _deserialize_skip(void *dst, const char *data, size_t offset);
	static size_t _deserialize_copy_wo_check(void *dst, const char *data, size_t offset);
	size_t _deserialize_first_time(const char *data, size_t size);
	size_t _deserialize(const char *data, size_t size);

public:
	size_t deserialize(const char *data, size_t size)
	{
		return _f_deserialize(data, size, false);
	}
	size_t mtx_deserialize(const char *data, size_t size)
	{
		return _f_deserialize(data, size, true);
	}
	void set_receive_state(TmCommRC state);
	void update_tm_robot_publish_state();
	void print();
};
