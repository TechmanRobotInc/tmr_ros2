#pragma once

#define _USE_MATH_DEFINES
#include <math.h>

#include <vector>
#include <string>
#include <mutex>
#include <functional>


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

private:
	char _buf_[256];
	unsigned char _is_linked_;
	unsigned char _has_error_;
	unsigned char _is_proj_running_;
	unsigned char _is_proj_paused_;
	unsigned char _is_safeguard_A_triggered_;
	unsigned char _is_ESTOP_pressed_;
	char _camera_light_;

	int _error_code_;

	float _joint_angle_[DOF];
	float _flange_pose_[6];
	float _tool_pose_[6];
	float _tcp_frame_[6];
	float _tcp_mass_;
	float _tcp_cog_[6];

	float _tcp_force_vec_[3];
	float _tcp_force_;
	float _tcp_speed_vec_[6];
	float _tcp_speed_;
	float _joint_speed_[DOF];
	float _joint_torque_[DOF];
	float _joint_torque_average_[DOF] = {0};
	float _joint_torque_min_[DOF] = {0};
	float _joint_torque_max_[DOF] = {0};

	int _proj_speed_;
	int _ma_mode_;

	char _stick_play_pause_;

	int _robot_light_;

	unsigned char _ctrller_DO_[16];
	unsigned char _ctrller_DI_[16];
	float _ctrller_AO_[2];
	float _ctrller_AI_[2];
	unsigned char _ee_DO_[4];
	unsigned char _ee_DI_[4];
	float _ee_AO_[2];
	float _ee_AI_[2];
	bool isDataTableCorrect = false; 

private:
	unsigned char _is_linked;
	unsigned char _has_error;
	unsigned char _is_proj_running;
	unsigned char _is_proj_paused;
	unsigned char _is_safeguard_A_triggered;
	unsigned char _is_ESTOP_pressed;
	char _camera_light;

	int _error_code;
	std::string _error_content;

	std::vector<double> _joint_angle;	
	std::vector<double> _flange_pose;
	std::vector<double> _tool_pose;
	
	std::vector<double> _tcp_force_vec;
	double _tcp_force;
	std::vector<double> _tcp_speed_vec;
	double _tcp_speed;
	std::vector<double> _joint_speed;
	std::vector<double> _joint_torque;
    std::vector<double> _joint_torque_average;
	std::vector<double> _joint_torque_min;
	std::vector<double> _joint_torque_max;

	std::vector<double> _tcp_frame;
	double _tcp_mass;
	std::vector<double> _tcp_cog;

	int _proj_speed;
	int _ma_mode;

	unsigned char _stick_play_pause;

	int _robot_light;

	std::vector<unsigned char> _ctrller_DO;
	std::vector<unsigned char> _ctrller_DI;
	std::vector<float> _ctrller_AO;
	std::vector<float> _ctrller_AI;
	std::vector<unsigned char> _ee_DO;
	std::vector<unsigned char> _ee_DI;
	std::vector<float> _ee_AO;
	std::vector<float> _ee_AI;

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
	unsigned char is_linked() { return _is_linked; }
	unsigned char has_error() { return _has_error; }
    bool is_data_table_correct(){return isDataTableCorrect;}
	unsigned char is_project_running() { return _is_proj_running; }
	unsigned char is_project_paused() { return _is_proj_paused; }

	unsigned char is_safeguard_A() { return _is_safeguard_A_triggered; }
	unsigned char is_EStop() { return _is_ESTOP_pressed; }

	char camera_light() { return _camera_light; } // R/W

	int error_code() { return _error_code; }
	std::string error_content() { return _error_content; }

	std::vector<double> flange_pose() { return _flange_pose; }
	std::vector<double> joint_angle() { return _joint_angle; }
	std::vector<double> tool_pose() { return _tool_pose; }

	std::vector<double> tcp_force_vec() { return _tcp_force_vec; }
	double tcp_force() { return _tcp_force; }
	std::vector<double> tcp_speed_vec() { return _tcp_speed_vec; }
	double tcp_speed() { return _tcp_speed; }
	std::vector<double> joint_speed() { return _joint_speed; }
	std::vector<double> joint_torque() { return _joint_torque; }
	std::vector<double> joint_torque_average() { return _joint_torque_average; }
	std::vector<double> joint_torque_min() { return _joint_torque_min; }
	std::vector<double> joint_torque_max() { return _joint_torque_max; }

	int project_speed() { return _proj_speed; }
	int ma_mode() { return _ma_mode; }

	unsigned char stick_play_pause() { return _stick_play_pause; } // R/W

	int robot_light() { return _robot_light; }

	std::vector<unsigned char> ctrller_DO() { return _ctrller_DO; }
	std::vector<unsigned char> ctrller_DI() { return _ctrller_DI; }
	std::vector<float> ctrller_AO() { return _ctrller_AO; }
	std::vector<float> ctrller_AI() { return _ctrller_AI; }

	std::vector<unsigned char> ee_DO() { return _ee_DO; }
	std::vector<unsigned char> ee_DI() { return _ee_DI; }
	//std::vector<float> ee_AO() { return _ee_AO; }
	std::vector<float> ee_AI() { return _ee_AI; }

public:
	void mtx_lock() { mtx.lock(); }
	void mtx_unlock() { mtx.unlock(); }

	std::vector<double> mtx_flange_pose();
	std::vector<double> mtx_joint_angle();
	std::vector<double> mtx_tool_pose();

	std::string mtx_error_content();

	std::vector<unsigned char> mtx_ctrller_DO();
	std::vector<unsigned char> mtx_ctrller_DI();
	std::vector<float> mtx_ctrller_AO();
	std::vector<float> mtx_ctrller_AI();

	std::vector<unsigned char> mtx_ee_DO();
	std::vector<unsigned char> mtx_ee_DI();
	//std::vector<float> mtx_ee_AO();
	std::vector<float> mtx_ee_AI();

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
	static size_t _deserialize_get_name(std::string &name, const char *data, size_t offset);
	static size_t _deserialize_skip(void *dst, const char *data, size_t offset);
	static size_t _deserialize_copy_wo_check(void *dst, const char *data, size_t offset);
	size_t _deserialize_first_time(const char *data, size_t size, bool lock);
	size_t _deserialize(const char *data, size_t size, bool use_mtx);
	void _deserialize_update(bool lock);

public:
	size_t deserialize(const char *data, size_t size)
	{
		return _f_deserialize(data, size, false);
	}
	size_t mtx_deserialize(const char *data, size_t size)
	{
		return _f_deserialize(data, size, true);
	}

	void print();
};
