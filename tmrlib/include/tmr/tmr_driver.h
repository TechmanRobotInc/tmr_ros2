#pragma once
#include "tmr_communication.h"
#include "tmr_tmsvr_communication.h"
#include "tmr_tmsct_communication.h"
#include "tmr_command.h"

namespace tmr
{

class Driver
{
public:
	TmSvrCommunication svr;
	TmSctCommunication sct;
	RobotState &state;

	//const int DOF = state.DOF;
	const CommRC RC_OK = CommRC::OK;
private:
	std::condition_variable *_svr_cv = nullptr;
	std::condition_variable *_sct_cv = nullptr;
	bool _has_svr_thrd = false;
	bool _has_sct_thrd = false;

	bool _is_keep_exec_traj = false;

	////////////////////////////////
	// tm_driver Param.
	////////////////////////////////

	double _max_velocity = M_PI;
	double _max_tcp_speed = 1.0;
	double _max_payload = 4.0;

public:
	explicit Driver(const std::string &ip);
	explicit Driver(const std::string &ip,
		std::condition_variable *psvr_cv,
		std::condition_variable *psct_cv);

	// start: connect to server, run project, connect to listen node
	bool start(int timeout_ms = -1, bool stick_play = true);

	// halt: disconnect to listen node, stop project, disconnect to server
	void halt();

	////////////////////////////////
	// tm_driver Param.
	////////////////////////////////

	void set_this_max_velocity(double max_vel) { _max_velocity = max_vel; }
	void set_this_max_tcp_speed(double max_spd) { _max_tcp_speed = max_spd; }
	void set_this_max_payload(double payload) { _max_payload = payload; }

	////////////////////////////////
	// SVR Robot Function (write_XXX)
	////////////////////////////////


	////////////////////////////////
	// SCT Robot Function (set_XXX)
	////////////////////////////////

	bool script_exit(const std::string &id = "Exit");
	bool set_tag(int tag, int wait = 0, const std::string &id = "Tag");
	bool set_wait_tag(int tag, int timeout_ms = 0, const std::string &id = "WaitTag");
	bool set_stop(const std::string &id = "Stop");
	bool set_pause(const std::string &id = "Pause");
	bool set_resume(const std::string &id = "Resume");

	//enum class IOModule { ControlBox, EndEffector };
	//enum class IOType { DI, DO, InstantDO, AI, AO, InstantAO };
	bool set_io(IOModule module, IOType type, int pin, float state, const std::string &id = "io");
	bool set_joint_pos_PTP(const std::vector<double> &angs,
		double vel, double acc_time, int blend_percent, bool fine_goal = false, const std::string &id = "PTPJ");
	bool set_tool_pose_PTP(const std::vector<double> &pose,
		double vel, double acc_time, int blend_percent, bool fine_goal = false, const std::string &id = "PTPT");
	bool set_tool_pose_Line(const std::vector<double> &pose,
		double vel, double acc_time, int blend_percent, bool fine_goal = false, const std::string &id = "Line");
	// set_tool_pose_PLINE

	//
	// PVT Trajectory
	//

	bool set_pvt_enter(PvtMode mode, const std::string &id = "PvtEnter");
	bool set_pvt_exit(const std::string &id = "PvtExit");
	bool set_pvt_point(PvtMode mode,
		double t, const std::vector<double> &pos, const std::vector<double> &vel, const std::string &id = "PvtPt");
	bool set_pvt_point(PvtMode mode, const PvtPoint &point, const std::string &id = "PvtPt");

	bool set_pvt_traj(const PvtTraj &pvts, const std::string &id = "PvtTraj");

	bool run_pvt_traj(const PvtTraj &pvts);
	void stop_pvt_traj();

	void cubic_interp(PvtPoint &p, const PvtPoint &p0, const PvtPoint &p1, double t);
	bool fake_run_pvt_traj(const PvtTraj &pvts);


	bool set_vel_mode_start(VelMode mode, double timeout_zero_vel, double timeout_stop, const std::string &id = "VModeStart");
	bool set_vel_mode_stop(const std::string &id = "VModeStop");
	bool set_vel_mode_target(VelMode mode, const std::vector<double> &vel, const std::string &id = "VModeTrgt");

};

}