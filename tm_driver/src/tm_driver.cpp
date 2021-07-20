#ifdef NO_INCLUDE_DIR
#include "tm_driver.h"
#include "tm_print.h"
#else
#include "tm_driver/tm_driver.h"
#include "tm_driver/tm_print.h"
#endif

// no thread
TmDriver::TmDriver(const std::string &ip) 
	: svr{ ip, 4096 }
	, sct{ ip, 2048 }
	, state{ svr.state }
{
}

// has thread
TmDriver::TmDriver(const std::string &ip,
	std::condition_variable *psvr_cv,
	std::condition_variable *psct_cv)
	: svr{ ip, 4096, psvr_cv }
	, sct{ ip, 2048, psct_cv }
	, state{ svr.state }
{
	if (psvr_cv) {
		_svr_cv = psvr_cv;
		_has_svr_thrd = true;
	}
	if (psct_cv) {
		_sct_cv = psct_cv;
		_has_sct_thrd = true;
	}
}

bool TmDriver::start(int timeout_ms, bool stick_play)
{
	halt();
	print_info("TM_DRV: start");
	// connect to server
	bool rb = svr.start_tm_svr(timeout_ms);
	if (!rb) return rb;
	// send command to run project
	
	if (stick_play) {
		//svr.send_stick_play();
	}
	
	// connect to listen node
	rb = sct.start_tm_sct(timeout_ms);
	return rb;
}

void TmDriver::halt()
{
	print_info("TM_DRV: halt");
	if (sct.is_connected()) {
		// send command to stop project
		sct.send_script_exit();
	}
	sct.halt();
	if (svr.is_connected()) {
		// send command to stop project
	}
	svr.halt();
}

////////////////////////////////
// SVR Robot Function (write_XXX)
////////////////////////////////

////////////////////////////////
// SCT Robot Function (set_XXX)
////////////////////////////////

bool TmDriver::script_exit(const std::string &id)
{
	return (sct.send_script_str(id, TmCommand::script_exit()) == RC_OK);
}
bool TmDriver::set_tag(int tag, int wait, const std::string &id)
{
	return (sct.send_script_str(id, TmCommand::set_tag(tag, wait)) == RC_OK);
}
bool TmDriver::set_wait_tag(int tag, int timeout_ms, const std::string &id)
{
	return (sct.send_script_str(id, TmCommand::set_tag(tag, timeout_ms)) == RC_OK);
}
bool TmDriver::set_stop(const std::string &id)
{
	return (sct.send_script_str(id, TmCommand::set_stop()) == RC_OK);
}
bool TmDriver::set_pause(const std::string &id)
{
	return (sct.send_script_str(id, TmCommand::set_pause()) == RC_OK);
}
bool TmDriver::set_resume(const std::string &id)
{
	return (sct.send_script_str(id, TmCommand::set_resume()) == RC_OK);
}
bool TmDriver::set_io(TmIOModule module, TmIOType type, int pin, float state, const std::string &id)
{
	return (sct.send_script_str(id, TmCommand::set_io(module, type, pin, state)) == RC_OK);
}
bool TmDriver::set_joint_pos_PTP(const std::vector<double> &angs,
		double vel, double acc_time, int blend_percent, bool fine_goal, const std::string &id)
{
	int vel_pa = int(100.0 * (vel / _max_velocity));
	if (vel_pa >= 100) vel_pa = 100; //max 100%
	return (sct.send_script_str(
		id, TmCommand::set_joint_pos_PTP(angs, vel_pa, acc_time, blend_percent, fine_goal)
	) == RC_OK);
}
bool TmDriver::set_tool_pose_PTP(const std::vector<double> &pose,
		double vel, double acc_time, int blend_percent, bool fine_goal, const std::string &id)
{
	int vel_pa = int(100.0 * (vel / _max_velocity));
	return (sct.send_script_str(
		id, TmCommand::set_tool_pose_PTP(pose, vel_pa, acc_time, blend_percent, fine_goal)
	) == RC_OK);
}
bool TmDriver::set_tool_pose_Line(const std::vector<double> &pose,
		double vel, double acc_time, int blend_percent, bool fine_goal, const std::string &id)
{
	return (sct.send_script_str(
		id, TmCommand::set_tool_pose_Line(pose, vel, acc_time, blend_percent, fine_goal)
	) == RC_OK);
}

bool TmDriver::set_pvt_enter(TmPvtMode mode, const std::string &id)
{
	return (sct.send_script_str(id, TmCommand::set_pvt_enter(int(mode))) == RC_OK);
}
bool TmDriver::set_pvt_exit(const std::string &id)
{
	return (sct.send_script_str(id, TmCommand::set_pvt_exit()) == RC_OK);
}
bool TmDriver::set_pvt_point(TmPvtMode mode,
	double t, const std::vector<double> &pos, const std::vector<double> &vel, const std::string &id)
{
	if (t < 0.0 || pos.size() != vel.size() || pos.size() < 6) return false;

	return (sct.send_script_str(id, TmCommand::set_pvt_point(mode, t, pos, vel)) == RC_OK);
}
bool TmDriver::set_pvt_point(TmPvtMode mode, const TmPvtPoint &point, const std::string &id)
{
	return (sct.send_script_str(id, TmCommand::set_pvt_point(mode, point)) == RC_OK);
}

bool TmDriver::set_pvt_traj(const TmPvtTraj &pvts, const std::string &id)
{
	std::string script = TmCommand::set_pvt_traj(pvts);
	print_info("TM_DRV: send script (pvt traj.):\n");
	printf("%s\n", script.c_str());
	return (sct.send_script_str(id, script) == RC_OK);
}

bool TmDriver::run_pvt_traj(const TmPvtTraj &pvts)
{
	if (!sct.is_connected()) return false;

	_is_executing_traj = true;

	if (!set_pvt_traj(pvts)) {
		_is_executing_traj = false; return false;
	}

	// wait
	double time = 0.0;
	while (_is_executing_traj && time < pvts.total_time) {
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		time += 0.001;
	}

	if (_is_executing_traj) {
		_is_executing_traj = false;
	}
	else {
		set_stop();
	}
	print_info("TM_DRV: traj. exec. time:= %.3f", time);
	return true;
}
void TmDriver::stop_pvt_traj()
{
	_is_executing_traj = false;
	//set_stop();
}
