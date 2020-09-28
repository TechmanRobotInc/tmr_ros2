#ifdef NO_INCLUDE_DIR
#include "tmr_driver.h"
#include "tmr_print.h"
#else
#include "tmr/tmr_driver.h"
#include "tmr/tmr_print.h"
#endif

namespace tmr
{

// no thread
Driver::Driver(const std::string &ip) 
	: svr{ ip, 4096 }
	, sct{ ip, 2048 }
	, state{ svr.state }
{
}

// has thread
Driver::Driver(const std::string &ip,
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

bool Driver::start(int timeout_ms, bool stick_play)
{
	halt();
	tmr_INFO_STREAM("TM_DRV: start");
	// connect to server
	bool rb = svr.start(timeout_ms);
	if (!rb) return rb;
	// send command to run project
	if (stick_play) {
		svr.send_stick_play();
	}
	// connect to listen node
	rb = sct.start(timeout_ms);
	return rb;
}

void Driver::halt()
{
	tmr_INFO_STREAM("TM_DRV: halt");

	stop_pvt_traj();

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

bool Driver::script_exit(const std::string &id)
{
	return (sct.send_script_str(id, Command::script_exit()) == RC_OK);
}
bool Driver::set_tag(int tag, int wait, const std::string &id)
{
	return (sct.send_script_str(id, Command::set_tag(tag, wait)) == RC_OK);
}
bool Driver::set_wait_tag(int tag, int timeout_ms, const std::string &id)
{
	return (sct.send_script_str(id, Command::set_tag(tag, timeout_ms)) == RC_OK);
}
bool Driver::set_stop(const std::string &id)
{
	return (sct.send_script_str(id, Command::set_stop()) == RC_OK);
}
bool Driver::set_pause(const std::string &id)
{
	return (sct.send_script_str(id, Command::set_pause()) == RC_OK);
}
bool Driver::set_resume(const std::string &id)
{
	return (sct.send_script_str(id, Command::set_resume()) == RC_OK);
}
bool Driver::set_io(IOModule module, IOType type, int pin, float state, const std::string &id)
{
	return (sct.send_script_str(id, Command::set_io(module, type, pin, state)) == RC_OK);
}
bool Driver::set_joint_pos_PTP(const std::vector<double> &angs,
		double vel, double acc_time, int blend_percent, bool fine_goal, const std::string &id)
{
	int vel_pa = int(100.0 * (vel / _max_velocity));
	return (sct.send_script_str(
		id, Command::set_joint_pos_PTP(angs, vel_pa, acc_time, blend_percent, fine_goal)
	) == RC_OK);
}
bool Driver::set_tool_pose_PTP(const std::vector<double> &pose,
		double vel, double acc_time, int blend_percent, bool fine_goal, const std::string &id)
{
	int vel_pa = int(100.0 * (vel / _max_velocity));
	return (sct.send_script_str(
		id, Command::set_tool_pose_PTP(pose, vel_pa, acc_time, blend_percent, fine_goal)
	) == RC_OK);
}
bool Driver::set_tool_pose_Line(const std::vector<double> &pose,
		double vel, double acc_time, int blend_percent, bool fine_goal, const std::string &id)
{
	return (sct.send_script_str(
		id, Command::set_tool_pose_Line(pose, vel, acc_time, blend_percent, fine_goal)
	) == RC_OK);
}

bool Driver::set_pvt_enter(PvtMode mode, const std::string &id)
{
	return (sct.send_script_str(id, Command::set_pvt_enter(int(mode))) == RC_OK);
}
bool Driver::set_pvt_exit(const std::string &id)
{
	return (sct.send_script_str(id, Command::set_pvt_exit()) == RC_OK);
}
bool Driver::set_pvt_point(PvtMode mode,
	double t, const std::vector<double> &pos, const std::vector<double> &vel, const std::string &id)
{
	if (t < 0.0 || pos.size() != vel.size() || pos.size() < 6) return false;

	return (sct.send_script_str(id, Command::set_pvt_point(mode, t, pos, vel)) == RC_OK);
}
bool Driver::set_pvt_point(PvtMode mode, const PvtPoint &point, const std::string &id)
{
	return (sct.send_script_str_silent(id, Command::set_pvt_point(mode, point)) == RC_OK);
}

bool Driver::set_pvt_traj(const PvtTraj &pvts, const std::string &id)
{
	std::string script = Command::set_pvt_traj(pvts);
	tmr_INFO_STREAM("TM_DRV: send script (pvt traj.):");
	std::cout << script << "\n";
	return (sct.send_script_str_silent(id, script) == RC_OK);
}

bool Driver::run_pvt_traj(const PvtTraj &pvts)
{
	auto time_start = std::chrono::steady_clock::now();
	auto time_now = time_start;

	if (pvts.points.size() == 0) return false;

	if (!sct.is_connected()) return false;

	_is_keep_exec_traj = true;

	if (!set_pvt_traj(pvts)) {
		_is_keep_exec_traj = false;
		return false;
	}

	// wait
	double time = 0.0;
	while (_is_keep_exec_traj && time < pvts.total_time) {
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		time_now = std::chrono::steady_clock::now();
		time = std::chrono::duration_cast<std::chrono::duration<double>>(time_now - time_start).count();
		//time += 0.001;
	}

	if (_is_keep_exec_traj) {
		_is_keep_exec_traj = false;
	}
	else {
		set_stop();
	}
	tmr_INFO_STREAM("TM_DRV: traj. exec. time:=" << time);
	return true;
}
void Driver::stop_pvt_traj()
{
	_is_keep_exec_traj = false;
}

void Driver::cubic_interp(PvtPoint &p, const PvtPoint &p0, const PvtPoint &p1, double t)
{
	double c, d, T = p1.time;

	if (t < 0.0) t = 0.0;
	else if (t > T) t = T;

	p.time = t;

	for (size_t i = 0; i < p.positions.size(); ++i) {
		c = ((3.0 * (p1.positions[i] - p0.positions[i]) / T) - 2.0 * p0.velocities[i] - p1.velocities[i]) / T;
		d = ((2.0 * (p0.positions[i] - p1.positions[i]) / T) + p0.velocities[i] + p1.velocities[i]) / (T*T);
		p.positions[i] = p0.positions[i] + p0.velocities[i] * t + c * t*t + d * t*t*t;
		p.velocities[i] = p0.velocities[i] + 2.0 * c * t + 3.0 * d * t*t;
	}
}
bool Driver::fake_run_pvt_traj(const PvtTraj &pvts)
{
	auto time_init = std::chrono::steady_clock::now();
	auto time_start = time_init;
	auto time_now = time_init;

	if (pvts.mode != PvtMode::Joint || pvts.points.size() < 2) return false;

	//for (auto &p : pvts.points) tmr_INFO_STREAM(Command::set_pvt_point(pvts.mode, p));

	_is_keep_exec_traj = true;

	PvtPoint p_start;
	p_start.time = 0.0;
	p_start.positions = state.mtx_joint_angle();
	p_start.velocities = state.mtx_joint_speed();
	PvtPoint &p0 = p_start;
	PvtPoint point = p_start;
	std::vector<double> zeros(state.DOF);
	size_t idx = 0;

	// first point
	tmr_INFO_STREAM(Command::set_pvt_point(pvts.mode, p0));
	tmr_INFO_STREAM(Command::set_pvt_point(pvts.mode, pvts.points[idx]));
	point.time = pvts.points[0].time;

	while (_is_keep_exec_traj) {
		cubic_interp(point, p0, pvts.points[idx], point.time);
		state.mtx_set_joint_states(point.positions, point.velocities, zeros);

		std::this_thread::sleep_for(std::chrono::milliseconds(4));

		time_now = std::chrono::steady_clock::now();
		point.time = std::chrono::duration_cast<std::chrono::duration<double>>(time_now - time_start).count();
		if (point.time > pvts.points[idx].time) {
			p0 = pvts.points[idx];
			point.time -= pvts.points[idx].time;
			time_start = time_now;
			++idx;
			if (idx == pvts.points.size()) break;

			tmr_INFO_STREAM(Command::set_pvt_point(pvts.mode, pvts.points[idx]));
		}
	}
	// last point
	if (_is_keep_exec_traj) {
		idx = pvts.points.size() - 1;
		cubic_interp(point, pvts.points[idx - 1], pvts.points[idx], pvts.points[idx].time);
	}
	state.mtx_set_joint_states(point.positions, zeros, zeros);

	time_now = std::chrono::steady_clock::now();
	point.time = std::chrono::duration_cast<std::chrono::duration<double>>(time_now - time_init).count();
	tmr_INFO_STREAM("TM_DRV: traj. exec. time:=" << point.time);

	_is_keep_exec_traj = false;
	return true;
}




bool Driver::set_vel_mode_start(VelMode mode, double timeout_zero_vel, double timeout_stop, const std::string &id)
{
	return (sct.send_script_str(id, Command::set_vel_mode_start(mode, timeout_zero_vel, timeout_stop)) == RC_OK);
}
bool Driver::set_vel_mode_stop(const std::string &id)
{
	return (sct.send_script_str(id, Command::set_vel_mode_stop()) == RC_OK);
}
bool Driver::set_vel_mode_target(VelMode mode, const std::vector<double> &vel, const std::string &id)
{
	return (sct.send_script_str_silent(id, Command::set_vel_mode_target(mode, vel)) == RC_OK);
}

}