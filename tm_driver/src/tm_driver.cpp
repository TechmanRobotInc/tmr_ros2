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
	, sct{ ip, 2048, isOnListenNode }
	, state{ svr.state }
{
}

// has thread
TmDriver::TmDriver(const std::string &ip,
	std::condition_variable *psvr_cv,
	std::condition_variable *psct_cv)
	: svr{ ip, 4096, psvr_cv }
	, sct{ ip, 2048, isOnListenNode,psct_cv }
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

	stop_pvt_traj();

	if (sct.is_connected()) {
		// send command to stop project
		// sct.send_script_exit();
	}
	sct.halt();
	if (svr.is_connected()) {
		// send command to stop project
	}
	svr.halt();
}

bool TmDriver::get_connect_recovery_guide(){
	return connect_recovery_is_halt;
}

void TmDriver::set_connect_recovery_guide(bool is_halt){
	this->connect_recovery_is_halt = is_halt;
}

////////////////////////////////
// SVR Robot Function (write_XXX)
////////////////////////////////

////////////////////////////////
// SCT Robot Function (set_XXX)
////////////////////////////////
bool TmDriver::is_on_listen_node(){
	return isOnListenNode;
}

void TmDriver::back_to_listen_node(){
	isOnListenNode = true;
}

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
	print_info("%s\n", script.c_str());
	return (sct.send_script_str(id, script) == RC_OK);
}

bool TmDriver::run_pvt_traj(const TmPvtTraj &pvts)
{
	auto time_start = std::chrono::steady_clock::now();
	auto time_now = time_start;

	if (pvts.points.size() == 0) return false;

	if (!sct.is_connected()) return false;

	_is_executing_traj = true;

	if (!set_pvt_traj(pvts)) {
		_is_executing_traj = false; 
		return false;
	}

	// wait
	double time = 0.0;
	while (_is_executing_traj && time < pvts.total_time) {
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		time_now = std::chrono::steady_clock::now();
		time = std::chrono::duration_cast<std::chrono::duration<double>>(time_now - time_start).count();
		//time += 0.001;
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
}

void TmDriver::cubic_interp(TmPvtPoint &p, const TmPvtPoint &p0, const TmPvtPoint &p1, double t)
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

bool TmDriver::fake_run_pvt_traj(const TmPvtTraj &pvts)
{
	auto time_init = std::chrono::steady_clock::now();
	auto time_start = time_init;
	auto time_now = time_init;

	if (pvts.mode != TmPvtMode::Joint || pvts.points.size() < 2) return false;


	_is_executing_traj = true;

	TmPvtPoint p_start;
	p_start.time = 0.0;
	p_start.positions = state.joint_angle();
	p_start.velocities = state.mtx_joint_speed();
	TmPvtPoint &p0 = p_start;
	TmPvtPoint point = p_start;
	std::vector<double> zeros(state.DOF);
	size_t idx = 0;

	// first point
	//print_info(TmCommand::set_pvt_point(pvts.mode, p0));
	//print_info(TmCommand::set_pvt_point(pvts.mode, pvts.points[idx]));
	print_info(TmCommand::set_pvt_point(pvts.mode, p0).c_str());
	print_info(TmCommand::set_pvt_point(pvts.mode, pvts.points[idx]).c_str());
	point.time = pvts.points[0].time;

	while (_is_executing_traj) {
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

			print_info(TmCommand::set_pvt_point(pvts.mode, pvts.points[idx]).c_str());
		}
	}
	// last point
	if (_is_executing_traj) {
		idx = pvts.points.size() - 1;
		cubic_interp(point, pvts.points[idx - 1], pvts.points[idx], pvts.points[idx].time);
	}
	state.mtx_set_joint_states(point.positions, zeros, zeros);

	time_now = std::chrono::steady_clock::now();
	point.time = std::chrono::duration_cast<std::chrono::duration<double>>(time_now - time_init).count();
	print_info("TM_DRV: traj. exec. time:= %f", point.time);
	//RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),"TM_DRV: traj. exec. time:=" << point.time);

	_is_executing_traj = false;
	return true;
}

bool TmDriver::set_vel_mode_start(VelMode mode, double timeout_zero_vel, double timeout_stop, const std::string &id)
{
	return (sct.send_script_str(id, TmCommand::set_vel_mode_start(mode, timeout_zero_vel, timeout_stop)) == RC_OK);
}

bool TmDriver::set_vel_mode_stop(const std::string &id)
{
	return (sct.send_script_str(id, TmCommand::set_vel_mode_stop()) == RC_OK);
}

bool TmDriver::set_vel_mode_target(VelMode mode, const std::vector<double> &vel, const std::string &id)
{
	return (sct.send_script_str_silent(id, TmCommand::set_vel_mode_target(mode, vel)) == RC_OK);
}
