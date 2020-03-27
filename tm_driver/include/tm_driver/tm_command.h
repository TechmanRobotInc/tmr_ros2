#pragma once
#define _USE_MATH_DEFINES
#include <math.h>

#include <string>
#include <vector>

enum class TmIOModule { ControlBox, EndEffector };
enum class TmIOType { DI, DO, InstantDO, AI, AO, InstantAO };

enum class TmPvtMode { Joint, Tool };
struct TmPvtPoint {
	double time;
	std::vector<double> positions;
	std::vector<double> velocities;
};
struct TmPvtTraj {
	TmPvtMode mode;
	std::vector<TmPvtPoint> points;
	double total_time;
};

class TmCommand
{
public:
	////////////////////////////////
	// Unit conversion
	////////////////////////////////

	static double deg(double ang) { return (180.0 / M_PI) * ang; }
	static double rad(double ang) { return (M_PI / 180.0) * ang; }
	static std::vector<double> degs(const std::vector<double> &angs)
	{
		std::vector<double> rv(angs.size());
		for (size_t i = 0; i < angs.size(); ++i) {
			rv[i] = deg(angs[i]);
		}
		return rv;
	}
	static std::vector<double> rads(const std::vector<double> &angs)
	{
		std::vector<double> rv(angs.size());
		for (size_t i = 0; i < angs.size(); ++i) {
			rv[i] = rad(angs[i]);
		}
		return rv;
	}
	static std::vector<double> mmdeg_pose(std::vector<double> pose)
	{
		std::vector<double> rv(pose.size());
		if (pose.size() == 6) {
			for (size_t i = 0; i < 3; ++i) { rv[i] = 1000.0 * pose[i]; }
			for (size_t i = 3; i < 6; ++i) { rv[i] = deg(pose[i]); }
		}
		return rv;
	}
	static std::vector<double> m_rad_pose(std::vector<double> pose)
	{
		std::vector<double> rv(pose.size());
		if (pose.size() == 6) {
			for (size_t i = 0; i < 3; ++i) { rv[i] = 0.001 * pose[i]; }
			for (size_t i = 3; i < 6; ++i) { rv[i] = rad(pose[i]); }
		}
		return rv;
	}

	////////////////////////////////
	// Fucntions
	////////////////////////////////

	static std::string set_stop() { return "StopAndClearBuffer()"; }
	static std::string set_pause() { return "Pause()"; }
	static std::string set_resume() { return "Resume()"; }
	static std::string set_io(TmIOModule module, TmIOType type, int pin, float state);
	static std::string set_joint_pos_PTP(const std::vector<double> &angs,
		int vel_percent, double acc_time, int blend_percent, bool fine_goal, int precision = 5);
	static std::string set_tool_pose_PTP(const std::vector<double> &pose,
		int vel_percent, double acc_time, int blend_percent, bool fine_goal, int precision = 5);
	static std::string set_tool_pose_Line(const std::vector<double> &pose,
		double vel, double acc_time, int blend_percent, bool fine_goal, int precision = 5);

	static std::string set_pvt_enter(int mode) { return "PVTEnter(" + std::to_string(mode) + ")"; }
	static std::string set_pvt_exit() { return "PVTExit()"; }
	static std::string set_pvt_point(TmPvtMode mode,
		double t, const std::vector<double> &pos, const std::vector<double> &vel, int precision = 5);
	static std::string set_pvt_point(TmPvtMode mode, const TmPvtPoint &point, int precision = 5)
	{
		return set_pvt_point(mode, point.time, point.positions, point.velocities, precision);
	}
	static std::string set_pvt_traj(const TmPvtTraj &pvts, int precision = 5);
};