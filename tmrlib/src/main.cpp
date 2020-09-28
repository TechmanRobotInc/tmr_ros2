#include <iostream>

#ifdef _WIN32
#include <conio.h>
#else
#include <string.h>
#include <unistd.h>
#include <termios.h>
#endif

#ifdef NO_INCLUDE_DIR
#include "tmr_print.h"
#include "tmr_driver.h"
#else
#include "tmr/tmr_print.h"
#include "tmr/tmr_driver.h"
#endif

#include <functional>

#ifdef _WIN32
void init_termios(int echo) {}
void reset_termios() {}
int kb_hit() { return _kbhit(); }
int get_char() { return _getche(); }
#else
static struct termios oldt, newt;
/* Initialize new terminal i/o settings */
void init_termios(int echo) {
	tcgetattr(STDIN_FILENO, &oldt); /* grab old terminal i/o settings */
	newt = oldt; /* make new settings same as old settings */
	newt.c_lflag &= ~ICANON; /* disable buffered i/o */
	newt.c_lflag &= echo ? ECHO : ~ECHO; /* set echo mode */
	tcsetattr(STDIN_FILENO, TCSANOW, &newt); /* use these new terminal i/o settings now */
}
/* Restore old terminal i/o settings */
void reset_termios() {
	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
}
int kb_hit() {
	struct timeval tv;
	fd_set rdfs;
	tv.tv_sec = 0;
	tv.tv_usec = 0;
	FD_ZERO(&rdfs);
	FD_SET(STDIN_FILENO, &rdfs);
	select(STDIN_FILENO + 1, &rdfs, NULL, NULL, &tv);
	return FD_ISSET(STDIN_FILENO, &rdfs);
}
int get_char() { return getchar(); }
#endif

void print_bytes(const std::vector<char> &bytes)
{
	std::cout << tmr::TmPacket::string_from_bytes(bytes);
}

void print_tm_packet(const tmr::TmPacket &pck, const std::string &name)
{
	std::cout << name << "\n";
	std::cout << "header: " << pck.header << "\n";
	std::cout << "data:   "; //<< pck.data << "\n";
	print_bytes(pck.data);
	std::cout << std::endl;
}

void tui_loop(const std::string &host)
{
	std::condition_variable svr_cv;
	std::condition_variable sct_cv;

	tmr::Driver iface{ host, &svr_cv, &sct_cv };

	int id_cnt = 0;

	char cstr[512];
	char delim[] = " ,;\t";
	char c;
	while (true) {
		memset(cstr, 0, 512);
		fgets(cstr, 512, stdin);
		int n = int(strlen(cstr));
		if (n > 0) {
			if (cstr[n - 1] == '\n') { cstr[n - 1] = '\0'; }
		}

		if (strncmp(cstr, "quit", 4) == 0) {
			iface.halt();
			break;
		}
		else if (strncmp(cstr, "start", 5) == 0) {
			iface.start();
		}
		else if (strncmp(cstr, "halt", 4) == 0) {
			iface.halt();
		}
		else if (strncmp(cstr, "show", 4) == 0) {

			iface.state.print();

			std::cout << "---\n";
		}
		else if (strncmp(cstr, "loop", 4) == 0) {
			char tmp_c = '\0';
			init_termios(1);
			while (true) {
				if (kb_hit()) {
					tmp_c = get_char();
					if (tmp_c == 'q' || tmp_c == 'Q')
						break;
				}
				//std::mutex lock;
				//std::unique_lock<std::mutex> locker{lock};
				//while (!iface.svr.state_updated()) {
				//	svr_cv.wait(locker);
				//}
				iface.svr.reset_state_update();
				iface.state.print();

				std::cout << "---\n";
			}
			reset_termios();
		}
		else if (strncmp(cstr, "stop", 4) == 0) {
			iface.sct.send_script_str(std::to_string(id_cnt), "StopAndClearBuffer()");
		}
		else if (strncmp(cstr, "home", 4) == 0) {
			std::string script = "PTP(\"JPP\",0,0,0,0,0,0,10,200,0,false)";
			iface.sct.send_script_str(std::to_string(id_cnt), script);
		}
		else if (strncmp(cstr, "movej0", 6) == 0) {
			std::vector<double> zeros;
			zeros.assign(6, 0.0);
			iface.set_joint_pos_PTP(zeros, 0.5, 0.2, 0);
		}
		else if (strncmp(cstr, "movejx", 6) == 0) {
			auto pose = iface.state.mtx_tool_pose();
			pose[0] += 0.01;
			iface.set_tool_pose_PTP(pose, 0.5, 0.2, 0);
		}
		else if (strncmp(cstr, "movejy", 6) == 0) {
			auto pose = iface.state.mtx_tool_pose();
			pose[1] += 0.01;
			iface.set_tool_pose_PTP(pose, 0.5, 0.2, 0);
		}
		else if (strncmp(cstr, "movejz", 6) == 0) {
			auto pose = iface.state.mtx_tool_pose();
			pose[2] += 0.01;
			iface.set_tool_pose_PTP(pose, 0.5, 0.2, 0);
		}
		else if (strncmp(cstr, "movelx", 6) == 0) {
			auto pose = iface.state.mtx_tool_pose();
			pose[0] += 0.01;
			iface.set_tool_pose_Line(pose, 0.1, 0.2, 0);
		}
		else if (strncmp(cstr, "movely", 6) == 0) {
			auto pose = iface.state.mtx_tool_pose();
			pose[1] += 0.01;
			iface.set_tool_pose_Line(pose, 0.1, 0.2, 0);
		}
		else if (strncmp(cstr, "movelz", 6) == 0) {
			auto pose = iface.state.mtx_tool_pose();
			pose[2] += 0.01;
			iface.set_tool_pose_Line(pose, 0.1, 0.2, 0);
		}
		else if (strncmp(cstr, "pvt_traj", 8) == 0) {
			tmr::PvtPoint point;
			tmr::PvtTraj traj;
			point.time = 5.0;
			point.positions.assign(6, 0.0);
			point.velocities.assign(6, 0.0);
			traj.mode = tmr::PvtMode::Joint;
			traj.points.push_back(point);
			traj.total_time = 5.0;

			std::thread(std::bind(&tmr::Driver::run_pvt_traj, &iface, traj)).detach();

			double t = 0.0;
			char tmp_c = '\0';
			init_termios(1);
			while (t < traj.total_time) {
				if (kb_hit()) {
					tmp_c = get_char();
					if (tmp_c == 'q' || tmp_c == 'Q') {
						iface.stop_pvt_traj();
						break;
					}
				}
				printf(".\n");
				std::this_thread::sleep_for(std::chrono::milliseconds(250));
				t += 0.25;
			}
			reset_termios();
		}
		else {
			std::string cmd{ cstr };
			std::cout << "send cmd: " << cmd << "\n";

			iface.sct.send_script_str(std::to_string(id_cnt), cmd);
		}

		++id_cnt;
		if (id_cnt > 9) { id_cnt = 9; }
	}
}

int main(int argc, char **argv)
{
	std::string host;
	if (argc > 1) {
		host = argv[1];
	}
	else {
		tmr_ERROR_STREAM("no ip-address");
		return 1;
	}
	tui_loop(host);
	return 0;
}
