#include "tm_driver.h"
#include "tm_print.h"

#include <rclcpp/rclcpp.hpp>

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "tm_msgs/msg/sct_response.hpp"
#include "tm_msgs/msg/sta_response.hpp"

#include "tm_msgs/srv/connect_tm.hpp"
#include "tm_msgs/srv/write_item.hpp"
#include "tm_msgs/srv/ask_item.hpp"
#include "tm_msgs/srv/send_script.hpp"
#include "tm_msgs/srv/set_event.hpp"
#include "tm_msgs/srv/set_io.hpp"
//#include "tm_msgs/srv/set_payload.hpp"
#include "tm_msgs/srv/set_positions.hpp"
#include "tm_msgs/srv/ask_sta.hpp"


class TmSctRos2
{
public:
    rclcpp::Node::SharedPtr node;
    TmSctCommunication &sct_;
    TmSvrCommunication &svr_;
    TmDriver &iface_;
    TmRobotState &state_;

protected:
  bool is_fake_;

public:
    std::vector<std::string> jns_;

    std::mutex checkIsOnListenNodeMutex;
    std::condition_variable checkIsOnListenNodeCondVar;
    std::mutex firstCheckIsOnListenNodeMutex;
    std::condition_variable firstCheckIsOnListenNodeCondVar;

    struct SctAndStaMsg {
        rclcpp::Publisher<tm_msgs::msg::SctResponse>::SharedPtr sct_pub;
        rclcpp::Publisher<tm_msgs::msg::StaResponse>::SharedPtr sta_pub;

        tm_msgs::msg::SctResponse sct_msg;
        tm_msgs::msg::StaResponse sta_msg;
    } sm_;

    bool sta_updated_;
    std::mutex sta_mtx_;
    std::condition_variable sta_cv_;

    int sct_reconnect_timeout_ms_;
    int sct_reconnect_timeval_ms_;
    std::thread sct_thread_;
    std::thread checkListenNodeThread;
    bool firstEnter = true;

    rclcpp::Service<tm_msgs::srv::ConnectTM>::SharedPtr connect_tm_srv_;

    rclcpp::Service<tm_msgs::srv::SendScript>::SharedPtr send_script_srv_;
    rclcpp::Service<tm_msgs::srv::SetEvent>::SharedPtr set_event_srv_;
    rclcpp::Service<tm_msgs::srv::SetIO>::SharedPtr set_io_srv_;
    rclcpp::Service<tm_msgs::srv::SetPositions>::SharedPtr set_positions_srv_;

    rclcpp::Service<tm_msgs::srv::AskSta>::SharedPtr ask_sta_srv_;

    rclcpp_action::Server<control_msgs::action::FollowJointTrajectory>::SharedPtr as_;
    std::mutex as_mtx_;
    std::string goal_id_;
    bool has_goal_;

public:
    explicit TmSctRos2(rclcpp::Node::SharedPtr node, TmDriver &iface, bool is_fake);
    ~TmSctRos2();

protected:
    void sct_msg();
    void check_is_on_listen_node_from_script(std::string id, std::string script);
    void sta_msg();
    bool sct_func();
    void check_is_on_listen_node();
    void sct_responsor();
    void sct_connect_recover();
    bool ask_sta_struct(std::string subcmd, std::string subdata, double waitTime,std::string &reSubcmd, std::string &reSubdata);
public:
    bool connect_tmsct(
        const std::shared_ptr<tm_msgs::srv::ConnectTM::Request> req,
        std::shared_ptr<tm_msgs::srv::ConnectTM::Response> res);

    bool send_script(
        const std::shared_ptr<tm_msgs::srv::SendScript::Request> req,
        std::shared_ptr<tm_msgs::srv::SendScript::Response> res);
    bool set_event(
        const std::shared_ptr<tm_msgs::srv::SetEvent::Request> req,
        std::shared_ptr<tm_msgs::srv::SetEvent::Response> res);
    bool set_io(
        const std::shared_ptr<tm_msgs::srv::SetIO::Request> req,
        std::shared_ptr<tm_msgs::srv::SetIO::Response> res);
    bool set_positions(
        const std::shared_ptr<tm_msgs::srv::SetPositions::Request> req,
        std::shared_ptr<tm_msgs::srv::SetPositions::Response> res);

    bool ask_sta(
        const std::shared_ptr<tm_msgs::srv::AskSta::Request> req,
        std::shared_ptr<tm_msgs::srv::AskSta::Response> res);

protected:
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Goal> goal
  );
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle
  );
  void handle_accepted(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle
  );
  void execute_traj(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle
  );

  void reorder_traj_joints(
    std::vector<trajectory_msgs::msg::JointTrajectoryPoint> &new_traj_points,
    const trajectory_msgs::msg::JointTrajectory &traj
  );
  bool has_points(const trajectory_msgs::msg::JointTrajectory &traj);
  bool is_traj_finite(const trajectory_msgs::msg::JointTrajectory &traj);
  bool is_positions_match(const trajectory_msgs::msg::JointTrajectoryPoint &point, double eps = 0.01);
  void set_pvt_traj(TmPvtTraj &pvts, const std::vector<trajectory_msgs::msg::JointTrajectoryPoint> &traj_points, double Tmin = 0.1);
  std::shared_ptr<TmPvtTraj> get_pvt_traj(
    const std::vector<trajectory_msgs::msg::JointTrajectoryPoint> &traj_points, double Tmin = 0.1
  );
};
