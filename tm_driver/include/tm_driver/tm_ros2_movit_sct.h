#include "tm_ros2_sct.h"
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

class TmRos2SctMoveit : public TmSctRos2{
  private:
    rclcpp_action::Server<control_msgs::action::FollowJointTrajectory>::SharedPtr as_;
    std::mutex as_mtx_;
    std::string goal_id_;
    bool has_goal_;
    TmSctCommunication &sct_;
    TmSvrCommunication &svr_;
    TmRobotState &state_;

    void intial_action();
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle);
    void handle_accepted(
      std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle);
    void execute_traj(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle);
    void reorder_traj_joints(
      std::vector<trajectory_msgs::msg::JointTrajectoryPoint> &new_traj_points,
      const trajectory_msgs::msg::JointTrajectory& traj);
    bool has_points(const trajectory_msgs::msg::JointTrajectory &traj);
    bool is_traj_finite(const trajectory_msgs::msg::JointTrajectory &traj);
    bool is_positions_match(
      const trajectory_msgs::msg::JointTrajectoryPoint &point, double eps);
    void set_pvt_traj(
      TmPvtTraj &pvts, const std::vector<trajectory_msgs::msg::JointTrajectoryPoint> &traj_points, double Tmin);
      std::shared_ptr<TmPvtTraj> get_pvt_traj(
    const std::vector<trajectory_msgs::msg::JointTrajectoryPoint> &traj_points, double Tmin = 0.1);
  public:
    TmRos2SctMoveit(rclcpp::Node::SharedPtr node, TmDriver &iface, bool is_fake) 
     : TmSctRos2(node, iface, is_fake)
     , sct_(iface.sct)
     , svr_(iface.svr)
     , state_(iface.state){
        intial_action();
    }
};