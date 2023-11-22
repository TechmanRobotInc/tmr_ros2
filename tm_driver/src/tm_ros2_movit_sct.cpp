#include "tm_driver/tm_ros2_movit_sct.h"

auto sec = [](const builtin_interfaces::msg::Duration& t) {
    return static_cast<double>(t.sec) + 1e-9 * static_cast<double>(t.nanosec);
};

void TmRos2SctMoveit::intial_action(){
    as_ = rclcpp_action::create_server<control_msgs::action::FollowJointTrajectory>(
     node->get_node_base_interface(),
     node->get_node_clock_interface(),
     node->get_node_logging_interface(),
     node->get_node_waitables_interface(),
     "tmr_arm_controller/follow_joint_trajectory",
     std::bind(&TmRos2SctMoveit::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
     std::bind(&TmRos2SctMoveit::handle_cancel, this, std::placeholders::_1),
     std::bind(&TmRos2SctMoveit::handle_accepted, this, std::placeholders::_1)
    );
    goal_id_.clear();
    has_goal_ = false;
}

rclcpp_action::GoalResponse TmRos2SctMoveit::handle_goal(const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Goal> goal)
{
  auto goal_id = rclcpp_action::to_string(uuid);
  print_info("Received new action goal %s", goal_id.c_str());

  if (has_goal_) {
    return rclcpp_action::GoalResponse::ACCEPT_AND_DEFER;
  }

  if (!is_fake_) {
    if (!svr_.is_connected()) {
      print_error("Goal: %s, got rejected because of SVR not being connected", goal_id.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (!sct_.is_connected()) {
      print_error("Goal: %s, got rejected because of SCT not being connected", goal_id.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (state_.has_error()) {
      print_error("Goal: %s, got rejected because STATE has an error", goal_id.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
  }

  if (!has_points(goal->trajectory)) {
    std::string msg = control_msgs::action::to_yaml(*goal);
    const std::string delimiter = "\n";
    size_t pos = 0;
    std::string token;
    while ((pos = msg.find(delimiter)) != std::string::npos) {
      token = msg.substr(0, pos);
      print_error(token.c_str());
      msg.erase(0, pos + delimiter.length());
    }

    print_error("Goal: %s, got rejected because of no trajectory points", goal_id.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }

  print_info("Goal: %s, got accepted", goal_id.c_str());
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}
rclcpp_action::CancelResponse TmRos2SctMoveit::handle_cancel(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle)
{
  auto goal_id = rclcpp_action::to_string(goal_handle->get_goal_id());
  print_info("Got request to cancel goal %s", goal_id.c_str());
  {
    std::lock_guard<std::mutex> lck(as_mtx_);
    if (goal_id_.compare(goal_id) == 0 && has_goal_) {
      has_goal_ = false;
      iface_.stop_pvt_traj();
    }
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}
void
TmRos2SctMoveit::handle_accepted(std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle)
{
  {
    std::unique_lock<std::mutex> lck(as_mtx_);
    goal_id_ = rclcpp_action::to_string(goal_handle->get_goal_id());
    has_goal_ = true;
  }
  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread{std::bind(&TmRos2SctMoveit::execute_traj, this, std::placeholders::_1), goal_handle}.detach();
}

void
TmRos2SctMoveit::execute_traj(const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle)
{
  print_info("TM_ROS: trajectory thread begin");

  auto result = std::make_shared<control_msgs::action::FollowJointTrajectory::Result>();

  //actually, no need to reorder
  //std::vector<trajectory_msgs::msg::JointTrajectoryPoint> traj_points;
  //reorder_traj_joints(traj_points, goal_handle->get_goal()->trajectory);
  auto &traj_points = goal_handle->get_goal()->trajectory.points;

  if (!is_positions_match(traj_points.front(), 0.01)) {
    result->error_code = result->PATH_TOLERANCE_VIOLATED;
    result->error_string = "Start point doesn't match current pose";
    print_warn(result->error_string.c_str());
    goal_handle->abort(result);
  }

  auto pvts = get_pvt_traj(traj_points, 0.025);
  print_info("TM_ROS: traj. total time:= %d", static_cast<int>(pvts->total_time));

  if (!goal_handle->is_executing()) {
    print_info("goal_handle->execute()");
    goal_handle->execute();
  }

  if (!is_fake_) {
    iface_.run_pvt_traj(*pvts, ((100.0 / state_.project_speed()) - 0.95) * pvts->total_time);  // Total time running is X.05 * total_time, with X 100/project_speed
  }
  else {
    iface_.fake_run_pvt_traj(*pvts);
  }
  if (rclcpp::ok()) {
    if (!is_positions_match(traj_points.back(), 0.01)) {
      result->error_code = result->GOAL_TOLERANCE_VIOLATED;
      result->error_string = "Current pose doesn't match Goal point";
      print_warn(result->error_string.c_str());
      goal_handle->abort(result);
    }
    else {
      result->error_code = result->SUCCESSFUL;
      result->error_string = "Goal reached, success!";
      print_info(result->error_string.c_str());
      goal_handle->succeed(result);
    }
  }
  {
    std::lock_guard<std::mutex> lck(as_mtx_);
    goal_id_.clear();
    has_goal_ = false;
  }
  print_info("TM_ROS: trajectory thread end");
}

void
TmRos2SctMoveit::reorder_traj_joints(
  std::vector<trajectory_msgs::msg::JointTrajectoryPoint> &new_traj_points,
  const trajectory_msgs::msg::JointTrajectory& traj)
{
  /* Reorders trajectory - destructive */
  std::vector<size_t> mapping;
  mapping.resize(jns_.size(), jns_.size());
  for (size_t i = 0; i < traj.joint_names.size(); ++i) {
    for (size_t j = 0; j < jns_.size(); ++j) {
      if (traj.joint_names[i] == jns_[j])
        mapping[j] = i;
    }
  }
  new_traj_points.clear();
  for (unsigned int i = 0; i < traj.points.size(); ++i) {
    trajectory_msgs::msg::JointTrajectoryPoint new_point;
    for (unsigned int j = 0; j < traj.points[i].positions.size(); ++j) {
      new_point.positions.push_back(traj.points[i].positions[mapping[j]]);
      new_point.velocities.push_back(traj.points[i].velocities[mapping[j]]);
      if (traj.points[i].accelerations.size() != 0)
        new_point.accelerations.push_back(traj.points[i].accelerations[mapping[j]]);
    }
    new_point.time_from_start = traj.points[i].time_from_start;
    new_traj_points.push_back(new_point);
  }
}
bool TmRos2SctMoveit::has_points(const trajectory_msgs::msg::JointTrajectory &traj)
{
  if (traj.points.size() == 0) return false;
  for (auto &point : traj.points) {
    if (point.positions.size() != traj.joint_names.size() ||
      point.velocities.size() != traj.joint_names.size()) return false;
  }
  return true;
}
bool TmRos2SctMoveit::is_traj_finite(const trajectory_msgs::msg::JointTrajectory &traj)
{
  for (auto &point : traj.points) {
    for (auto &p : point.positions) {
      if (!std::isfinite(p)) return false;
    }
    for (auto &v : point.velocities) {
      if (!std::isfinite(v)) return false;
    }
  }
  return true;
}
bool TmRos2SctMoveit::is_positions_match(
  const trajectory_msgs::msg::JointTrajectoryPoint &point, double eps)
{
  auto q_act = state_.joint_angle();
  for (size_t i = 0; i < point.positions.size(); ++i) {
    if (fabs(point.positions[i] - q_act[i]) > eps)
      return false;
  }
  return true;
}
void
TmRos2SctMoveit::set_pvt_traj(TmPvtTraj &pvts, const std::vector<trajectory_msgs::msg::JointTrajectoryPoint> &traj_points, double Tmin)
{
  print_debug("TM_ROS: Traj size: %d", static_cast<int>(traj_points.size()));
  if (traj_points.size() < 2)
  {
    print_warn("TM_ROS: Traj rejected because size<2: %d", static_cast<int>(traj_points.size()));
    return;
  }

  double total_time = sec(traj_points[traj_points.size()-1].time_from_start);
  if (total_time < Tmin)
  {
    print_warn("TM_ROS: Traj rejected because total time<Tmin(%f): %f", Tmin, total_time);
    return;
  }

  std::stringstream ss;
  ss << "msg:\n";
  for (const auto& point : traj_points)
  {
    ss << trajectory_msgs::msg::to_yaml(point) << "\n";
  }
  RCLCPP_DEBUG_STREAM(node->get_logger(), ss.str());

  pvts.mode = TmPvtMode::Joint;

  size_t i = 0, i_1 = 0, i_2 = 0;
  int skip_count = 0;
  TmPvtPoint point;

  // first point
  if (sec(traj_points[i].time_from_start) >= 1e-6) {
    print_warn("TM_ROS: Traj.: first point should be the current position, with time_from_start set to 0.0");
  }
  point.time = sec(traj_points[i].time_from_start);
  point.positions = traj_points[i].positions;
  point.velocities = traj_points[i].velocities;
  pvts.points.push_back(point);

  for (i = 1; i < traj_points.size() - 1; ++i) {
    point.time = sec(traj_points[i].time_from_start) - sec(traj_points[i_1].time_from_start);
    if (point.time >= Tmin) {
      i_2 = i_1;
      i_1 = i;
      point.positions = traj_points[i].positions;
      point.velocities = traj_points[i].velocities;
      pvts.points.push_back(point);
    }
    else {
      print_debug("Skipping index: %d", i);
      ++skip_count;
    }
  }
  if (skip_count > 0) {
    print_warn("TM_ROS: Traj.: skip %d points", (int)skip_count);	
  }
  // last point
  if (traj_points.size() > 1) {
    i =  traj_points.size() - 1;
    point.time = sec(traj_points[i].time_from_start) - sec(traj_points[i_1].time_from_start);
    point.positions = traj_points[i].positions;
    point.velocities = traj_points[i].velocities;
    if (point.time >= Tmin) {
      pvts.points.push_back(point);
    }
    else {
      point.time = sec(traj_points[i].time_from_start) - sec(traj_points[i_2].time_from_start);
      pvts.points.back() = point; // This is safe because the starting point is always added.
      ++skip_count;
      print_warn("TM_ROS: Traj.: skip 1 more last point");
    }
  }
  pvts.total_time = sec(traj_points.back().time_from_start);
}

std::shared_ptr<TmPvtTraj>
TmRos2SctMoveit::get_pvt_traj(const std::vector<trajectory_msgs::msg::JointTrajectoryPoint> &traj_points, double Tmin)
{
  std::shared_ptr<TmPvtTraj> pvts = std::make_shared<TmPvtTraj>();
  set_pvt_traj(*pvts, traj_points, Tmin);
  return pvts;
}
