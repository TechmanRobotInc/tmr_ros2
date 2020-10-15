#include "tmr/tmr_driver.h"
#include "tmr/tmr_print.h"

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "tmr_msgs/msg/feedback_state.hpp"
#include "tmr_msgs/msg/svr_response.hpp"

#include "tmr_msgs/srv/connect_tm.hpp"
#include "tmr_msgs/srv/write_item.hpp"
#include "tmr_msgs/srv/ask_item.hpp"


class TmSvrRos2 : public rclcpp::Node
{
public:
  tmr::TmSvrCommunication &svr_;
  tmr::TmSctCommunication &sct_;
  tmr::Driver &iface_;
  tmr::RobotState &state_;

protected:
  const bool is_fake_;

public:
  std::vector<std::string> jns_;

  struct PubMsg {
    rclcpp::Publisher<tmr_msgs::msg::FeedbackState>::SharedPtr fbs_pub;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr tool_pose_pub;
    rclcpp::Publisher<tmr_msgs::msg::SvrResponse>::SharedPtr svr_pub;

    tmr_msgs::msg::FeedbackState fbs_msg;
    sensor_msgs::msg::JointState joint_msg;
    geometry_msgs::msg::PoseStamped tool_pose_msg;
    tmr_msgs::msg::SvrResponse svr_msg;
  } pm_;

  bool svr_updated_;
  std::mutex svr_mtx_;
  std::condition_variable svr_cv_;

  int pub_reconnect_timeout_ms_;
  int pub_reconnect_timeval_ms_;
  std::thread pub_thread_;

  rclcpp::Service<tmr_msgs::srv::ConnectTM>::SharedPtr connect_tm_srv_;

  rclcpp::Service<tmr_msgs::srv::WriteItem>::SharedPtr write_item_srv_;
  rclcpp::Service<tmr_msgs::srv::AskItem>::SharedPtr ask_item_srv_;

public:
  explicit TmSvrRos2(const rclcpp::NodeOptions &options, tmr::Driver &iface, bool is_fake, bool stick_play = false);
  ~TmSvrRos2();

protected:
  void fake_publisher();

  void publish_fbs();
  void publish_svr();
  bool publish_func();
  void publisher();

public:
  bool connect_tmsvr(
    const std::shared_ptr<tmr_msgs::srv::ConnectTM::Request> req,
    std::shared_ptr<tmr_msgs::srv::ConnectTM::Response> res);

  bool write_item(
    const std::shared_ptr<tmr_msgs::srv::WriteItem::Request> req,
    std::shared_ptr<tmr_msgs::srv::WriteItem::Response> res);
  bool ask_item(
    const std::shared_ptr<tmr_msgs::srv::AskItem::Request> req,
    std::shared_ptr<tmr_msgs::srv::AskItem::Response> res);

};
