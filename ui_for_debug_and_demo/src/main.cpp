#include "tm_ros_driver_windows.hpp"

#include <QApplication>
#include "rclcpp/rclcpp.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  qRegisterMetaType<tm_msgs::msg::FeedbackState::SharedPtr>("tm_msgs::msg::FeedbackState::SharedPtr");
  QApplication a(argc, argv);
  MainWindow w;
  w.show();
  return a.exec();
}