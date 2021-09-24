/*****************************************************************************
** Includes
*****************************************************************************/
#include "tm_ros_driver_windows.hpp"
#include <QApplication>

/*****************************************************************************
** Main
*****************************************************************************/
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  qRegisterMetaType<tm_msgs::msg::FeedbackState::SharedPtr>("tm_msgs::msg::FeedbackState::SharedPtr");
  qRegisterMetaType<std::string>("std::string");
  QApplication a(argc, argv);
  MainWindow w;
  w.show();
  a.connect(&a, SIGNAL(lastWindowClosed()), &a, SLOT(quit()));
  
  return a.exec();
}