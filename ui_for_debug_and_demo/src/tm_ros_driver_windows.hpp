#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QMessageBox>
#include <QFileDialog>
#include <QStringListModel>
#include <QButtonGroup>
#include <QThread>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "tm_msgs/msg/feedback_state.hpp"
#include "tm_msgs/srv/connect_tm.hpp"
#include "tm_msgs/srv/set_io.hpp"
QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class RosPage : public QThread {
 Q_OBJECT
 private:
  rclcpp::Node::SharedPtr node;
  rclcpp::Subscription<tm_msgs::msg::FeedbackState>::SharedPtr feedBackStatusSubscription;
  rclcpp::Client<tm_msgs::srv::ConnectTM>::SharedPtr connectSvrClient;
  rclcpp::Client<tm_msgs::srv::ConnectTM>::SharedPtr connectSctClient;
  bool lastStatus;
  void initial_subscriber();
  void initial_client();
  void feedback_states_callback(tm_msgs::msg::FeedbackState::SharedPtr msg);
  void send_service(rclcpp::Client<tm_msgs::srv::ConnectTM>::SharedPtr client, std::shared_ptr<tm_msgs::srv::ConnectTM::Request> request);
 signals:
  void send_ui_feed_back_status(tm_msgs::msg::FeedbackState::SharedPtr msg);
 private slots:
  void send_sct_as_re_connect();
  void send_svr_as_re_connect();
  void change_control_box_io_button();
 public:
  RosPage(std::string nodeName);
  ~RosPage();
  void run();
};

class MainWindow : public QMainWindow {
  Q_OBJECT
 private:
  Ui::MainWindow *ui;
  std::unique_ptr<RosPage> rosPage;
  void initial_ros_thread_to_ui_page();
  void initial_ui_page_to_ros_thread();
  void set_text_true_false(bool isTrue, QLabel* label);
  void initial_ui_compoment();
 signals:
  void send_sct_as_re_connect();
  void send_svr_as_re_connect();
  void change_control_box_io_button();
 private slots:
  void send_ui_feed_back_status(tm_msgs::msg::FeedbackState::SharedPtr msg);
  void click_set_sct_re_connect_button();
  void click_set_svr_re_connect_button();
  void click_change_control_box_io_button();
 public:
  MainWindow(QWidget *parent = nullptr);
  ~MainWindow();

};
#endif // MAINWINDOW_H