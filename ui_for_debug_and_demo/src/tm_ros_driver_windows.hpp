/*****************************************************************************
** Ifdefs
*****************************************************************************/
#ifndef MAINWINDOW_H
#define MAINWINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include <string>

#include <QMainWindow>
#include <QMessageBox>
#include <QStringListModel>
#include <QThread>
#include <QStandardItemModel>

#include "tm_msgs/msg/feedback_state.hpp"
#include "tm_msgs/msg/sct_response.hpp"
#include "tm_msgs/msg/svr_response.hpp"
#include "tm_msgs/msg/sta_response.hpp"
#include "tm_msgs/srv/connect_tm.hpp"
#include "tm_msgs/srv/set_io.hpp"
#include <ctime>

using std::chrono::system_clock;
/*****************************************************************************
** Namespaces
*****************************************************************************/
QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

/*****************************************************************************
** Class
*****************************************************************************/
class RosPage : public QThread {
 Q_OBJECT
 
public:
  RosPage(std::string nodeName);
  ~RosPage();
  void run();

private:
  rclcpp::Node::SharedPtr node;
  rclcpp::Subscription<tm_msgs::msg::FeedbackState>::SharedPtr feedBackStatusSubscription;
  rclcpp::Subscription<tm_msgs::msg::SctResponse>::SharedPtr sctResponseSubscription;
  rclcpp::Subscription<tm_msgs::msg::StaResponse>::SharedPtr staResponseSubscription;
  rclcpp::Subscription<tm_msgs::msg::SvrResponse>::SharedPtr svrResponseSubscription;
  rclcpp::Client<tm_msgs::srv::ConnectTM>::SharedPtr connectSvrClient;
  rclcpp::Client<tm_msgs::srv::ConnectTM>::SharedPtr connectSctClient;
  void feedback_states_callback(tm_msgs::msg::FeedbackState::SharedPtr msg);
  void sct_response_callback(tm_msgs::msg::SctResponse::SharedPtr msg);
  void sta_response_callback(tm_msgs::msg::StaResponse::SharedPtr msg);
  void svr_response_callback(tm_msgs::msg::SvrResponse::SharedPtr msg);
  void send_service(rclcpp::Client<tm_msgs::srv::ConnectTM>::SharedPtr client, std::shared_ptr<tm_msgs::srv::ConnectTM::Request> request);
  bool lastStatus;
  void initial_subscriber();
  void initial_client();
  std::string current_time();
  
signals:
  void send_ui_feed_back_status(tm_msgs::msg::FeedbackState::SharedPtr msg);
  void send_to_ui_list(std::string);
  
private slots:
  void send_sct_as_re_connect();
  void send_svr_as_re_connect();
  void change_control_box_io_button();
  
};

class MainWindow : public QDialog {
  Q_OBJECT

public:
  MainWindow(QWidget *parent = nullptr);
  ~MainWindow();

private:
  Ui::MainWindow *ui;
  std::unique_ptr<RosPage> rosPage;
  std::unique_ptr<QStandardItemModel> statusItemModel;
  int rowIndex;
  void initial_ros_thread_to_ui_page();
  void initial_ui_page_to_ros_thread();
  void set_text_nan_initial(QLabel* label);
  void set_text_true_false(bool isTrue, QLabel* label, bool isReverse);
  void set_text_on_off(bool isTrue, QLabel* label, bool isReverse);
  void set_text_high_low(bool isTrue, QLabel* label, bool isReverse);
  void set_text_null_reserve(bool isTrue, QLabel* label);
  void set_text_manual_auto(int isValue, QLabel* label);
  void set_text_zero_false(int isZero, QLabel* label);
  void int_base_format_change(quint32 msg, QLabel* label, int base); 
  void initial_link_ctrl_label();    
  void initial_status_ctrl_label();      
  void initial_ui_component();
  QString format_change(std::string msg);
  
signals:
  void send_sct_as_re_connect();
  void send_svr_as_re_connect();
  void change_control_box_io_button();

private slots:
  void send_ui_feed_back_status(tm_msgs::msg::FeedbackState::SharedPtr msg);
  void send_to_ui_list(std::string);
  void click_set_sct_re_connect_button();
  void click_set_svr_re_connect_button();
  void click_change_control_box_io_button();
  void click_clear_response_button();
  void quit();  

};
#endif // MAINWINDOW_H
