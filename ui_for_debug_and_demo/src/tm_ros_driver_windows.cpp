#include "tm_ros_driver_windows.hpp"
#include "ui_tm_ros_driver_windows.h"

void MainWindow::initial_ros_thread_to_ui_page(){
  connect(rosPage.get(), SIGNAL(send_ui_feed_back_status(tm_msgs::msg::FeedbackState::SharedPtr)), this, SLOT(send_ui_feed_back_status(tm_msgs::msg::FeedbackState::SharedPtr)));
  connect(this, SIGNAL(change_control_box_io_button()),rosPage.get(),SLOT(change_control_box_io_button()));
}
void MainWindow::set_text_true_false(bool isTrue, QLabel* label){
  if(isTrue){
    label->setText("TRUE");
    label->setStyleSheet("QLabel { color : red; }");
  } else{
    label->setText("False");
    label->setStyleSheet("QLabel { color : black; }");
  }
}
void MainWindow::send_ui_feed_back_status(tm_msgs::msg::FeedbackState::SharedPtr msg){
  set_text_true_false(msg->is_svr_connected,ui->is_svr_connected_status_label);
  set_text_true_false(msg->is_sct_connected,ui->is_sct_connected_label_status_label);

  set_text_true_false(msg->robot_link,ui->robot_link_status_label);
  set_text_true_false(msg->robot_error,ui->robot_error_status_label);
  set_text_true_false(msg->project_run,ui->project_run_status_label);
  set_text_true_false(msg->project_pause,ui->project_pause_status_label);
  set_text_true_false(msg->safetyguard_a,ui->safetyguard_a_status_label);
  set_text_true_false(msg->e_stop,ui->e_stop_status_label);

  ui->error_code_status_label->setText(QString::number(msg->error_code));
  ui->error_content_status_label->setText(QString::fromStdString(msg->error_content));
  if(msg->cb_digital_output.size()>0){
    if(msg->cb_digital_output[0] == 0){
      set_text_true_false(false,ui->control_box_io1_status_label);
    } else{
      set_text_true_false(true,ui->control_box_io1_status_label);
    }
    
  }
  
}
void MainWindow::click_set_sct_re_connect_button(){
  send_sct_as_re_connect();
}
void MainWindow::click_set_svr_re_connect_button(){
  send_svr_as_re_connect();
}
void MainWindow::click_change_control_box_io_button(){
  std::cout<<"click_change_control_box_io_button"<<std::endl;
  change_control_box_io_button();
}
void MainWindow::initial_ui_compoment(){
  connect(ui->set_sct_re_connect_button, SIGNAL(clicked()),this,SLOT(click_set_sct_re_connect_button()));
  connect(ui->set_svr_re_connect_button, SIGNAL(clicked()),this,SLOT(click_set_svr_re_connect_button()));
  connect(ui->change_control_box_io_button, SIGNAL(clicked()),this,SLOT(click_change_control_box_io_button()));
}
void MainWindow::initial_ui_page_to_ros_thread(){
  connect(this, SIGNAL(send_sct_as_re_connect()),rosPage.get(),SLOT(send_sct_as_re_connect()));
  connect(this, SIGNAL(send_svr_as_re_connect()),rosPage.get(),SLOT(send_svr_as_re_connect()));
}
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
  ui->setupUi(this);
  initial_ui_compoment();
  rosPage = std::make_unique<RosPage>("simulator_run_control_node");
  rosPage->start();
  initial_ros_thread_to_ui_page();
  initial_ui_page_to_ros_thread();
}
MainWindow::~MainWindow(){
  
}