/*****************************************************************************
** Includes
*****************************************************************************/
#include "tm_ros_driver_windows.hpp"
#include "ui_tm_ros_driver_windows.h"

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/
void MainWindow::initial_ui_component(){
  connect(ui->set_sct_re_connect_button, SIGNAL(clicked()),this,SLOT(click_set_sct_re_connect_button()));
  connect(ui->set_svr_re_connect_button, SIGNAL(clicked()),this,SLOT(click_set_svr_re_connect_button()));
  connect(ui->change_control_box_io_button, SIGNAL(clicked()),this,SLOT(click_change_control_box_io_button()));
  connect(ui->clear_response_button, SIGNAL(clicked()),this,SLOT(click_clear_response_button()));
  connect(ui->Close, SIGNAL(clicked()),this,SLOT(quit()));
  
  statusItemModel = std::make_unique<QStandardItemModel>();
  ui->robot_response_listView->setModel(statusItemModel.get());
  ui->robot_response_listView->setSelectionMode(QAbstractItemView::ExtendedSelection);
  ui->robot_response_listView->setEditTriggers(QAbstractItemView::NoEditTriggers);
}
void MainWindow::initial_ros_thread_to_ui_page(){
  connect(rosPage.get(), SIGNAL(send_ui_feed_back_status(tm_msgs::msg::FeedbackState::SharedPtr)), this, SLOT(send_ui_feed_back_status(tm_msgs::msg::FeedbackState::SharedPtr)));
  connect(rosPage.get(), SIGNAL(send_to_ui_list(std::string)), this, SLOT(send_to_ui_list(std::string)));
}
void MainWindow::set_text_nan_initial(QLabel* label){
  //label->setAlignment(Qt::AlignCenter);
  QFont f("Arial",14);
  f.setWeight(600);  
  label->setFont(f);
  label->setText("NaN");
  label->setStyleSheet("background-color:rgb(81, 86, 90)");
  label->setStyleSheet("QLabel { color:rgb(143,122,102);}");
}
void MainWindow::set_text_true_false(bool isTrue, QLabel* label, bool isReverse){
  if(!isReverse)
  {
    if(isTrue){
      label->setText("<html><head/><body><p><span style=\" font-size:14pt; font-weight:600; color:#73d216;\">True</span></p></body></html>");
    } else{
      label->setText("<html><head/><body><p><span style=\" font-size:14pt; font-weight:600; color:#cc0000;\">False</span></p></body></html>");
    }
  }
  else
  {
    if(isTrue){
      label->setText("<html><head/><body><p><span style=\" font-size:14pt; font-weight:600; color:#cc0000;\">True</span></p></body></html>");
    } else{
      label->setText("<html><head/><body><p><span style=\" font-size:14pt; font-weight:600; color:#73d216;\">False</span></p></body></html>");
    }
  }
}
void MainWindow::set_text_on_off(bool isTrue, QLabel* label, bool isReverse){
  if(!isReverse)
  {
    if(isTrue){
      label->setText("<html><head/><body><p><span style=\"font-size:14pt; font-weight:600; color:#73d216;\">On</span></p></body></html>");
    } else{
      label->setText("<html><head/><body><p><span style=\"font-size:14pt; font-weight:600; color:#fcaf3e;\">Off</span></p></body></html>");
    }
  }
  else
  {
    if(isTrue){
      label->setText("<html><head/><body><p><span style=\"font-size:14pt; font-weight:600; color:#cc0000;\">On</span></p></body></html>");
    } else{
      label->setText("<html><head/><body><p><span style=\"font-size:14pt; font-weight:600; color:#73d216;\">Off</span></p></body></html>");
    }
  }
}
void MainWindow::set_text_high_low(bool isTrue, QLabel* label, bool isReverse){
  if(!isReverse)
  {
    if(isTrue){
      label->setText("<html><head/><body><p><span style=\" font-size:14pt; font-weight:600; color:#73d216;\">&nbsp;&nbsp;L</span></p></body></html>");
    } else{
      label->setText("<html><head/><body><p><span style=\" font-size:14pt; font-weight:600; color:#73d216;\">&nbsp;&nbsp;H</span></p></body></html>");
    }
  }
  else
  {
    if(isTrue){
      label->setText("<html><head/><body><p><span style=\" font-size:14pt; font-weight:600; color:#73d216;\">&nbsp;&nbsp;H</span></p></body></html>");
    } else{
      label->setText("<html><head/><body><p><span style=\" font-size:14pt; font-weight:600; color:#73d216;\">&nbsp;&nbsp;L</span></p></body></html>");
    }
  }
}
void MainWindow::set_text_null_reserve(bool isTrue, QLabel* label){
  if(isTrue){
    label->setText("<html><head/><body><p><span style=\" font-size:14pt; font-weight:600; color:#73d216;\">Null</span></p></body></html>");
  } else{
    label->setText("<html><head/><body><p><span style=\" font-size:14pt; font-weight:600; color:#8F7A66;\">RSV</span></p></body></html>");
  }
}
void MainWindow::set_text_manual_auto(int isValue, QLabel* label){
  if(isValue == 2){   // 1:AUTO  2:MANUAL  0: None
    label->setText("<html><head/><body><p><span style=\" font-size:14pt; font-weight:600; color:#73d216;\">MAN</span></p></body></html>");
  } else if (isValue == 1) {
    label->setText("<html><head/><body><p><span style=\" font-size:14pt; font-weight:600; color:#2a52be;\">Auto</span></p></body></html>");
  } else{
    label->setText("<html><head/><body><p><span style=\" font-size:14pt; font-weight:600; color:#73d216;\">None</span></p></body></html>");  
  }    
}
void MainWindow::set_text_zero_false(int isZero, QLabel* label){
  if(isZero == 0){
    label->setText("<html><head/><body><p><span style=\" font-size:14pt; font-weight:600; color:#73d216;\">0</span></p></body></html>");
  } else{
    label->setText("<html><head/><body><p><span style=\" font-size:14pt; font-weight:600; color:#cc0000;\">False</span></p></body></html>");
  }
}
void MainWindow::int_base_format_change(quint32 msg, QLabel* label, int base){
  QFont f("Arial",14);
  f.setWeight(600);  
  label->setFont(f);
  label->setText(QString::number(msg,base));
  label->setStyleSheet("background-color:rgb(81, 86, 90)");
  label->setStyleSheet("QLabel { color:rgb(143,122,102);}");
}
void MainWindow::initial_link_ctrl_label(){
  set_text_nan_initial(ui->svrclient_status_label);
  set_text_nan_initial(ui->sctclient_status_label); 
}
void MainWindow::initial_status_ctrl_label(){
  set_text_nan_initial(ui->safeguard_a_status_label);
  set_text_nan_initial(ui->robot_link_status_label);
  set_text_nan_initial(ui->robot_error_status_label);
  set_text_nan_initial(ui->project_run_status_label);
  set_text_nan_initial(ui->project_pause_status_label);
  set_text_nan_initial(ui->data_table_correct_status_label);
  set_text_nan_initial(ui->estop_status_label);
  set_text_nan_initial(ui->ma_mode_label);
  set_text_nan_initial(ui->error_code_status_label);
  set_text_nan_initial(ui->ctrl_do0_status_label);
  set_text_nan_initial(ui->tmsrv_cperr_status_label);
  set_text_nan_initial(ui->tmscript_cperr_status_label);
  set_text_nan_initial(ui->tmsrv_dataerr_status_label);
  set_text_nan_initial(ui->tmscript_dataerr_status_label);
}
QString MainWindow::format_change(std::string msg){
  std::string str;
  str = "<html><head/><body><p><span style=\" font-size:14pt; font-weight:600; color:#73d216;\">";
  str += msg;
  str += "</span></p></body></html>";
  return QString::fromStdString(str);
}
void MainWindow::send_ui_feed_back_status(tm_msgs::msg::FeedbackState::SharedPtr msg){
  set_text_on_off(msg->is_svr_connected,ui->svrclient_status_label,false);
  set_text_on_off(msg->is_sct_connected,ui->sctclient_status_label,false);
  if (msg->is_svr_connected)
  {
    set_text_true_false(msg->robot_link,ui->robot_link_status_label,false);
    set_text_true_false(msg->robot_error,ui->robot_error_status_label,true);
    set_text_on_off(msg->project_run,ui->project_run_status_label,false);  
    set_text_on_off(msg->project_pause,ui->project_pause_status_label,true);
    set_text_on_off(msg->safetyguard_a,ui->safeguard_a_status_label,true);
    set_text_on_off(msg->e_stop,ui->estop_status_label,true);
    set_text_true_false(msg->is_data_table_correct,ui->data_table_correct_status_label,false); 
    set_text_true_false(msg->tmsrv_cperr,ui->tmsrv_cperr_status_label,true);
    set_text_true_false(msg->tmscript_cperr,ui->tmscript_cperr_status_label,true);
    set_text_true_false(msg->tmsrv_dataerr,ui->tmsrv_dataerr_status_label,true);
    set_text_true_false(msg->tmscript_dataerr,ui->tmscript_dataerr_status_label,true);
    set_text_manual_auto(msg->ma_mode,ui->ma_mode_label);
    if(msg->error_code == 0){
        set_text_true_false(msg->error_code,ui->error_code_status_label,true);    
        //set_text_null_reserve(true,ui->error_content_status_label);
    } else{ 
        int_base_format_change(msg->error_code,ui->error_code_status_label,16);
        //set_text_null_reserve(false,ui->error_content_status_label);
    }
    if(msg->cb_digital_output.size() > 0){
      if(msg->cb_digital_output[0] == 0){
        set_text_high_low(false,ui->ctrl_do0_status_label,true);
      } else{
        set_text_high_low(true,ui->ctrl_do0_status_label,true);
      }
    }
  }
  else
  {
    initial_status_ctrl_label();
  }
  if(msg->disconnection_times == 0){
    set_text_zero_false(msg->disconnection_times,ui->linklost_status_label);    
    set_text_zero_false(msg->max_not_connect_in_s,ui->maxlosttime_status_label);  
  } else{ 
    int_base_format_change(msg->disconnection_times,ui->linklost_status_label,10);
    int_base_format_change(msg->max_not_connect_in_s,ui->maxlosttime_status_label,10);
  }
}
void MainWindow::click_set_sct_re_connect_button(){
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("rclcpp"), "click [SctClient Re-Connect] button");  
  initial_link_ctrl_label();
  initial_status_ctrl_label();
  send_sct_as_re_connect();
}
void MainWindow::click_set_svr_re_connect_button(){
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("rclcpp"), "click [SvrClient Re-Connect] button");	
  initial_link_ctrl_label();
  initial_status_ctrl_label();
  send_svr_as_re_connect();
}
void MainWindow::click_change_control_box_io_button(){
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("rclcpp"), "click [DO0 Ctrl] button");
  change_control_box_io_button();
}
void MainWindow::quit()
{
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("rclcpp"), "click [Quit_GUI] button");    
  QApplication::quit();
}
void MainWindow::initial_ui_page_to_ros_thread(){
  connect(this, SIGNAL(send_sct_as_re_connect()),rosPage.get(),SLOT(send_sct_as_re_connect()));
  connect(this, SIGNAL(send_svr_as_re_connect()),rosPage.get(),SLOT(send_svr_as_re_connect()));
  connect(this, SIGNAL(change_control_box_io_button()),rosPage.get(),SLOT(change_control_box_io_button()));
}
void MainWindow::send_to_ui_list(std::string showMessage){
  QList<QStandardItem *> itemList;
  itemList<< new QStandardItem(QString::fromStdString(showMessage));
  statusItemModel->appendRow(itemList);
  QModelIndex vIndex = statusItemModel->index(rowIndex, 0);
  rowIndex++;
  QMap<int, QVariant> vMap = statusItemModel->itemData(vIndex);
  statusItemModel->setItemData(vIndex, vMap);
}
void MainWindow::click_clear_response_button(){
  statusItemModel.reset();
  statusItemModel = std::make_unique<QStandardItemModel>();
  rowIndex = 0;
  ui->robot_response_listView->setModel(statusItemModel.get());
}
MainWindow::MainWindow(QWidget *parent)
    : QDialog(parent)
    , ui(new Ui::MainWindow)
{
  ui->setupUi(this);
  initial_ui_component();
  rosPage = std::make_unique<RosPage>("simulator_run_control_node");
  rosPage->start();
  initial_ros_thread_to_ui_page();
  initial_ui_page_to_ros_thread();
  ui->log_tabWidget->removeTab(0);
}
MainWindow::~MainWindow(){
  delete ui;
}
