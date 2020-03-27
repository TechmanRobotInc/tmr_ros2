#include <cstdio>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/image.hpp"

int encoding2mat_type(const std::string & encoding)
  {
    if (encoding == "mono8") {
      return CV_8UC1;
    } else if (encoding == "bgr8") {
      return CV_8UC3;
    } else if (encoding == "mono16") {
      return CV_16SC1;
    } else if (encoding == "rgba8") {
      return CV_8UC4;
    } else if (encoding == "bgra8") {
      return CV_8UC4;
    } else if (encoding == "32FC1") {
      return CV_32FC1;
    } else if (encoding == "rgb8") {
      return CV_8UC3;
    }else if (encoding =="8UC3"){
      return CV_8UC3;
    }
    
    else {
      std::cout<<"the unknow image type is "<<encoding<<std::endl;
      throw std::runtime_error("Unsupported encoding type");
    }
  }

int main(int argc, char** argv){
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("test_image_sub");

  auto subscription = node->create_subscription<sensor_msgs::msg::Image>("techman_image",10,
    [&](const sensor_msgs::msg::Image::SharedPtr msg) ->void {

  std::cerr << "Received image #" << msg->header.frame_id.c_str() << std::endl;

 
  // Convert to an OpenCV matrix by assigning the data.
  cv::Mat frame(
  msg->height, msg->width, encoding2mat_type(msg->encoding),
  const_cast<unsigned char *>(msg->data.data()), msg->step);

  if (msg->encoding == "rgb8") {
    cv::cvtColor(frame, frame, cv::COLOR_RGB2BGR);
  }
  std::cout << "Width : " << frame.size().width << std::endl;
  std::cout << "Height: " << frame.size().height << std::endl;

  cv::Mat cvframe = frame;

  // Show the image in a window called "showimage".
  cv::imshow("showimage",cvframe );
  // Draw the screen and wait for 1 millisecond.
  cv::waitKey(1);
  
});
	
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
