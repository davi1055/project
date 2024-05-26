/**
 * @file robot_describtion.cpp
 * @author davi (davicheng1114@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2024-05-24
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "robot_describtion/robot_describtion.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotDescribtion>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}