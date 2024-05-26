/**
 * @file robot_world.cpp
 * @author davi (davicheng1114@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2024-05-26
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "robot_world/robot_world.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotWorld>();
  rclcpp::spin(node);
  return 0;
}