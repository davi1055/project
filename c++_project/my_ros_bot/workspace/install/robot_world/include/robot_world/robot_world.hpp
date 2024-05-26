/**
 * @file robot_world.hpp
 * @author davi (davicheng1114@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2024-05-26
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef ROBOT_WORLD_INCLUDE_ROBOT_WORLD_ROBOT_WORLD_HPP_
#define ROBOT_WORLD_INCLUDE_ROBOT_WORLD_ROBOT_WORLD_HPP_

#include "rclcpp/rclcpp.hpp"

class RobotWorld : public rclcpp::Node {
public:
	RobotWorld() : Node("robot_world") {
		RCLCPP_INFO(this->get_logger(), "Robot World Node has been started.");
	}
};

#endif //  ROBOT_WORLD_INCLUDE_ROBOT_WORLD_ROBOT_WORLD_HPP_