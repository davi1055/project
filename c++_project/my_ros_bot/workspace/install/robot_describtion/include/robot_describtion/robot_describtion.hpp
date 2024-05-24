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
#ifndef ROBOT_DESCRIBTION_INCLUDE_ROBOT_DESCRIBTION_ROBOT_DESCRIBTION_HPP_
#define ROBOT_DESCRIBTION_INCLUDE_ROBOT_DESCRIBTION_ROBOT_DESCRIBTION_HPP_

#include "rclcpp/rclcpp.hpp"

class RobotDescribtion : public rclcpp::Node {
public:
	RobotDescribtion() : Node("robot_describtion") {
		RCLCPP_INFO(this->get_logger(), "Robot Describtion Node has been started.");
	}
};

#endif //  ROBOT_DESCRIBTION_INCLUDE_ROBOT_DESCRIBTION_ROBOT_DESCRIBTION_HPP_