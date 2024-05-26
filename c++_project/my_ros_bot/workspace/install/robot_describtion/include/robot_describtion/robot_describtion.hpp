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
#include "std_msgs/msg/string.hpp"

class RobotDescribtion : public rclcpp::Node {
public:
	RobotDescribtion() : Node("robot_describtion") {
		// RCLCPP_INFO(this->get_logger(), "Robot Describtion Node has been started.");
		this->publisher_ = this->create_publisher<std_msgs::msg::String>("robot_describtion", 1);
		timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&RobotDescribtion::timer_callback, this)
    );
	}
private:
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
	rclcpp::TimerBase::SharedPtr timer_;
	void timer_callback();
};

void RobotDescribtion::timer_callback(){
	auto message = std_msgs::msg::String();
    message.data = "robot_describtion node";
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
}

#endif //  ROBOT_DESCRIBTION_INCLUDE_ROBOT_DESCRIBTION_ROBOT_DESCRIBTION_HPP_