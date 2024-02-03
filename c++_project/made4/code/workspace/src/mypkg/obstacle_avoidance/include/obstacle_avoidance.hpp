// Copyright[2024] <davi>
#ifndef _ROOT_WORKSPACE_SRC_MYPKG_OBSTACLE_AVOIDANCE_INCLUDE_OBSTACLE_AVOIDANCE_HPP_
#define _ROOT_WORKSPACE_SRC_MYPKG_OBSTACLE_AVOIDANCE_INCLUDE_OBSTACLE_AVOIDANCE_HPP_
#include <iostream>
#include <cmath>
#include <string>
#include <std_msgs/msg/string.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>

class Avoidancer : public rclcpp::Node {
 public:
    explicit Avoidancer(const std::string& name) : Node(name) {
        // 避障器接收雷达信息
        auto avoidancer_sub_callback = std::bind(
            &Avoidancer::avoidancerSubCallback, this, std::placeholders::_1);

        // 创建订阅者
        avoidancer_sub_ptr_ =
            this->create_subscription<sensor_msgs::msg::LaserScan>(
                "scan", 1, avoidancer_sub_callback);

        // 创建发布者
        avoidancer_pub_ptr_ =
            this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);

        // 定时器定时发布消息
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&Avoidancer::avoidancerPubCallback, this));
    }

    ~Avoidancer(){
    this->cmd_vel_.linear.x = 0;
    this->cmd_vel_.linear.y = 0;
    this->cmd_vel_.linear.z = 0;
    this->cmd_vel_.angular.x = 0;
    this->cmd_vel_.angular.y = 0;
    this->cmd_vel_.angular.z = 0;
    this->avoidancer_pub_ptr_->publish(this->cmd_vel_);
}

 private:
    std_msgs::msg::String mode_msg_{};
    geometry_msgs::msg::Twist cmd_vel_{};

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
        avoidancer_sub_ptr_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr avoidancer_pub_ptr_;
    rclcpp::TimerBase::SharedPtr timer_;  // 发布者发布的定时器

    float front_distance_;  // 车前方障碍物的距离
    float back_distance_;   // 车后方障碍物的距离
    float min_front_distance_ = 16.0f;   
    float is_front_know_ = false;   
    bool turn_direction;    // 车避障时旋转的方向

    // 避障器订阅者回调函数
    void avoidancerSubCallback(
        const sensor_msgs::msg::LaserScan::ConstSharedPtr msg);
    // 避障器发布者回调函数
    void avoidancerPubCallback();

 public:
    // 判读移动模式函数
    void setMoveMode();
    // 发布信息函数
    void setPubMessage();
    // 获取避障方向
    void getDirection(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg);

    // 获取前后平均距离
    void getDistance(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg);
};

void Avoidancer::avoidancerSubCallback(
    const sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
    this->getDistance(msg);
    // this->front_distance_ = msg->ranges[0];
    // this->back_distance_ = msg->ranges[359];
    this->getDirection(msg);
}

void Avoidancer::getDirection(
    const sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
    uint count_left = 0;
    uint count_right = 0;
    float total_distance_left = 0;
    float total_distance_right = 0;
    float average_distnce_left = 0;
    float average_distnce_right = 0;

    for (int i = 0; i < 90; i++) {
        if (!std::isinf(msg->ranges.data()[i])) {
            count_left++;
            total_distance_left += msg->ranges.data()[i];
        }
        if (!std::isinf(msg->ranges.data()[719 - i])) {
            count_right++;
            total_distance_right += msg->ranges.data()[719 - i];
        }
    }

    average_distnce_left = total_distance_left / count_left;
    average_distnce_right = total_distance_right / count_right;

    this->turn_direction = average_distnce_left > average_distnce_right;
}

void Avoidancer::getDistance(
    const sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
    float total_front_distance = 0;
    float total_back_distance = 0;
    float count_front = 0;
    float count_back = 0;

    for (int i = 0; i < 25; i++) {
	    if (!std::isinf(msg->ranges.data()[i])) {
		    if (min_front_distance_ > msg->ranges.data()[i])
			    min_front_distance_ = msg->ranges.data()[i];
		    is_front_know_ = true;   
		    total_front_distance += msg->ranges.data()[i];
		    count_front++;
	    }
	    if (!std::isinf(msg->ranges.data()[719 - i])) {
		    total_front_distance += msg->ranges.data()[719 - i];
		    count_front++;
	    }
	    if (!std::isinf(msg->ranges.data()[359 + i])) {
		    total_back_distance += msg->ranges.data()[359 + i];
		    count_back++;
	    }
	    if (!std::isinf(msg->ranges.data()[359 - i])) {
		    total_back_distance += msg->ranges.data()[359 - i];
		    count_back++;
	    }
    }

    this->front_distance_ = total_front_distance / count_front;
    this->back_distance_ = total_back_distance / count_back;
}

void Avoidancer::avoidancerPubCallback() {
    this->setMoveMode();
    this->setPubMessage();
    this->avoidancer_pub_ptr_->publish(this->cmd_vel_);
}

void Avoidancer::setMoveMode() {

    // 如果前方小于0.5，且后方大于0.5则后退
//    if (this->front_distance_ < 0.5 && this->back_distance_ > 0.5) {
//        this->mode_msg_.data = "back";
//        RCLCPP_INFO(this->get_logger(), "mode: %s",
//                    this->mode_msg_.data.c_str());
//        return;
//    }

    // 如果前方小于0.5，且后方小于0.5则停止
//    if (this->front_distance_ < 0.5 && this->back_distance_ < 0.5) {
//        this->mode_msg_.data = "stop";
//        RCLCPP_INFO(this->get_logger(), "mode: %s",
//                    this->mode_msg_.data.c_str());
//        return;
//    }

// 如果前方小于0.5则根据两边的障碍物距离转向
//    if (this->front_distance_ < 0.5) {
	if (this->min_front_distance_ < 0.6 || !is_front_know_) 
		this->mode_msg_.data = this->turn_direction ? "left" : "right";
	// 都不满足则前进
	else
		this->mode_msg_.data = "forward";
	min_front_distance_ = 16.0f;
	is_front_know_ = false;   
	RCLCPP_INFO(this->get_logger(), "mode: %s", this->mode_msg_.data.c_str());
}

void Avoidancer::setPubMessage() {
    // 初始化发布的信息, 默认状态为停止
    this->cmd_vel_.linear.x = 0;
    this->cmd_vel_.linear.y = 0;
    this->cmd_vel_.linear.z = 0;
    this->cmd_vel_.angular.x = 0;
    this->cmd_vel_.angular.y = 0;
    this->cmd_vel_.angular.z = 0;

    if (this->mode_msg_.data == "forward") {
        this->cmd_vel_.linear.x = 0.3;
    }

    if (this->mode_msg_.data == "left") {
        this->cmd_vel_.angular.z = 1;
    }

    if (this->mode_msg_.data == "right") {
        this->cmd_vel_.angular.z = -1;
    }

    if (this->mode_msg_.data == "back") {
        this->cmd_vel_.linear.x = -0.3;
    }
}

#endif  // _ROOT_WORKSPACE_SRC_MYPKG_OBSTACLE_AVOIDANCE_INCLUDE_OBSTACLE_AVOIDANCE_HPP_
