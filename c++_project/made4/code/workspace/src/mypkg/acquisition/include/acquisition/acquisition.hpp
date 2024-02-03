/**
 * @file acquistion_node.hpp
 * @author luoyebai (2112216825@qq.com)
 * @brief camera captures image node
 * @version 0.1
 * @date 2023-04-27
 *
 * @copyright Copyright (c) 2023
 *
 */

#pragma once
// ROS
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>  // NOLINT
// OpenCV
#include <opencv2/opencv.hpp>

/**
 * @brief acquire image and publish it
 *
 */
class AcquisitionNode : public rclcpp::Node {
 public:
    /**
     * @brief Construct a new Acquisition Node object
     *
     * @param options
     */
    AcquisitionNode() : Node("acquisition") {
        // set log level
        this->declare_parameter("rcl_log_level", 0);
	this->declare_parameter("exposure",40);
        this->get_parameter("rcl_log_level", log_level_);
        this->get_parameter("exposure", exposure_);
        this->get_logger().set_level((rclcpp::Logger::Level)log_level_);
	param_sub_ = std::make_shared<rclcpp::ParameterEventHandler>(this);

	param_cb_handle_ = param_sub_->add_parameter_callback("exposure",[this](const rclcpp::Parameter & p){exposure_ = p.as_int();
			camera_.set(cv::CAP_PROP_EXPOSURE,exposure_);
			});

	// camera init
	camera_ = cv::VideoCapture(0);
	camera_.set(cv::CAP_PROP_AUTO_EXPOSURE, 1);
	camera_.set(cv::CAP_PROP_EXPOSURE,exposure_);
	camera_.set(cv::CAP_PROP_GAIN,0);
	camera_.set(cv::CAP_PROP_BRIGHTNESS,0);
	camera_.set(cv::CAP_PROP_CONTRAST,64);
	camera_.set(cv::CAP_PROP_SATURATION,128);
	RCLCPP_INFO(this->get_logger(),"contrast:%.0f;gain:%.0f;saturation:%.0f;exposure:%.0f;brightness:%.0f",
			camera_.get(cv::CAP_PROP_CONTRAST),
			camera_.get(cv::CAP_PROP_GAIN),
			camera_.get(cv::CAP_PROP_SATURATION),
			camera_.get(cv::CAP_PROP_EXPOSURE),
			camera_.get(cv::CAP_PROP_BRIGHTNESS)
		   );


        while (camera_.isOpened() == false) {
            RCLCPP_INFO(this->get_logger(), "camera init failed,sleep 1s");
            rclcpp::sleep_for(std::chrono::seconds(1));
        }
        // publisher init
        RCLCPP_INFO(this->get_logger(), "acquisition node start");
        image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
            "publish_image", 1);

        // timer binding callback function
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(0),
            std::bind(&AcquisitionNode::TimerCallBack, this));
    }

 private:
    int log_level_;
    int exposure_;
    std::shared_ptr<rclcpp::ParameterEventHandler> param_sub_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> param_cb_handle_;
    cv::VideoCapture camera_;
    const sensor_msgs::msg::CameraInfo camera_info_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    /**
     * @brief callback function for publishing message
     *
     */
    void TimerCallBack();
};

void AcquisitionNode::TimerCallBack() {
    this->get_logger().set_level((rclcpp::Logger::Level)log_level_);
    cv::Mat src;
    if (!camera_.read(src)) {
        RCLCPP_WARN(this->get_logger(), "camera can't open");
        camera_.release();
        return;
    }
    // publish message
    sensor_msgs::msg::Image img_msg;
    std_msgs::msg::Header header;
    cv_bridge::CvImage cv_bridge;
    header.stamp = this->get_clock()->now();
    //cv::resize(src, src, cv::Size(160, 120));
    cv_bridge =
        cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, src);
    cv_bridge.toImageMsg(img_msg);
    // publish
    image_publisher_->publish(img_msg);
}
