/**
 * @file follow.cpp
 * @author Chenkey-Z(2458305572@qq.com)
 * @version 0.1
 * @date 2024-1-9
 * @copyright Copyright (c) 2024
 */

#include "line_follow/follow.hpp"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Follow>("line_follow");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
