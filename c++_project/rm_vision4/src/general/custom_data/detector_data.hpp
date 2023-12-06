#ifndef GENERAL_CUSTOM_DATA_DETECTOR_DATA_HPP_
#define GENERAL_CUSTOM_DATA_DETECTOR_DATA_HPP_

#include "lower_computer_data.hpp"

enum ArmorType
{
    SMALL = 0,
    LARGE = 1,
    BUFF  = 2
};

struct Detection_pack
{
    bool findArmor;
    ArmorType type;
    cv::Point2f centerPoint;
    std::vector<cv::Point2f> cornerPoints;
    cv::Mat img;
    float timestamp;
    Imu imu;
    float speed;
};
#endif // !GENERAL_CUSTOM_DATA_DETECTOR_DATA_HPP_