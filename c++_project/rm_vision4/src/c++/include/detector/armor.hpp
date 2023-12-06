/**
 * @file armor.hpp
 * @author chenjunnnnnn
 * @brief 来自chenjunnn自瞄开源
 * @version 0.1
 * @date 2023-04-05
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef DETECTOR_ARMOR_HPP_
#define DETECTOR_ARMOR_HPP_

// std
#include <algorithm>
#include <string>

// opencv
#include <opencv2/core.hpp>

// general
#include "message/message.hpp"

// const int RED = 0;
// const int BLUE = 1;

/**
 * @brief light结构体 包括长宽,上下两点,颜色,旋转角度等信息
 *
 */
struct Light : public cv::RotatedRect
{
  Light() = default;
  explicit Light(cv::RotatedRect box) : cv::RotatedRect(box)
  {
    cv::Point2f p[4];
    box.points(p);
    std::sort(p, p + 4, [](const cv::Point2f &a, const cv::Point2f &b)
              { return a.y < b.y; });
    top = (p[0] + p[1]) / 2;
    bottom = (p[2] + p[3]) / 2;

    length = cv::norm(top - bottom);
    width = cv::norm(p[0] - p[1]);

    tilt_angle = std::atan2(std::abs(top.x - bottom.x), std::abs(top.y - bottom.y));
    tilt_angle = tilt_angle / CV_PI * 180;
  }

  int color;
  cv::Point2f top, bottom;
  double length;
  double width;
  float tilt_angle;
};

/**
 * @brief armor结构体 包括类型,角度,中心,数字,左右灯条和中心等信息
 *
 */
struct Armor
{
  Armor() = default;
  Armor(const Light &l1, const Light &l2)
  {
    if (l1.center.x < l2.center.x)
    {
      left_light = l1, right_light = l2;
    }
    else
    {
      left_light = l2, right_light = l1;
    }
    center = (left_light.center + right_light.center) / 2;
  }
  bool is_armor;
  float angle;
  Light left_light, right_light;
  cv::Point2f center;

  cv::Mat number_img;

  // char number;
  std::string number;
  float similarity;
  float confidence;
  std::string classfication_result;
  ArmorType armor_type;
};

#endif // DETECTOR_ARMOR_HPP_
