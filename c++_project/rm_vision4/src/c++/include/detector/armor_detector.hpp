/**
 * @file armor_detector.hpp
 * @author chenjunnnnnn
 * @brief 识别部分来自chenjunnn自瞄开源
 * @version 0.1
 * @date 2023-04-05
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef DETECTOR_ARMOR_DETECTOR_HPP_
#define DETECTOR_ARMOR_DETECTOR_HPP_

// std
#include <cmath>
#include <string>
#include <vector>

// opencv
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>

// general
#include "message/message.hpp"

// detector
#include "armor.hpp"
#include "number_classifier.hpp"

/**
 * @brief 识别的class 包括debug的数据和识别过程的函数结构
 *
 */
class ArmorDetector
{
public:
    struct LightParams
    {
        // width / height
        double min_ratio;
        double max_ratio;
        // vertical angle
        double max_angle;
    };

    struct ArmorParams
    {
        double min_light_ratio;
        double min_small_center_distance;
        double max_small_center_distance;
        double min_large_center_distance;
        double max_large_center_distance;
        // horizontal angle
        double max_angle;
    };

    /**
     * @brief 识别装甲板
     *
     * @param init_min_l   二值化阈值
     * @param init_color    颜色
     * @param init_l        灯条
     * @param init_a        装甲
     */
    ArmorDetector(const int &init_min_l, const int &init_color,
                  const LightParams &init_l, const ArmorParams &init_a);

    int min_lightness;
    int detect_color;
    LightParams l;
    ArmorParams a;

    // Debug msgs
    std::vector<Light> debug_lights;
    std::vector<Armor> debug_armors;

    /**
     * @brief   预处理
     *
     * @param rbg_img   输入的图像
     * @return cv::Mat  处理后的图像
     */
    cv::Mat preprocessImage(const cv::Mat &rbg_img);

    /**
     * @brief 灯条查找
     *
     * @param rbg_img rbg
     * @param binary_img  binary
     * @return std::vector<Light>   返回找到的所有灯条
     */
    std::vector<Light> findLights(const cv::Mat &rbg_img,
                                  const cv::Mat &binary_img);

    /**
     * @brief 灯条匹配装甲
     *
     * @param lights 所有的灯条
     * @return std::vector<Armor>  匹配到的装甲
     */
    std::vector<Armor> matchLights(const std::vector<Light> &lights);

private:
    bool isLight(const Light &light);

    bool containLight(const Light &light_1, const Light &light_2,
                      const std::vector<Light> &lights);

    bool isArmor(Armor &armor);
};

ArmorDetector::ArmorDetector(const int &init_min_l, const int &init_color,
                             const LightParams &init_l, const ArmorParams &init_a)
    : min_lightness(init_min_l), detect_color(init_color), l(init_l),
      a(init_a) {}

cv::Mat ArmorDetector::preprocessImage(const cv::Mat &rgb_img)
{
    cv::Mat gray_img;
    cv::cvtColor(rgb_img, gray_img, cv::COLOR_RGB2GRAY);

    cv::Mat binary_img;
    cv::threshold(gray_img, binary_img, min_lightness, 255, cv::THRESH_BINARY);

    return binary_img;
}

std::vector<Light> ArmorDetector::findLights(const cv::Mat &rbg_img,
                                             const cv::Mat &binary_img)
{
    using std::vector;
    vector<vector<cv::Point>> contours;
    vector<cv::Vec4i> hierarchy;
    cv::findContours(binary_img, contours, hierarchy, cv::RETR_EXTERNAL,
                     cv::CHAIN_APPROX_SIMPLE);

    vector<Light> lights;
    this->debug_lights.clear();

    for (const auto &contour : contours)
    {
        if (contour.size() < 5)
            continue;

        auto r_rect = cv::minAreaRect(contour);
        auto light = Light(r_rect);

        if (isLight(light))
        {
            auto rect = light.boundingRect();
            if ( // Avoid assertion failed
                0 <= rect.x && 0 <= rect.width &&
                rect.x + rect.width <= rbg_img.cols && 0 <= rect.y &&
                0 <= rect.height && rect.y + rect.height <= rbg_img.rows)
            {
                int sum_r = 0, sum_b = 0;
                auto roi = rbg_img(rect);
                // Iterate through the ROI
                for (size_t i = 0; i < roi.rows; ++i)
                {
                    for (size_t j = 0; j < roi.cols; ++j)
                    {
                        if (cv::pointPolygonTest(
                                contour, cv::Point2f(j + rect.x, i + rect.y),
                                false) >= 0)
                        {
                            // if point is inside contour
                            sum_r += roi.at<cv::Vec3b>(i, j)[0];
                            sum_b += roi.at<cv::Vec3b>(i, j)[2];
                        }
                    }
                }
                // Sum of red pixels > sum of blue pixels ?
                light.color = sum_r > sum_b ? RED : BLUE;
                lights.emplace_back(light);
            }
        }
    }

    return lights;
}

bool ArmorDetector::isLight(const Light &light)
{
    // The ratio of light (short side / long side)
    float ratio = light.width / light.length;
    bool ratio_ok = l.min_ratio < ratio && ratio < l.max_ratio;

    bool angle_ok = light.tilt_angle < l.max_angle;

    // 25 < area < all / 20
    bool area_ok = light.size.area() < (WIDTH * HEIGHT) / 20 && light.size.area() > 25;

    bool is_light = ratio_ok && angle_ok && area_ok;

    // Fill in debug information
    Light light_data = light;
    // light_data.center.x = light.center.x;
    // light_data.ratio = ratio;
    // light_data.angle = light.tilt_angle;
    // light_data.is_light = is_light;
    this->debug_lights.push_back(light_data);

    return is_light;
}

std::vector<Armor> ArmorDetector::matchLights(const std::vector<Light> &lights)
{
    std::vector<Armor> armors;
    this->debug_armors.clear();

    // Loop all the pairing of lights
    for (auto light_1 = lights.begin(); light_1 != lights.end(); light_1++)
    {
        for (auto light_2 = light_1 + 1; light_2 != lights.end(); light_2++)
        {
            if (light_1->color != detect_color ||
                light_2->color != detect_color)
                continue;

            if (containLight(*light_1, *light_2, lights))
            {
                continue;
            }
            auto armor = Armor(*light_1, *light_2);
            if (isArmor(armor))
            {
                armors.emplace_back(armor);
            }
        }
    }

    return armors;
}

// Check if there is another light in the boundingRect formed by the 2 lights
bool ArmorDetector::containLight(const Light &light_1, const Light &light_2,
                                 const std::vector<Light> &lights)
{
    auto points = std::vector<cv::Point2f>{light_1.top, light_1.bottom,
                                           light_2.top, light_2.bottom};
    auto bounding_rect = cv::boundingRect(points);

    for (const auto &test_light : lights)
    {
        if (test_light.center == light_1.center ||
            test_light.center == light_2.center)
            continue;

        if (bounding_rect.contains(test_light.top) ||
            bounding_rect.contains(test_light.bottom) ||
            bounding_rect.contains(test_light.center))
        {
            return true;
        }
    }

    return false;
}

bool ArmorDetector::isArmor(Armor &armor)
{
    Light light_1 = armor.left_light;
    Light light_2 = armor.right_light;
    // Ratio of the length of 2 lights (short side / long side)
    float light_length_ratio = light_1.length < light_2.length
                                   ? light_1.length / light_2.length
                                   : light_2.length / light_1.length;
    bool light_ratio_ok = light_length_ratio > a.min_light_ratio;

    // Distance between the center of 2 lights (unit : light length)
    float avg_light_length = (light_1.length + light_2.length) / 2;
    float center_distance =
        cv::norm(light_1.center - light_2.center) / avg_light_length;
    bool center_distance_ok = (a.min_small_center_distance < center_distance &&
                               center_distance < a.max_small_center_distance) ||
                              (a.min_large_center_distance < center_distance &&
                               center_distance < a.max_large_center_distance);
    // Angle of light center connection
    cv::Point2f diff = light_1.center - light_2.center;
    float angle = std::abs(std::atan(diff.y / diff.x)) / CV_PI * 180;
    bool angle_ok = angle < a.max_angle;

    bool is_armor = light_ratio_ok && center_distance_ok && angle_ok;
    armor.armor_type =
        center_distance > a.min_large_center_distance ? LARGE : SMALL;
    // Fill in debug information
    Armor armor_data = armor;
    armor_data.center.x = (light_1.center.x + light_2.center.x) / 2;
    // armor_data.center_x = (light_1.center.x + light_2.center.x) / 2;
    // armor_data.light_ratio = light_length_ratio;
    // armor_data.center_distance = center_distance;
    armor_data.angle = angle;
    armor_data.is_armor = is_armor;
    armor_data.armor_type = (ArmorType)(armor.armor_type == SMALL);
    this->debug_armors.push_back(armor_data);

    return is_armor;
}

// static cv::Point2f cam_center_;
void drawResults(cv::Mat img, const std::vector<Light> &lights,
                 const std::vector<Armor> &armors)
{

    // Draw Lights
    for (const auto &light : lights)
    {
        auto color = light.color == RED ? cv::Scalar(255, 255, 0)
                                        : cv::Scalar(255, 0, 255);
        cv::ellipse(img, light, color, 2);
    }

    // Draw armors
    for (const auto &armor : armors)
    {
        cv::line(img, armor.left_light.top, armor.right_light.bottom,
                 cv::Scalar(0, 255, 0), 2);
        cv::line(img, armor.left_light.bottom, armor.right_light.top,
                 cv::Scalar(0, 255, 0), 2);
    }

    // Show numbers and confidence
    for (const auto &armor : armors)
    {
        cv::putText(img, armor.classfication_result, armor.left_light.top,
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 2);
    }
    // cv::imshow("", img);
    // Draw camera center
    // cv::circle(img, cam_center_, 5, cv::Scalar(255, 0, 0), 2);
}

/**
 * @brief 总体的识别函数
 *
 * @param img
 * @param binary_image
 * @param detector
 * @param classifier
 * @return std::vector<Armor>
 */
std::vector<Armor> detectArmors(cv::Mat img, cv::Mat &binary_image,
                                std::unique_ptr<ArmorDetector> &detector,
                                std::unique_ptr<NumberClassifier> &classifier)
{
    // auto start_time = this->now();
    // // Convert ROS img to cv::Mat
    // auto img = cv_bridge::toCvShare(img_msg, "rgb8")->image;

    // Detect armors
    // detector_->min_lightness = get_parameter("min_lightness").as_int();
    // detector_->detect_color = get_parameter("detect_color").as_int();

    // tjc-debug
    // detector->min_l.ightness = 200;
    // detector->detect_color = Color::ARMOR_RED;

    // rgb
    auto binary_img = detector->preprocessImage(img);
    binary_image = binary_img.clone();
    auto lights = detector->findLights(img, binary_img);
    auto armors = detector->matchLights(lights);

    // Extract numbers
    if (!armors.empty())
    {
        classifier->extractNumbers(img, armors);
        classifier->classify(armors);
    }

    // Publish debug info
    // if (debug_)
    if (true)
    {
        std::sort(detector->debug_lights.begin(), detector->debug_lights.end(),
                  [](const auto &l1, const auto &l2)
                  {
                      return l1.center.x < l2.center.x;
                  });
        std::sort(detector->debug_armors.begin(), detector->debug_armors.end(),
                  [](const auto &a1, const auto &a2)
                  {
                      return a1.center.x < a2.center.x;
                  });

        // lights_data_pub_->publish(detector_->debug_lights);
        // armors_data_pub_->publish(detector_->debug_armors);

        if (!armors.empty())
        {
            // Combine all number images to one
            std::vector<cv::Mat> number_imgs;
            number_imgs.reserve(armors.size());
            for (auto &armor : armors)
            {
                cv::resize(armor.number_img, armor.number_img,
                           cv::Size(20, 28));
                number_imgs.emplace_back(armor.number_img);
            }
            cv::Mat all_num_img;
            cv::vconcat(number_imgs, all_num_img);
        }
#ifdef _DEBUG_
        drawResults(img, lights, armors);
#endif // !_DEBUG_
    }

    return armors;
}

#endif // DETECTOR_ARMOR_DETECTOR_HPP_
