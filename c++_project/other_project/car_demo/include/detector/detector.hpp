/**
 * @file detector.hpp
 * @author luoyebai (2112216825@qq.com)
 * @brief detect
 * @version 0.1
 * @date 2023-06-28
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef INCLUDE_DETECTOR_HPP_
#define INCLUDE_DETECTOR_HPP_

// stl
#include <array>
#include <vector>
// include
#include "road.hpp"

/**
 * @brief Parameters of traffic lights
 *
 */
struct TrafficLightParams {
  int thresh;
  double min_ratio;
  double max_ratio;
  int min_area;
};

/**
 * @brief Parameters of roadway
 *!!! unuse
 */
struct RoadwayParams {
  int max_hsv[3];
  int min_hsv[3];
};

/**
 * @brief There are parameters to identify
 *
 */
struct DetectParams {
  RoadwayParams roadway_params;
  TrafficLightParams traffic_light_params;
  // src,binary,red,green,result
  bool debug_show[5];
};

/**
 * @brief Identified classes
 *
 */
class Detector {
public:
  Detector() = default;
  ~Detector() = default;

  /**
   * @brief Construct a new Detector object
   *
   * @param params config
   */
  explicit Detector(DetectParams params) : detect_params(params) {}

  // Identified parameters
  DetectParams detect_params{};

  /**
   * @brief detect
   * Overall implementation of identification
   * @param src input image
   * @param is_detect_traffic_light Enable or not detect traffic light

   * @param is_detect_road_way Enable or not detect road way
   */
  void detect(cv::Mat src, bool is_detect_traffic_light = true,
              bool is_detect_road_way = true);

  /**
   * @brief debug show
   *
   */
  void showImage() noexcept;

private:
  cv::Mat image_;
  cv::Mat hsv_image_;
  cv::Mat binary_img_;
  cv::Mat red_binary_img_;
  cv::Mat green_binary_img_;
  cv::Mat result_img_;
  TrafficLight traffic_light_;
  Roadway road_way_;

  bool isRedCircle(cv::Mat input);
  bool isGreenArrow(cv::Mat input);
  bool detectTrafficLight();
  bool detectRoadWay();
  void drawTrafficLight(bool is_draw_traffic_light);
  void drawRoadWay(bool is_draw_road_way);
  void drawResult(bool is_draw_traffic_light = true,
                  bool is_draw_road_way = true);
};

bool Detector::isRedCircle(cv::Mat input) {
  // circle  test
  cv::GaussianBlur(input, input, cv::Size(3, 3), 2, 2);
  auto m = cv::getStructuringElement(0, cv::Size(3, 3));
  cv::dilate(input, input, m);
  std::vector<cv::Vec3f> circles;
  cv::HoughCircles(input, circles, cv::HOUGH_GRADIENT, 1, 10, 100, 10, 0, 0);
  if (circles.size() >= 1) {
    cv::Point2f circle_center(circles[0][0], circles[0][1]);
    float circle_radius = circles[0][2];
    traffic_light_.round = Round{0, circle_center, circle_radius};
    traffic_light_.state = TrafficLight::State::STOP;
    return true;
  }

  return false;
}

bool Detector::isGreenArrow(cv::Mat input) {
  // circle  test
  cv::GaussianBlur(input, input, cv::Size(3, 3), 2, 2);
  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(input, contours, hierarchy, cv::RETR_EXTERNAL,
                   cv::CHAIN_APPROX_SIMPLE);
  for (const auto &contour : contours) {
    // contour must big
    if (contour.size() < 30)
      continue;
    auto arrow = Arrow(contour, detect_params.traffic_light_params.min_ratio,
                       detect_params.traffic_light_params.max_ratio,
                       detect_params.traffic_light_params.min_area);
    // get arrow state
    if (arrow.is_arrow) {
      traffic_light_.arrow = arrow;
      traffic_light_.state = arrow.left_or_right
                                 ? TrafficLight::State::LEFT_RUN
                                 : TrafficLight::State::RIGHT_RUN;
      return true;
    }
  }
  return false;
}

bool Detector::detectTrafficLight() {
  bool is_detect_ok = false;
  std::array<cv::Mat, 3> channels;
  cv::split(image_, channels);
  // channels-bgr
  // red
  cv::subtract(channels[2], channels[0], channels[2]);
  cv::subtract(channels[2], channels[1], channels[2]);
  cv::threshold(channels[2], red_binary_img_,
                detect_params.traffic_light_params.thresh, 255,
                cv::THRESH_BINARY);
  // green
  cv::subtract(channels[1], channels[0], channels[1]);
  cv::subtract(channels[1], channels[2], channels[1]);
  cv::threshold(channels[1], green_binary_img_,
                detect_params.traffic_light_params.thresh, 255,
                cv::THRESH_BINARY);
  traffic_light_.state = TrafficLight::State::NONE;

  // Judgment Arrow

  is_detect_ok = isGreenArrow(green_binary_img_);
  if (is_detect_ok)
    return true;

  // If it's not an arrow, determine if it's a circle
  is_detect_ok = isRedCircle(red_binary_img_);
  return is_detect_ok;
}

void Detector::drawTrafficLight(bool is_draw_traffic_light) {
  if (traffic_light_.state == TrafficLight::State::NONE)
    putText(result_img_, "NONE", cv::Point(0, 50), cv::FONT_HERSHEY_SIMPLEX, 2,
            cv::Scalar(0, 255, 0), 4, 8);
  if (is_draw_traffic_light) {
    const char *state_text[4]{"NONE", "STOP", "LEFT_RUN", "RIGHT_RUN"};
    putText(result_img_, state_text[traffic_light_.state], cv::Point(0, 50),
            cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(0, 255, 0), 4, 8);
    if (traffic_light_.state == TrafficLight::State::STOP) {
      cv::circle(result_img_, traffic_light_.round.center,
                 traffic_light_.round.radius, cv::Scalar(0, 255, 0), 2,
                 cv::LINE_AA);
      cv::circle(result_img_, traffic_light_.round.center, 1,
                 cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
    }
    if (traffic_light_.state > 1) {
      cv::drawContours(
          result_img_,
          std::vector<std::vector<cv::Point>>{traffic_light_.arrow.contour}, -1,
          cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
    }
  }
  return;
}

bool Detector::detectRoadWay() {
  // get black binary image
  std::array<cv::Mat, 3> channels;
  cv::split(image_, channels);
  cv::threshold(channels[0], channels[0], 100, 255, cv::THRESH_BINARY);
  cv::threshold(channels[1], channels[1], 100, 255, cv::THRESH_BINARY);
  cv::threshold(channels[2], channels[2], 100, 255, cv::THRESH_BINARY);
  cv::bitwise_not(channels[0], channels[0]);
  cv::bitwise_not(channels[1], channels[1]);
  cv::bitwise_not(channels[2], channels[2]);
  cv::bitwise_and(channels[0], channels[1], channels[0]);
  cv::bitwise_and(channels[0], channels[2], channels[0]);
  binary_img_ = channels[0];
  // Deburring
  auto m = cv::getStructuringElement(0, cv::Size(5, 5));
  cv::erode(binary_img_, binary_img_, m);
  cv::dilate(binary_img_, binary_img_, m);
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(binary_img_, contours, cv::RETR_EXTERNAL,
                   cv::CHAIN_APPROX_SIMPLE);

  // At least two sides of the road
  if (contours.size() < 2)
    return false;

  // Selection of roads on both sides
  for (auto contour_1 = contours.begin(); contour_1 != contours.end(); contour_1++) 
{
    auto rect_1 = cv::minAreaRect(*contour_1);
    // x ok
    bool is_rect_1_x_ok = rect_1.center.x < image_.size().width / 2;
    // area ok
    bool is_rect_1_area_ok = rect_1.size.area() > 10000;

    bool is_rect_1_ok = is_rect_1_area_ok && is_rect_1_x_ok;
    if (!is_rect_1_ok)
      continue;
    for (auto contour_2 = contour_1 + 1; contour_2 != contours.end();
         contour_2++) {
      auto rect_2 = cv::minAreaRect(*contour_2);
      bool is_rect_2_x_ok = rect_2.center.x > image_.size().width / 2;
      bool is_rect_2_area_ok = rect_2.size.area() > 200;
      bool is_rect_2_ok = is_rect_2_area_ok && is_rect_2_x_ok;
      if (!is_rect_2_ok)
        continue;
      // get new road way
      auto new_road_way = Roadway(*contour_1, *contour_2);

      // The Right road way
      if (new_road_way.is_road_ok) 
	  {
        road_way_ = new_road_way;
        return true;
      }
    }
  }

  return false;
}

void Detector::drawRoadWay(bool is_draw_road_way) {
  if (is_draw_road_way) {
    putText(result_img_, "ROADWAY FIND", cv::Point(0, 150),
            cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(0, 255, 0), 4, 8);
    cv::drawContours(
        result_img_,
        std::vector<std::vector<cv::Point>>{road_way_.left_line.contour,
                                            road_way_.right_line.contour},
        -1, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
    cv::Point aim_point =
        (road_way_.left_line.center + road_way_.right_line.center) / 2;
    cv::Point car_center_point =
        cv::Point(image_.size().width / 2, image_.size().height / 2);
    cv::circle(result_img_, car_center_point, 10, cv::Scalar(0, 0, 255), 2,
               cv::LINE_AA);
    cv::circle(result_img_, aim_point, 10, cv::Scalar(0, 255, 0), 2,
               cv::LINE_AA);
    cv::line(result_img_, aim_point, car_center_point, cv::Scalar(0, 255, 0), 2,
             cv::LINE_AA);
  } else {
    putText(result_img_, "NONE", cv::Point(0, 150), cv::FONT_HERSHEY_SIMPLEX, 2,
            cv::Scalar(0, 255, 0), 4, 8);
  }
}

void Detector::drawResult(bool is_draw_traffic_light, bool is_draw_road_way) {
  result_img_ = image_.clone();
  // draw traffic light state
  drawTrafficLight(is_draw_traffic_light);
  // draw road way state
  drawRoadWay(is_draw_road_way);
}

void Detector::detect(cv::Mat src, bool is_detect_traffic_light,
                      bool is_detect_road_way) {
  image_ = src.clone();
  cv::cvtColor(image_, hsv_image_, cv::COLOR_BGR2HSV);
  bool is_find_traffic_light = false, is_find_road_way = false;
  if (is_detect_traffic_light)
    is_find_traffic_light = detectTrafficLight();
  if (is_detect_road_way)
    is_find_road_way = detectRoadWay();
  drawResult(is_find_traffic_light, is_find_road_way);
  return;
}

void Detector::showImage() noexcept {
  // debug
  if (detect_params.debug_show[0])
    cv::imshow("src", image_);
  if (detect_params.debug_show[1])
    cv::imshow("binary", binary_img_);
  if (detect_params.debug_show[2])
    cv::imshow("red_binary", red_binary_img_);
  if (detect_params.debug_show[3])
    cv::imshow("green_binary", green_binary_img_);
  if (detect_params.debug_show[4])
    cv::imshow("result", result_img_);
  return;
}

#endif // !INCLUDE_DETECTOR_HPP_
