/**
 * @file main.cpp
 * @author luoyebai (2112216825@qq.com)
 * @brief car demo
 * @version 0.1
 * @date 2023-06-28
 *
 * @copyright Copyright (c) 2023
 *
 */

// #define DEBUG
#ifndef DEBUG
// std
#include <memory>
// opencv
#include <opencv2/core.hpp>
#include <opencv2/core/persistence.hpp>
// include
#include "detector/detector.hpp"
#include "read_config/read_config.hpp"

int main() {
  int cap_list = -1;
  DetectParams init_params;
  // config read
  std::unique_ptr<FileRead> file_read =
      std::make_unique<FileRead>("../config/config.yaml");
  // camera list config
  file_read->getConfig("use_camera", cap_list);
  // detect config
  file_read->getConfig("min_threshold",
                       init_params.traffic_light_params.thresh);
  file_read->getConfig("min_ratio", init_params.traffic_light_params.min_ratio);
  file_read->getConfig("max_ratio", init_params.traffic_light_params.max_ratio);
  file_read->getConfig("min_area", init_params.traffic_light_params.min_area);
  // debug show config
  file_read->getConfig("show_src_img", init_params.debug_show[0]);
  file_read->getConfig("show_binary_img", init_params.debug_show[1]);
  file_read->getConfig("show_red_img", init_params.debug_show[2]);
  file_read->getConfig("show_green_img", init_params.debug_show[3]);
  file_read->getConfig("show_result_img", init_params.debug_show[4]);

  // detector
  std::unique_ptr<Detector> detector = std::make_unique<Detector>(init_params);
  cv::Mat src;
  cv::VideoCapture capture;
  if (cap_list == -1) {
    capture.open("../docs/小车项目/真实环境行车记录仪.avi");
  } else {
    capture.open(cap_list);
  }
  while (true) {
    capture >> src;
    detector->detect(src);
    detector->showImage();
    if (cv::waitKey(50) >= 0) {
      cv::destroyAllWindows();
      break;
    }
  }
  return 0;
}

// Here is a code for obtaining HSV online
#else
// GET HSV TEST
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

#define WINDOW_NAME "【效果图窗口】" // 为窗口标题定义的宏

void pickHighLightFire(Mat &inputFrame, Mat &outputFrame);
void on_MouseHandle(int event, int x, int y, int flags, void *param);

int max_h = 0, max_s = 0, max_v = 0;
int min_h = 255, min_s = 255, min_v = 255;
int main() {
  int multiple = 3; // 图片的放大倍数
  auto cap = VideoCapture("../docs/小车项目/真实环境行车记录仪.avi");
  Mat inputImage;
  while (1) {
    cap >> inputImage;
    Mat outputImage;
    resize(inputImage, inputImage,
           Size(multiple * inputImage.cols, multiple * inputImage.rows));
    cvtColor(inputImage, outputImage, COLOR_BGR2HSV);

    // 设置鼠标操作回调函数
    namedWindow(WINDOW_NAME);
    setMouseCallback(WINDOW_NAME, on_MouseHandle, (void *)&outputImage);
    imshow(WINDOW_NAME, inputImage);
    while (1) {
      if (waitKey(10) == 27)
        break; // 按下ESC键，程序退出
    }
    waitKey();
  }
  return 0;
}

void on_MouseHandle(int event, int x, int y, int flags, void *param) {

  Mat &image = *(cv::Mat *)param;
  switch (event) {
  // 左键按下消息
  case EVENT_LBUTTONDOWN: {
    max_h = MAX(max_h, image.at<Vec3b>(y, x)[0]);
    max_s = MAX(max_v, image.at<Vec3b>(y, x)[1]);
    max_v = MAX(max_v, image.at<Vec3b>(y, x)[2]);
    min_h = MIN(min_h, image.at<Vec3b>(y, x)[0]);
    min_s = MIN(min_s, image.at<Vec3b>(y, x)[1]);
    min_v = MIN(min_v, image.at<Vec3b>(y, x)[2]);
    cout << static_cast<int>(max_h) << ",";
    cout << static_cast<int>(max_s) << ",";
    cout << static_cast<int>(max_v) << "  ";
    cout << static_cast<int>(min_h) << ",";
    cout << static_cast<int>(min_s) << ",";
    cout << static_cast<int>(min_v) << "\n";
  } break;
  }
}

#endif // DEBUGs