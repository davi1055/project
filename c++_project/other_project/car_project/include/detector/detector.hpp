/**
 * @file detector.hpp
 * @author 戴炜程(1224802565@qq.com)
 * @version 0.1
 * @date 2023-06-28
 * @copyright Copyright (c) 2023
 */

#ifndef DETECTOR_HPP_
#define DETECTOR_HPP_
#include <opencv4/opencv2/opencv.hpp>


struct VisionData
{
    int fixed_derection = 0; // 0前进 1左 2右
    int signal = 0;          // 0停 1左 2右
};

class Detector
{
private:
    bool find_line_patrol = 0;
    uint color = 0;                // 0红 1绿
    uint direction_correction = 0; // 0方向正确 1左 2右
    uint signal_cmd = 0;           // 信号灯命令 0停 1左 2右
    cv::Point2f car_direction;     // 小车前进的方向
    cv::Point2f targrt_point;      // 前进的目标点
    cv::Mat src;
    cv::Mat processed_img;
    cv::Mat debug_img;

public:
    Detector() = default;
    ~Detector() = default;
    void Detect(cv::Mat src);         // 识别总函数
    void GetSourceImage(cv::Mat src); // 获得原图函数
    void ImageProcessing();           // 图像处理函数
    void SignalLightDetect();         // 信号灯识别函数
    bool JudgeLinePatrol();           // 黑线判断函数
    void DirectControl();             // 巡线方向控制函数
    void SentVisionData(VisionData vision_data);            // 传出视觉数据函数
};

void Detector::Detect(cv::Mat src)
{
    this->find_line_patrol = 0;
    this->signal_cmd = 0;
    this->find_line_patrol = JudgeLinePatrol();
    this->DirectControl();
    cv::imshow("",this->src);

    if (this->color == 0)
    {
        std::cout << "red light ! Stop!!\n";
        this->signal_cmd = 0;
        this->src = src.clone();
        ImageProcessing();
    }
    if (this->color == 1)
    {
        this->src = src.clone();
        ImageProcessing();
        if (!processed_img.empty())
        {
            this->SignalLightDetect();
        }
    }
    cv::imshow("1", debug_img);
}

void Detector::GetSourceImage(cv::Mat src)
{
    // cv::resize(src, src, cv::Size(), 0.3, 0.3);
    this->src = src.clone();
    this->car_direction = cv::Point2f(src.cols / 2, src.rows / 2);
}

void Detector::ImageProcessing()
{
    std::vector<cv::Mat> channels;
    cv::Mat threshold_img;
    cv::Mat channel_img;
    cv::Mat img = this->src.clone();
    cv::split(img, channels);

    // 识别绿
    cv::subtract(channels[1], channels[2], channel_img);
    // cv::subtract(channel_img, channels[0], channel_img);
    cv::threshold(channel_img, threshold_img, 100, 255, cv::THRESH_BINARY);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(threshold_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    cv::Mat black_img = cv::Mat::zeros(img.size(), CV_8UC1);
    cv::drawContours(black_img, contours, -1, cv::Scalar(255, 255, 255), 1, 8);
    if (contours.size() < 1)
    {
        this->color = 0;
    }
    else
    {
        this->color = 1;
        this->processed_img = black_img.clone();
    }
}

void Detector::SignalLightDetect()
{
    cv::Mat processed_img = this->processed_img.clone();
    cv::dilate(processed_img, processed_img, 10, cv::Point(-1, -1), 1); // 膨胀

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(this->processed_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::vector<cv::Moments> mu(contours.size());
    for (size_t i = 0; i < contours.size(); i++)
    {
        mu[i] = moments(contours[i], false);
    }
    std::vector<cv::Point2f> centroids(contours.size());
    for (size_t i = 0; i < contours.size(); i++)
    {
        centroids[i] = cv::Point2f(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);
    }
    cv::circle(processed_img, centroids[0], 1, cv::Scalar(255, 255, 255), 1, 8, 0);

    cv::Point2f center;
    float radius;

    cv::RotatedRect rotrect;
    cv::Point2f rect_points[4]; // point[0]左下:x最小 point[1]左上:y最小 point[2]右上:x最大 point[3]右下:y最大
    cv::Mat boxPoints2f, boxPointsCov;
    // 获取最小外接矩形点阵
    rotrect = cv::minAreaRect(contours[0]);
    rotrect.points(rect_points);
    cv::boxPoints(rotrect, boxPoints2f);
    boxPoints2f.assignTo(boxPointsCov, CV_32S);

    // 绘制旋转矩形及其顶点
    cv::polylines(processed_img, boxPointsCov, true, cv::Scalar(255, 255, 255), 2);
    cv::circle(processed_img, center, 1, cv::Scalar(255, 255, 255), 1, 8, 0);
    cv::Point2f rect_point((rect_points[0].x + rect_points[2].x) / 2, (rect_points[0].y + rect_points[2].y) / 2);
    if (centroids[0].x > rect_point.x)
    {
        this->signal_cmd = 2;
        // std::cout << "turn right!!\n";
    }
    if (centroids[0].x < rect_point.x)
    {
        this->signal_cmd = 1;
        // std::cout << "turn left!!\n";
    }
    debug_img = processed_img.clone();
}

bool Detector::JudgeLinePatrol()
{
    this->targrt_point = cv::Point2f(0, 0);

    std::vector<std::vector<cv::Point>> contours_test;
    // 获取黑色线条
    cv::Mat src = this->src.clone();
    cv::Mat binary_img;
    std::vector<cv::Mat> channels;
    cv::split(src, channels);
    cv::threshold(channels[0], channels[0], 100, 255, cv::THRESH_BINARY);
    cv::threshold(channels[1], channels[1], 100, 255, cv::THRESH_BINARY);
    cv::threshold(channels[2], channels[2], 100, 255, cv::THRESH_BINARY);
    cv::bitwise_not(channels[0], channels[0]);
    cv::bitwise_not(channels[1], channels[1]);
    cv::bitwise_not(channels[2], channels[2]);
    cv::bitwise_and(channels[0], channels[1], channels[0]);
    cv::bitwise_and(channels[0], channels[2], channels[0]);
    binary_img = channels[0];

    auto m = cv::getStructuringElement(0, cv::Size(5, 5));
    cv::erode(binary_img, binary_img, m);
    cv::dilate(binary_img, binary_img, m);
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    cv::Mat black_img = cv::Mat::zeros(src.size(), CV_8UC1);
    cv::drawContours(black_img, contours, -1, cv::Scalar(100, 100, 100), 1, 8);

    if (contours.size() < 2)
    {
        debug_img = black_img.clone();
        return false;
    }

    for (auto contour_1 = contours.begin(); contour_1 != contours.end(); contour_1++)
    {
        auto rect_1 = cv::minAreaRect(*contour_1);
        bool is_rect_1_x_ok = rect_1.center.x < src.size().width / 2;
        bool is_rect_1_area_ok = rect_1.size.area() > 10000;
        bool is_rect_1_ok = is_rect_1_area_ok && is_rect_1_x_ok;
        if (!is_rect_1_ok)
        {
            debug_img = black_img.clone();
            continue;
        }
        for (auto contour_2 = contour_1 + 1; contour_2 != contours.end(); contour_2++)
        {
            auto rect_2 = cv::minAreaRect(*contour_2);
            bool is_rect_2_x_ok = rect_2.center.x > src.size().width / 2;
            bool is_rect_2_area_ok = rect_2.size.area() > 200;
            bool is_rect_2_ok = is_rect_2_area_ok && is_rect_2_x_ok;
            if (!is_rect_2_ok)
            {
                debug_img = black_img.clone();
                continue;
            }
            if (rect_1.center.y - rect_2.center.y < 100)
            {
                cv::Point2f rect_points_1[4]; // point[0]左下:x最小 point[1]左上:y最小 point[2]右上:x最大 point[3]右下:y最大
                cv::Mat boxPoints2f_1, boxPointsCov_1;
                rect_1.points(rect_points_1);
                cv::boxPoints(rect_1, boxPoints2f_1);
                boxPoints2f_1.assignTo(boxPointsCov_1, CV_32S);
                cv::polylines(black_img, boxPointsCov_1, true, cv::Scalar(255, 255, 255), 2);

                cv::Point2f rect_points_2[4]; // point[0]左下:x最小 point[1]左上:y最小 point[2]右上:x最大 point[3]右下:y最大
                cv::Mat boxPoints2f_2, boxPointsCov_2;
                rect_2.points(rect_points_2);
                cv::boxPoints(rect_2, boxPoints2f_2);
                boxPoints2f_2.assignTo(boxPointsCov_2, CV_32S);
                cv::polylines(black_img, boxPointsCov_2, true, cv::Scalar(255, 255, 255), 2);

                cv::Point2f rect_1_center_point = cv::Point2f((rect_points_1[0].x + rect_points_1[2].x) / 2, (rect_points_1[0].y + rect_points_1[2].y) / 2);
                cv::Point2f rect_2_center_point = cv::Point2f((rect_points_2[0].x + rect_points_2[2].x) / 2, (rect_points_2[0].y + rect_points_2[2].y) / 2);
                this->targrt_point = cv::Point2f((rect_1_center_point.x + rect_2_center_point.x) / 2, (rect_1_center_point.y + rect_2_center_point.y) / 2);
                cv::circle(black_img, this->targrt_point, 3, cv::Scalar(255, 255, 255), 1, 8, 0);
                cv::circle(black_img, this->car_direction, 1, cv::Scalar(255, 255, 255), 1, 8, 0);
                debug_img = black_img.clone();
                return true;
            }
        }
    }
    return false;
    debug_img = black_img.clone();
}

void Detector::DirectControl()
{
    this->direction_correction = 0;

    if (this->targrt_point.x == 0 && this->targrt_point.y == 0)
    {
        this->direction_correction = 0;
        std::cout << "go straight\n";
    }
    else
    {
        // 直行
        if (this->car_direction.x == this->targrt_point.x)
        {
            this->direction_correction = 0;
            std::cout << "go straight\n";
        }

        // 向右修正
        if (this->car_direction.x < this->targrt_point.x && abs(this->car_direction.x - this->targrt_point.x) < 50)
        {
            this->direction_correction = 2;
            std::cout << "right fix!\n";
        }

        // 向左修正
        if (this->car_direction.x > this->targrt_point.x && abs(this->car_direction.x - this->targrt_point.x) < 50)
        {
            this->direction_correction = 1;
            std::cout << "left fix!\n";
        }
    }
}

void Detector::SentVisionData(VisionData vision_data)
{
    vision_data.fixed_derection = direction_correction;
    vision_data.signal = signal_cmd;
}

#endif // DETECTOR_HPP_