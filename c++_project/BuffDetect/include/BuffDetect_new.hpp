/**
 * @file buff_detector.hpp
 * @author davi(1224802565@qq.com)
 * @brief 能量机关识别代码
 * @version 0.1
 * @date 2023-05-08
 * @copyright Copyright (c) 2023
 */

#ifndef BUFFDETECT_BUFFDETECT_H_
#define BUFFDETECT_BUFFDETECT_H_

#include <iostream>
#include <cstdlib>
#include <math.h>

// STL
#include <vector>

#include <eigen3/Eigen/Dense>
#include <opencv4/opencv2/core/eigen.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/core.hpp>
#include <opencv2/opencv.hpp>

bool compare_radius(const cv::Point3d &p1, const cv::Point3d &p2)
{
    return p1.z < p2.z;
}
bool compare_contours_len(const std::vector<cv::Point> &c1, const std::vector<cv::Point> &c2)
{
    double l1 = cv::arcLength(c1, true);
    double l2 = cv::arcLength(c2, true);
    return l1 < l2;
}

float GetLineAngle(cv::Vec4f line) // 起点xy 终点xy
{

    cv::Point p1(line[0], line[1]);
    cv::Point p2(line[2], line[3]);

    float angle = 0;

    float angle_ = atan2(p1.y - p2.y, p1.x - p2.x);
    angle = (angle_ * 180) / CV_PI;
    if (angle_ >= -CV_PI && angle_ < 0)
    {
        angle = 180 + (180 + angle);
    }

    return angle;
}

class Buff
{
private:
    uint color;                                                                                                     // RED==2,BLUE==3
    const std::vector<cv::Point3f> objectPoints = {{-186, 183, 0}, {-186, -190, 0}, {186, 183, 0}, {186, -190, 0}}; // 物体世界坐标系的坐标点(左上，左下，右上，右下)
    std::vector<cv::Point2f> imagePoints_;                                                                          // 物体在像素坐标系上的坐标点(左下，左上,右上，右下)
    std::vector<cv::Point2f> imagePoints;                                                                           // 物体在像素坐标系上的不变坐标点(左上，左下,右上，右下)
    cv::Mat rvec;                                                                                                   // 旋转向量
    cv::Mat tvec;                                                                                                   // 平移向量
    float radius_armor_center = cv::norm(this->energy_organ_center - this->armor_center);                           // 能量机关中心到装甲板中心的半径
    float radius_energy_organ;                                                                                      // 能量机关完整半径
    cv::Point2f energy_organ_center;                                                                                // 能量机关圆心坐标
    cv::Point2f armor_center = cv::Point2f(-1, -1);                                                                 // 待击打装甲板中心坐标

    double angle_Energy_organ_center2armor_center; // 能量机关中心只指向装甲板中心的向量的角度
    cv::Mat src;
    cv::Mat processed_img;         // 经过ImageProcess()函数处理过得图像
    bool if_find_contours = false; // 是否找到轮廓

public:
    // predict
    std::vector<std::vector<cv::Point2f>> judge_points; // 判断大小能量机关所需的数据点
    double angular_velocity;                            // 能量机关转动的角速度
    double FPS;
    float small_energy = 20 * CV_PI / 60; // 小能量机关角速度(rad/s)

    cv::Point2d armor_roi_position; // 装甲板中心点在ROI中的坐标

    cv::Point2f GetArmorCenter()
    {
        return armor_center;
    }

    std::vector<cv::Point2f> GetImagePoints()
    {
        return imagePoints;
    }

    // 总处理函数
    void Process(cv::Mat src)
    {
        this->ImageProcess(src);
        if (this->if_find_contours)
        {
            this->FindCenter();
            this->AngleSolve();
            // this->Predict();
        }
    }

    // 是否找到待击打装甲板中心
    bool IfFindBuff()
    {
        return this->armor_center.x == -1 && this->armor_center.y == -1 ? false : true;
    }

    Buff(uint init_color)
    {
        color = init_color;
    }

    /**
     * @brief  图像处理,返回扇页轮廓
     *
     * @param src 原图像
     * @param color 需要筛选的颜色
     * @return
     */
    void ImageProcess(cv::Mat src)
    {
        cv::resize(src, src, cv::Size(2*src.cols / 3, 2*src.rows / 3));
        this->src = src;

        std::vector<cv::Mat> channels;
        cv::Mat channel_img;
        cv::Mat threshold_img;
        cv::Mat test = cv::Mat::zeros(src.size(), CV_8UC3);
        cv::split(src, channels);
        if (this->color == 2)
        {
            cv::subtract(channels[2], channels[0], channel_img); // 识别红
            cv::subtract(channel_img, channels[1], channel_img);
            cv::threshold(channel_img, threshold_img, 100, 255, cv::THRESH_BINARY);
        }
        if (this->color == 3)
        {
            cv::subtract(channels[0], channels[2], channel_img); // 识别蓝
            cv::subtract(channel_img, channels[1], channel_img);
            cv::threshold(channel_img, threshold_img, 120, 255, cv::THRESH_BINARY);
        }

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(threshold_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        if (contours.size() > 1)
        {
            this->if_find_contours = true;
        }
        this->processed_img = threshold_img;
    }

    /**
     * @brief 寻找需要击打的能量机关的装甲板中心
     *
     * @param
     * @return ArmorCenter
     */
    void FindCenter()
    {
        std::vector<cv::Point3f> armor_data; // center.x center.y raduis
        cv::Mat processed_img =this->processed_img.clone();
        

        // cv::Mat element_1 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2));
        cv::Mat element_2 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1, 1));

        // cv::morphologyEx(processed_img, processed_img, cv::MORPH_CLOSE, element_1);
        cv::morphologyEx(processed_img, processed_img, cv::MORPH_OPEN, element_2);
        cv::imshow("test",processed_img);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(processed_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);


        // 凸包检测
        std::vector<std::vector<cv::Point>> hull(contours.size());
        for (int i = 0; i < contours.size(); i++)
        {
            // 寻找凸包
            cv::convexHull(cv::Mat(contours[i]), hull[i], false);
        }
        for (int i = 0; i < contours.size(); i++)
        {
            cv::fillConvexPoly(this->processed_img, hull[i], cv::Scalar(255, 255, 255), cv::LINE_AA, 0);
        }

        std::vector<std::vector<cv::Point>> contours_;
        cv::findContours(this->processed_img, contours_, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        cv::Mat black_img = cv::Mat::zeros(this->processed_img.size(), CV_8UC1);
        cv::drawContours(black_img, contours_, -1, cv::Scalar(255, 255, 255), 1, 8);

        for (auto it = contours_.begin(); it != contours_.end();)
        {
            double minArea=200;
            double area = contourArea(*it);
            if (area < minArea)
            {
                it = contours_.erase(it);
            }
            else
            {
                ++it;
            }
        }

        // 确保已完整识别到能量机关(轮廓最小个数为2)
        if (contours_.size() > 1)
        {
            std::sort(contours_.begin(), contours_.end(), compare_contours_len); // 根据轮廓长度对轮廓进行排序
            std::cout<<contours_.size()<<"\n";
            for (const auto &contours : contours_)
            {
                cv::Point2f center;
                float radius;

                minEnclosingCircle(contours, center, radius);
                cv::Point3f temp = cv::Point3f(center.x, center.y, radius);
                armor_data.push_back(temp);
            }
            // 根据半径大小对识别的轮廓进行排序。（半径最小为R标，半径第二小为待击打装甲板）
            std::sort(armor_data.begin(), armor_data.end(), compare_radius);

            cv::circle(black_img, cv::Point(armor_data[0].x, armor_data[0].y), 5, cv::Scalar(255, 255, 255), 1, 8, 0); // 画出能量机关的中心坐标
            cv::circle(black_img, cv::Point(armor_data[1].x, armor_data[1].y), 5, cv::Scalar(255, 255, 255), 1, 8, 0); // 画出待击打装甲板的中心坐标

            this->energy_organ_center.x = armor_data[0].x;
            this->energy_organ_center.y = armor_data[0].y;
            this->armor_center.x = armor_data[1].x;
            this->armor_center.y = armor_data[1].y;

            cv::RotatedRect rotrect;
            cv::Point2f rect_points[4]; // point[0]左下:x最小 point[1]左上:y最小 point[2]右上:x最大 point[3]右下:y最大
            cv::Mat boxPoints2f, boxPointsCov;

            // 获取最小外接矩形点阵
            rotrect = cv::minAreaRect(contours_[1]);
            rotrect.points(rect_points);
            cv::boxPoints(rotrect, boxPoints2f);
            boxPoints2f.assignTo(boxPointsCov, CV_32S);

            // 绘制旋转矩形及其顶点
            cv::polylines(black_img, boxPointsCov, true, cv::Scalar(255, 255, 255), 2);
            cv::circle(black_img, rect_points[0], 5, cv::Scalar(255, 255, 255), 1, 8, 0);

            std::vector<cv::Point2f> imagePoint_temp;
            imagePoint_temp.push_back(rect_points[0]); // 左下
            imagePoint_temp.push_back(rect_points[1]); // 左上
            imagePoint_temp.push_back(rect_points[2]); // 右上
            imagePoint_temp.push_back(rect_points[3]); // 右下
            this->imagePoints_ = imagePoint_temp;
            cv::imshow("2", black_img);

            cv::Vec4f line(this->energy_organ_center.x, this->energy_organ_center.y, this->armor_center.x, this->armor_center.y);
            cv::Point point(this->energy_organ_center.x, this->energy_organ_center.y);
            cv::Point point_(this->armor_center.x, this->armor_center.y);
            double angle = GetLineAngle(line);
            this->angle_Energy_organ_center2armor_center = angle;
        }
    }

    void AngleSolve()
    {
        if (this->angle_Energy_organ_center2armor_center >= 0 && this->angle_Energy_organ_center2armor_center < 90)
        {
            this->imagePoints.push_back(this->imagePoints_[0]);
            this->imagePoints.push_back(this->imagePoints_[3]);
            this->imagePoints.push_back(this->imagePoints_[1]);
            this->imagePoints.push_back(this->imagePoints_[2]);
        }
        else if (this->angle_Energy_organ_center2armor_center >= 90 && this->angle_Energy_organ_center2armor_center < 180)
        {
            this->imagePoints.push_back(this->imagePoints_[1]);
            this->imagePoints.push_back(this->imagePoints_[0]);
            this->imagePoints.push_back(this->imagePoints_[2]);
            this->imagePoints.push_back(this->imagePoints_[3]);
        }

        else if (this->angle_Energy_organ_center2armor_center >= 180 && this->angle_Energy_organ_center2armor_center < 270)
        {
            this->imagePoints.push_back(this->imagePoints_[2]);
            this->imagePoints.push_back(this->imagePoints_[1]);
            this->imagePoints.push_back(this->imagePoints_[3]);
            this->imagePoints.push_back(this->imagePoints_[0]);
        }

        else if (this->angle_Energy_organ_center2armor_center >= 270 && this->angle_Energy_organ_center2armor_center <= 360)
        {
            this->imagePoints.push_back(this->imagePoints_[3]);
            this->imagePoints.push_back(this->imagePoints_[2]);
            this->imagePoints.push_back(this->imagePoints_[0]);
            this->imagePoints.push_back(this->imagePoints_[1]);
        }
    }

    /**
     * @brief
     *
     * @param img
     * @param energy_organ_center
     * @param armor_center
     * @param radius_armor_center
     * @return cv::Point2f
     */
    cv::Point2f Predict()
    {
        double roi_x = this->energy_organ_center.x - this->radius_energy_organ;
        double roi_y = this->energy_organ_center.y - this->radius_energy_organ;
        double width = 2 * this->radius_energy_organ;
        double height = 2 * this->radius_energy_organ;
        cv::Rect buff_roi(roi_x, roi_y, width, height);

        this->armor_roi_position.x = this->armor_center.x - roi_x;
        this->armor_roi_position.y = this->armor_center.y - roi_y;
    }
};

#endif // BUFFDETECT_BUFFDETECT_H_