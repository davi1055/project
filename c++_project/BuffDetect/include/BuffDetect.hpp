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
    //**************参数区START**************//
    uint color; // RED==2,BLUE==3
    double minArea;
    //**************参数区END****************//

    const std::vector<cv::Point3f> objectPoints = {{-165, 183, 0}, {-165, 49, 0}, {165, 183, 0}, {165, 49, 0}}; // 物体世界坐标系的坐标点(左上，左下，右上，右下)
    std::vector<cv::Point2f> imagePoints_;                                                                       // 物体在像素坐标系上的坐标点(左下，左上,右上，右下)
    std::vector<cv::Point2f> imagePoints;                                                                        // 物体在像素坐标系上的不变坐标点(左上，左下,右上，右下)
    cv::Mat rvec;                                                                                                // 旋转向量
    cv::Mat tvec;                                                                                                // 平移向量
    float radius_armor_center;                                                                                   // 到装甲板中心的半径(当前)
    float radius_energy_organ;                                                                                   // 大扇页的半径(当前)
    cv::Point2f energy_organ_center;                                                                             // 能量机关圆心坐标
    cv::Point2f armor_center;                                                                                    // 待击打装甲板中心坐标

    double angle_Energy_organ_center2armor_center; // 能量机关中心只指向装甲板中心的向量的角度
    cv::Mat processed_img;                         // 经过ImageProcess()函数处理过得图像

public:
    // predict
    using PointsData = std::vector<std::vector<cv::Point2f>>;
    PointsData judge_points;                 // 判断大小能量机关所需的数据点
    std::vector<cv::Point2f> predict_points; // 预测用的点数据
    double FPS;
    float small_energy = 20 * 3.14159265358979323846 / 60; // 小能量机关角速度(rad/s)
    
    cv::Point2f GetArmorCenter()
    {
        return armor_center;
    }

    std::vector<cv::Point2f> GetImagePoints()
    {
        return imagePoints;
    }

    bool IfFindBuff()
    {
        return this->armor_center.x == -1 && this->armor_center.y == -1 ? false : true;
    }

    Buff(uint init_color, double init_minArea)
    {
        color = init_color;
        minArea = init_minArea;
    }

    /**
     * @brief  图像处理,返回扇页轮廓
     *
     * @param src 原图像
     * @param color 需要筛选的颜色
     * @return cv::Mat
     */
    void ImageProcess(cv::Mat src)
    {
        // color = this->color;
        std::vector<cv::Mat> channels;
        cv::Mat mat;
        cv::Mat channel_img;
        cv::Mat threshold_img;
        cv::Mat black_img = cv::Mat::zeros(src.size(), CV_8UC1);
        // 三通道分离
        cv::split(src, channels);

        // 减去干扰色
        if (color == 2) // 红色 2
        {
            cv::subtract(channels[2], channels[0], channel_img);
        }
        else if (color == 3) // 蓝色 3
        {
            cv::subtract(channels[0], channels[2], channel_img);
        }

        // 选择目标颜色进行二值化
        cv::threshold(channel_img, threshold_img, 100, 255, cv::THRESH_BINARY);
        // 开操作
        cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        cv::morphologyEx(threshold_img, mat, cv::MORPH_OPEN, element);
        // 寻找轮廓
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(mat, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        for (auto it = contours.begin(); it != contours.end();)
        {
            double area = contourArea(*it);
            if (area < minArea)
            {
                it = contours.erase(it);
            }
            else
            {
                ++it;
            }
        }
        cv::drawContours(black_img, contours, -1, cv::Scalar(255, 255, 255), 3, 4);
        // 凸包检测
        std::vector<std::vector<cv::Point>> hull(contours.size());
        for (int i = 0; i < contours.size(); i++)
        {
            // 寻找凸包
            cv::convexHull(cv::Mat(contours[i]), hull[i], false);
        }
        for (int i = 0; i < contours.size(); i++)
        {
            cv::fillConvexPoly(black_img, hull[i], cv::Scalar(255, 255, 255), cv::LINE_AA, 0);
        }
        std::vector<std::vector<cv::Point>> contours_;
        std::vector<cv::Vec4i> hierarchy_;
        cv::findContours(black_img, contours_, hierarchy_, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        cv::Mat result_img = cv::Mat::zeros(src.size(), CV_8UC3);
        cv::drawContours(result_img, contours_, -1, cv::Scalar(255, 255, 255), 3, 4);
        processed_img = result_img; // 将处理过的图像传回给类实例
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
        cv::Point2f armor_center = cv::Point2f(-1, -1);
        cv::Mat clone_img = processed_img.clone();
        // cv::cvtColor(processed_img, processed_img, cv::COLOR_BGR2GRAY);
        cv::cvtColor(clone_img, clone_img, cv::COLOR_BGR2GRAY);
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(clone_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        if (!contours.empty())
        {
            std::sort(contours.begin(), contours.end(), compare_contours_len); // 根据轮廓长度对轮廓进行排序
            // 画出所有检测到的轮廓，并根据半径筛选出半径最小的两个半径相同的外接圆,最小的两个外接圆的圆心的中点就是装甲板中点
            for (const auto &contour : contours)
            {
                cv::Point2f center;
                float radius;
                minEnclosingCircle(contour, center, radius);
                cv::Point3f temp = cv::Point3f(center.x, center.y, radius);
                cv::circle(clone_img, center, radius, cv::Scalar(255, 255, 255), 1);
                armor_data.push_back(temp);
            }
            // std::cout<<contours_len[0]<<" "<<contours_len[1]<<" "<<contours_len[2]<<" "<<contours_len[3]<<"\n";
            std::sort(armor_data.begin(), armor_data.end(), compare_radius);
            // cv::Mat test = cv::Mat::zeros(img.size(), CV_8UC3);
            // cv::drawContours(test,contours,0, cv::Scalar(255, 255, 255), 3, 4);
            // this->img_debug=test;
            cv::RotatedRect rotrect;
            cv::Point2f rect_points[4]; // point[0]左下:x最小 point[1]左上:y最小 point[2]右上:x最大 point[3]右下:y最大
            cv::Mat boxPoints2f, boxPointsCov;

            // 获取最小外接矩形点阵
            rotrect = cv::minAreaRect(contours[0]);
            rotrect.points(rect_points);
            cv::boxPoints(rotrect, boxPoints2f);
            boxPoints2f.assignTo(boxPointsCov, CV_32S);

            // 绘制旋转矩形及其顶点
            cv::Mat test = cv::Mat::zeros(clone_img.size(), CV_8UC3);
            cv::polylines(clone_img, boxPointsCov, true, cv::Scalar(255, 255, 255), 2);
            cv::circle(clone_img, rect_points[0], 5, cv::Scalar(255, 255, 255), 1, 8, 0);

            std::vector<cv::Point2f> imagePoint_temp;
            imagePoint_temp.push_back(rect_points[0]); // 左下
            imagePoint_temp.push_back(rect_points[1]); // 左上
            imagePoint_temp.push_back(rect_points[2]); // 右上
            imagePoint_temp.push_back(rect_points[3]); // 右下
            this->imagePoints_ = imagePoint_temp;

        }
        // 如果识别到能量机关
        if (!armor_data.empty())
        {
            armor_center.x = (armor_data[0].x + armor_data[1].x) / 2;
            armor_center.y = (armor_data[0].y + armor_data[1].y) / 2;

            // 画出能量机关中心
            cv::Point2f energy_organ_center;
            float radius_energy_organ;
            float radius_armor_center;
            std::vector<cv::Point> all_points;
            // 对所有(大)轮廓进行合并
            for (const auto &contour : contours)
            {
                all_points.insert(all_points.end(), contour.begin(), contour.end());
            }
            minEnclosingCircle(all_points, energy_organ_center, radius_energy_organ);
            circle(clone_img, energy_organ_center, radius_energy_organ, cv::Scalar(255, 255, 255), 2); // 绘制最小外接圆
            circle(clone_img, energy_organ_center, 5, cv::Scalar(255, 255, 255), 1, 8, 0);
            radius_armor_center = cv::norm(energy_organ_center - armor_center); // energy_organ_center到armor_center的距离为半径

            this->radius_energy_organ = radius_energy_organ;
            this->radius_armor_center = radius_armor_center;
            this->energy_organ_center = energy_organ_center;
            this->armor_center = armor_center;

            cv::Vec4f line(this->energy_organ_center.x, this->energy_organ_center.y, this->armor_center.x, this->armor_center.y);
            cv::Point point(this->energy_organ_center.x, this->energy_organ_center.y);
            cv::Point point_(this->armor_center.x, this->armor_center.y);
            double angle = GetLineAngle(line);
            this->angle_Energy_organ_center2armor_center = angle;

            cv::circle(clone_img, armor_center, 5, cv::Scalar(255, 255, 255), 1, 8, 0); // 画出装甲板中心
        }
        if (armor_data.empty())
        {
            this->armor_center = armor_center;
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
    // cv::Point2f Predict(cv::Mat img, cv::Point2f energy_organ_center, cv::Point2f armor_center, float radius_armor_center)
    // {
    //     // 先对识别到的能量机关用roi进行持续跟随并获取能量机关参数
    //     cv::RotatedRect roi_rect(this->energy_organ_center, cv::Size2f(this->radius_energy_organ * 2, this->radius_energy_organ * 2), 0);
    //     cv::Rect roi = roi_rect.boundingRect();
    //     cv::rectangle(img, roi, cv::Scalar(255, 255, 255), 1, 8, 0);
    //     cv::namedWindow("src");
    //     cv::imshow("src", img);

    //     // 首先检测能量机关的线速度和角速度(在ROI区域中计算),并判断大小能量机关
    //     cv::Point2f point_roi = this->armor_center - cv::Point2f(roi.x, roi.y); // 转换为在roi区域的坐标

    //     armor_center_last.resize(200, cv::Point2f(0, 0)); // 初始化2s的数据量，每个point先初始化为(0,0)
    //     std::vector<cv::Point2f>::iterator k = this->armor_center_last.begin();
    //     armor_center_last.erase(k);             // 删除头元素
    //     armor_center_last.push_back(point_roi); // 在尾部插入新元素
    //     // 检查转换后的坐标是否在 ROI 区域内
    //     if (roi.contains(point_roi))
    //     {
    //         std::cout << "Point in ROI: (" << point_roi.x << ", " << point_roi.y << ")"
    //                   << "\n";
    //     }

    //     std::cout << "point in src:" << this->armor_center << "\n";
    //     for (const auto &x : armor_center_last)
    //     {
    //         std::cout << x << "\n";
    //     }
    //     std::vector<cv::Point2f> temp;
    //     temp.push_back(energy_organ_center);
    //     temp.push_back(armor_center);
    //     if (this->judge_points.size() > 300)
    //         this->judge_points.erase(this->judge_points.begin()); // 删除头元素
    //     this->judge_points.push_back(temp);                       // 在尾部插入新元素
    //     // end为失效的迭代器
    //     //  std::cout<<(--this->judge_points.end())->at(0)<<" ";
    //     //  std::cout<<(--this->judge_points.end())->at(1)<<"\n";
    //     bool start_predict = false;
    //     // 判断能否开始预测
    //     if (this->judge_points.size() >= 300)
    //     {
    //         start_predict = true;
    //     }

    //     std::cout << "predict status:" << start_predict << "\n";

    //     if (start_predict)
    //     {
    //     }

    //     return cv::Point2f(0, 0);
    // }

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
};

#endif // BUFFDETECT_BUFFDETECT_H_