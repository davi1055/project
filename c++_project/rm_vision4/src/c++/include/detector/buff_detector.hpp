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
    //**************参数区END****************//

    const std::vector<cv::Point3f> objectPoints = {{-186, 183, 0}, {-186, -190, 0}, {186, 183, 0}, {186, -190, 0}}; // 物体世界坐标系的坐标点(左上，左下，右上，右下)
    std::vector<cv::Point2f> imagePoints_;                                                                      // 物体在像素坐标系上的坐标点(左下，左上,右上，右下)
    std::vector<cv::Point2f> imagePoints;                                                                       // 物体在像素坐标系上的不变坐标点(左上，左下,右上，右下)
    cv::Mat rvec;                                                                                               // 旋转向量
    cv::Mat tvec;                                                                                               // 平移向量
    float radius_armor_center=cv::norm(Buff::energy_organ_center-Buff::armor_center);                           // 能量机关中心到装甲板中心的半径
    float radius_energy_organ;                                                                                // 大扇页的半径(当前)
    cv::Point2f energy_organ_center;                                                                            // 能量机关圆心坐标
    cv::Point2f armor_center=cv::Point2f(-1,-1);                                                                                   // 待击打装甲板中心坐标

    double angle_Energy_organ_center2armor_center; // 能量机关中心只指向装甲板中心的向量的角度
    cv::Mat src;
    cv::Mat processed_img;                         // 经过ImageProcess()函数处理过得图像
    bool if_find_contours=false;                         //是否找到轮廓

public:
    // predict
    using PointsData = std::vector<std::vector<cv::Point2f>>;
    PointsData judge_points;                 // 判断大小能量机关所需的数据点
    std::vector<cv::Point2f> predict_points; // 预测用的点数据
    double FPS;
    float small_energy = 20 * CV_PI / 60; // 小能量机关角速度(rad/s)

    cv::Point2f GetArmorCenter()
    {
        return armor_center;
    }

    std::vector<cv::Point2f> GetImagePoints()
    {
        return imagePoints;
    }

    //是否找到待击打装甲板中心
    bool IfFindBuff()
    {
        return Buff::armor_center.x == -1 && Buff::armor_center.y == -1 ? false : true;
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
        Buff::src=src;
        std::vector<cv::Mat> channels;
        cv::Mat channel_img;
        cv::Mat threshold_img;
        cv::Mat test = cv::Mat::zeros(src.size(), CV_8UC3);
        cv::split(src, channels);
        if (Buff::color == 2)
        {
            cv::subtract(channels[2], channels[0], channel_img); // 识别红
            cv::subtract(channel_img, channels[1], channel_img);
            cv::threshold(channel_img, threshold_img, 200, 255, cv::THRESH_BINARY);
        }
        if (Buff::color == 3)
        {
            cv::subtract(channels[0], channels[2], channel_img); // 识别蓝
            cv::subtract(channel_img,channels[1],channel_img);
            cv::threshold(channel_img, threshold_img, 200, 255, cv::THRESH_BINARY);
        }

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(threshold_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        if(contours.size()>1)
        {
            Buff::if_find_contours=true;
        }
        Buff::processed_img = threshold_img;
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

        // 开操作
        cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2));
        cv::morphologyEx(Buff::processed_img, Buff::processed_img, cv::MORPH_OPEN, element);

        cv::Mat element_ = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(10,10));
        cv::morphologyEx(Buff::processed_img,Buff::processed_img,cv::MORPH_CLOSE,element_);
        // cv::GaussianBlur(Buff::processed_img,Buff::processed_img,cv::Size(1,1),0);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(Buff::processed_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // 凸包检测
        std::vector<std::vector<cv::Point>> hull(contours.size());
        for (int i = 0; i < contours.size(); i++)
        {
            // 寻找凸包
            cv::convexHull(cv::Mat(contours[i]), hull[i], false);
        }
        for (int i = 0; i < contours.size(); i++)
        {
            cv::fillConvexPoly(Buff::processed_img, hull[i], cv::Scalar(255, 255, 255), cv::LINE_AA, 0);
        }

        std::vector<std::vector<cv::Point>> contours_;
        cv::findContours(Buff::processed_img, contours_, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        cv::Mat black_img = cv::Mat::zeros(Buff::processed_img.size(), CV_8UC1);
        cv::drawContours(black_img, contours_, -1, cv::Scalar(255, 255, 255), 1, 8);
        
        //确保已完整识别到能量机关(轮廓最小个数为2)
        if (contours_.size()>1)
        {
            std::sort(contours_.begin(), contours_.end(), compare_contours_len); // 根据轮廓长度对轮廓进行排序
            for (const auto &contours : contours_)
            {
                cv::Point2f center;
                float radius;
                
                minEnclosingCircle(contours, center, radius);
                cv::Point3f temp = cv::Point3f(center.x, center.y, radius);
                cv::circle(black_img, center, radius, cv::Scalar(255, 255, 255), 1);
                armor_data.push_back(temp);
            }
            // 根据半径大小对识别的轮廓进行排序。（半径最小为R标，半径第二小为待击打装甲板）
            std::sort(armor_data.begin(), armor_data.end(), compare_radius);

            cv::circle(black_img,cv::Point(armor_data[0].x,armor_data[0].y),2,cv::Scalar(255,255,255),1,8,0);//画出能量机关的中心坐标
            cv::circle(black_img,cv::Point(armor_data[1].x,armor_data[1].y),2,cv::Scalar(255,255,255),1,8,0);//画出待击打装甲板的中心坐标

            Buff::energy_organ_center.x=armor_data[0].x;
            Buff::energy_organ_center.y=armor_data[0].y;
            Buff::armor_center.x=armor_data[1].x;
            Buff::armor_center.y=armor_data[1].y;

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
            Buff::imagePoints_ = imagePoint_temp;

            cv::Vec4f line(Buff::energy_organ_center.x, Buff::energy_organ_center.y, Buff::armor_center.x, Buff::armor_center.y);
            cv::Point point(Buff::energy_organ_center.x, Buff::energy_organ_center.y);
            cv::Point point_(Buff::armor_center.x, Buff::armor_center.y);
            double angle = GetLineAngle(line);
            Buff::angle_Energy_organ_center2armor_center = angle;
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
        
    }

    void AngleSolve()
    {
        if (Buff::angle_Energy_organ_center2armor_center >= 0 && Buff::angle_Energy_organ_center2armor_center < 90)
        {
            Buff::imagePoints.push_back(Buff::imagePoints_[0]);
            Buff::imagePoints.push_back(Buff::imagePoints_[3]);
            Buff::imagePoints.push_back(Buff::imagePoints_[1]);
            Buff::imagePoints.push_back(Buff::imagePoints_[2]);
        }
        else if (Buff::angle_Energy_organ_center2armor_center >= 90 && Buff::angle_Energy_organ_center2armor_center < 180)
        {
            Buff::imagePoints.push_back(Buff::imagePoints_[1]);
            Buff::imagePoints.push_back(Buff::imagePoints_[0]);
            Buff::imagePoints.push_back(Buff::imagePoints_[2]);
            Buff::imagePoints.push_back(Buff::imagePoints_[3]);
        }

        else if (Buff::angle_Energy_organ_center2armor_center >= 180 && Buff::angle_Energy_organ_center2armor_center < 270)
        {
            Buff::imagePoints.push_back(Buff::imagePoints_[2]);
            Buff::imagePoints.push_back(Buff::imagePoints_[1]);
            Buff::imagePoints.push_back(Buff::imagePoints_[3]);
            Buff::imagePoints.push_back(Buff::imagePoints_[0]);
        }

        else if (Buff::angle_Energy_organ_center2armor_center >= 270 && Buff::angle_Energy_organ_center2armor_center <= 360)
        {
            Buff::imagePoints.push_back(Buff::imagePoints_[3]);
            Buff::imagePoints.push_back(Buff::imagePoints_[2]);
            Buff::imagePoints.push_back(Buff::imagePoints_[0]);
            Buff::imagePoints.push_back(Buff::imagePoints_[1]);
        }
    }

    //总处理函数
    void Process(cv::Mat src)
    {
        Buff::ImageProcess(src);
        if(Buff::if_find_contours)
        {
            Buff::FindCenter();
            Buff::AngleSolve();
        }
    }
};

#endif // BUFFDETECT_BUFFDETECT_H_