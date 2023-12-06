#include<iostream>
#include"../include/BuffDetect_new.hpp"
#include "../general/GxCamera/GxCamera.h"       // import Galaxy Camera

#define USING_VEDIO

int main()
{
    Buff buff(2);
    #ifdef USING_VEDIO
        cv::VideoCapture capture("../12mm_red_dark.mp4");
    #else
        GxCamera camera;
        camera.initial();                   //设置曝光、白平衡、触发模式等参数
        camera.openDevice(false, "SN");     //打开设备 false意味着打开第一个接入的设备
    #endif
    while(true)
    {
        
        double start_time = cv::getTickCount();//开始计时（计算FPS值）
        cv::Mat src;
        float timestamp;
        
        #ifdef USING_VEDIO
        
            capture>>src;

        #else
            //采集一张图片
            if(!camera.read(&src, &timestamp))
            {             
                camera.close();                            //如果无法读取图片，就重启一边相机驱动（插拔情况除外）
                break;
            }
        #endif
        #ifdef USING_VEDIO
        if(src.data==nullptr)
        {
            main();
        }
        #endif
        buff.ImageProcess(src);
        buff.FindCenter();
        //cv::Point2f angle=buff.AngleSolve();
        //cv::Mat img_keypoint=buff.FilterKeypoints(threshold_img);
        //buff.Predict(threshold_img,buff.energy_organ_center,buff.armor_center,buff.radius_armor_center);

        //计算FPS值
        double end_time = cv::getTickCount();//结束计时
        double elapsed_time = (end_time - start_time) / cv::getTickFrequency();
        double FPS=1/elapsed_time;
        buff.FPS=FPS;
        //std::cout<<"FPS= "<<buff.FPS<<"\n";

        //cv::imshow("src", src);

        if(cv::waitKey(1)==27)
            return 0;
        
    }
}