/**
 * @file image_updating.hpp
 * @author donghao,luoyebai (2112216825@qq.com)
 * @brief 修改了学长部分代码
 * @version 0.1
 * @date 2023-04-05
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef _IMAGE_UPDATING_HPP_
#define _IMAGE_UPDATING_HPP_

// general
#include "message/message.hpp"

#ifndef _USING_VIDEO_
// image_updating
#include "GxCamera/GxCamera.h"
#endif

/**
 * @brief 主要进行摄像头的数据收发
 *
 * @return true
 * @return false
 */
bool CameraIO()
{
    time_t start_time;
    time_t end_time;
    uint32_t frame_count = 0;

    // float lastImuTime;
    timeval tv;

    Publisher<SensorsData> data_pub("sensors_data");
    Subscriber<Stm32Data> seq_data("stm32_data");

#ifdef _USING_VIDEO_
    //cv::VideoCapture capture("/home/davi/project/BuffDetect/blue_buff.mp4");
    cv::VideoCapture capture(0);
    // 30帧
    //capture.set(cv::CAP_PROP_FPS, 60);
#else
    GxCamera camera;
    camera.initial();               // 设置曝光、白平衡、触发模式等参数
    camera.openDevice(false, "SN"); // 打开设备 false意味着打开第一个接入的设备
#endif
    // double t, t1;

    while (1)
    {
        if (!frame_count)
        {
            time(&start_time);
        }

        cv::Mat src;
        float timestamp;

#ifdef _USING_VIDEO_
        capture >> src;
        cv::resize(src,src,cv::Size(1920,1080));
        if(cv::waitKey(20)>5)
            break;
        // std::this_thread::sleep_for(std::chrono::milliseconds(50)); // 等待50ms让图片采集的慢一些
#else
        if (!camera.read(&src, &timestamp))
        {                   // 采集一张图片
            camera.close(); // 如果无法读取图片，就重启一边相机驱动（插拔情况除外）
            break;
        }
#endif

        gettimeofday(&tv, NULL);
        // float timeImgae = tv.tv_usec / 1000;

        Stm32Data stmData = seq_data.pop();

        // std::cout << "yaw: " << stmData.imu.yaw/3.14*180 <<" pitch: " << stmData.imu.pitch/3.14*180 << std::endl;

        // 判断收到的imu是否更新（串口收到imu的系统时间） 一张图片对应一阵imu的数据，避免两张图片对应同一帧imu
        //  while(stmData.imu.time == lastImuTime)
        //{
        //     stmData = seq_data.pop();
        // }

        // data_pub.push({src, stmData.imu, stmData.speed, stmData.mode, timestamp});
        data_pub.push({src, stmData.imu, stmData.speed ,stmData.mode, timestamp});

        frame_count++;
        time(&end_time);
        if (end_time - start_time >= 1)
        {
            printf("<ImageUpdating: FrameCount: %u>\n", frame_count);
            frame_count = 0;
        }
    }
    return false;
}

/**
 * @brief 图像数据更新
 *
 */
void ImageUpdating()
{

    while (!CameraIO())
    {
        // std::cout << "camera reopen\n";
        puts("camera reopen\n");
        std::this_thread::sleep_for(std::chrono::microseconds(500)); // 等待0.5s
    }
}

#endif // !_IMAGE_UPDATING_HPP_