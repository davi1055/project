/**
 * @file record.hpp
 * @author donghao,luoyebai (2112216825@qq.com)
 * @brief 修改了部分学长代码
 * @version 0.1
 * @date 2023-04-14
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef _RECORD_HPP_
#define _RECORD_HPP_

// std
#include <ctime>
#include <thread>
#include <chrono>
#include <iostream>

// opencv
#include <opencv2/opencv.hpp>

// general
#include "message/message.hpp"

using namespace std::chrono;

/**
 * @brief 录像
 *
 * @param storage_location 录像文件夹路径
 */
void video_record(const std::string &record_dir = "../src/data_file/data/record/")
{
    // 用于比赛录制视频
    char now[64];
    std::time_t tt;
    struct tm *ttime;
    tt = time(nullptr);
    ttime = localtime(&tt);
    strftime(now, 64, "%Y-%m-%d_%H_%M_%S", ttime);
    std::string now_string(now);
    std::string path(std::string(record_dir + now_string).append(".avi"));
    try
    {
        auto writer = cv::VideoWriter(path, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 20.0, cv::Size(WIDTH * 2, HEIGHT * 2));
        if (!writer.isOpened())
        {
            std::cerr << "can't open file" << std::endl;
            return;
        }
        Subscriber<SensorsData> data_sub("sensors_data");
        while (true)
        {
            try
            {
                const auto &[image, imu, speed, mode , timestamp] = data_sub.pop();
                writer.write(image);
            }
            catch (MessageError &e)
            {
                std::this_thread::sleep_for(milliseconds(3000));
            }
        }
    }
    catch (...)
    {
        std::cout << "failed to record data" << std::endl;
        return;
    }
}

/**
 * @brief 录像的线程
 *
 */
void Recording()
{
    std::cerr << "============video_writer===========\n";
    std::thread([=]()
                { video_record(); })
        .detach();
}

#endif // !_RECORD_HPP_