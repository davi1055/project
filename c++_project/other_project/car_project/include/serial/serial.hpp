/**
 * @file serial.hpp
 * @author luoyebai (2112216825@qq.com),davi(1224802565@qq.com)
 * @brief 这份是我抄学长的懒得改了
 * 传数据的话在第400行左右传给vdata
 * data_file 的 data文件里有个serial_port文件记得复制
 * @version 0.1
 * @date 2023-03-10
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef _SERIAL_HPP_
#define _SERIAL_HPP_

// serial
#include "serial_port.hpp"

/**
 * @brief 数据发送给串口
 *
 * @param serial_port 串口号
 * @param required_stop 是否停止
 */
void robot_cmd_loop(SerialPort &serial_port,const bool &required_stop,VisionData vision_data)
{
    time_t start_time;
    time_t end_time;
    uint32_t frame_count = 0;
    // 订阅解算结果
    //Subscriber<VisionData> robot_cmd_sub("robot_cmd",0); // robocmd 放在general里面
    while (!required_stop)
    {

        if (!frame_count)
        {
            time(&start_time);
        }

        //auto vdata = robot_cmd_sub.pop();
        serial_port.TransformData(vision_data);
        // send_port.TransformData(VisionData{0.0,0.0,0.0,0,2,0});
        // //视觉可以发送手拟的数据给电控，查看串口是不是通的
        // std::this_thread::sleep_for(milliseconds(6));

        // auto vdata = VisionData{0};
        // auto vdata = robot_cmd_sub.pop();
        serial_port.Send();

        frame_count++;
        time(&end_time);
        if (end_time - start_time >= 1)
        {   
           printf("<SerialSend: FrameCount: %u>\n\n", frame_count);
            // std::cout << std::endl;
     
            frame_count = 0;
        }
    }
}

/**
 * @brief 数据接受&数据发送
 *
 * @return true
 * @return false
 */
bool robot_io_usb()
{
    SerialPort send_port;
    // 接收
    // SerialPort receive_port;
    send_port.OpenPort();

    bool robot_cmd_required_stop = false;
    bool robot_cmd_is_ok = true;
    // auto seq_data = std::queue<Stm32Data>();

    // 这是发送数据给电控的线程
    // std::thread robot_cmd_thread(robot_cmd_loop, std::ref(send_port),
    //                              std::ref(robot_cmd_required_stop));
    // robot_cmd_thread.detach();

    // 传给图像采集
    //Publisher<Stm32Data> seq_data("stm32_data");

    time_t start_time;
    time_t end_time;
    uint32_t frame_count = 0;

    while (robot_cmd_is_ok)
    {

        if (!frame_count)
        {
            time(&start_time);
        }

        //Stm32Data sdata;
        //sdata = send_port.Receive(); // 收到电控的数据(包括 IsHave mode color speed imu)
        // if (sdata.IsHave)
        // {
        //     std::cout<<"color": <<sdata.color <<" ";
        //     std::cout << "mode: " << sdata.mode << " ";
        //     std::cout << "speed: " << sdata.speed << " ";
        //     std::cout << "pitch: " << sdata.imu.pitch << " ";
        //     std::cout << "yaw: " << sdata.imu.yaw << " ";
        //     std::cout << "\n\n";
        // }
        //seq_data.push(sdata);

        // if (sdata.IsHave)
        //     frame_count++;

        time(&end_time);
        if (end_time - start_time >= 1)
        {
                if(frame_count ==0)
            {
                puts("can't receive serial" );
                //exit(0);
            }   
            printf("<Serial: FrameCount: %u>\n\n", frame_count);
            frame_count = 0;
        }
    }

    robot_cmd_required_stop = true;
    // robot_cmd_thread.join();
    return false;
}

/**
 * @brief 双向通讯线程
 *
 */
void SerialTalking()
{
    while (!robot_io_usb())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
}

#endif //! _SERIAL_HPP_
