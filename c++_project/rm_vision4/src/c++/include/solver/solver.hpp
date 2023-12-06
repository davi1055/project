/**
 * @file solver.hpp
 * @author donghao
 * @brief
 * @version 0.1
 * @date 2023-04-14
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef _SOLVER_HPP_
#define _SOLVER_HPP_

// solver
#include "armor_solver.hpp"
#include "tacker.hpp"

/**
 * @brief 角度解算线程
 *
 */
void AngleSolving()
{
    time_t start_time;
    time_t end_time;
    uint32_t frame_count = 0;
    float lasttime = 0.0f;
    // std::this_thread::sleep_for(milliseconds(3000));                 //可以去掉
    Subscriber<Detection_pack> armor_sub("armor_set");
    Publisher<VisionData> robot_cmd_pub("robot_cmd");

#ifdef _DEBUG_
    Publisher<SolverDebugData> debug_pub("solver_debug");
#endif //!_DEBUG_-

    AngleSolver angleSolver;
    angleSolver.setCameraParam("../src/data_file/data/init/camera_params.xml", 2); // 设置相机内参和畸变参数 (3号相机暂时弃用)
    angleSolver.setArmorSize(SMALL, 60, 135);                                     // 输入小装甲版参数  现在默认是小装甲板
    angleSolver.setArmorSize(LARGE, 113, 230);                                     // 输入大装甲版参数
    // PredictorKalman kalman;
    PredictorAdaptiveEKF predictor;
    while (true)
    {
        if (!frame_count)
        {
            time(&start_time);
        }

        double x = 0, y = 0, z = 0, yaw = 0, pitch = 0, distance = 0;
        auto detections = armor_sub.pop();

        float t = detections.timestamp;
        lasttime = t;

        Imu q = detections.imu;
        // q.roll=0;
        // q.yaw=0;
        // q.pitch=0;

        bool findTarget = detections.findArmor; // 是否检测到装甲板

        if (findTarget)
        {
            angleSolver.setBulletSpeed(detections.speed); // 设置子弹射速 m/s
            angleSolver.getAngle(detections, x, y, z, yaw, pitch, distance);
            // 只有小装甲板
            // std::cout << "yaw:" << yaw << " pitch:" << pitch << " distance:" << distance << "\n";
            // cv::Point2f predictPoint = kalman.estimate(x,y,z,yaw,pitch,distance,t);     //kf
            // ekf 返回预测点的二维坐标

            cv::Point2f predictPoint = predictor.estimate(x, y, z, yaw, pitch, distance, q, t) ;
            //std::cout<<detections.centerPoint<<"\n";
            //std::cout<< predictPoint<<"\n";
            cv::Mat test=detections.img.clone();
            //std::cout<<test.size()<<"\n";
            cv::circle(test,detections.centerPoint,2,cv::Scalar(255,255,255),1,8,0);
            // std::cout << yaw << "\n";
            // if ((predictPoint == cv::Point2f(0, 0)))
            // findTarget = false;
            // cv::Point2f predictPoint(0, 0);
            //  angleSolver.addPredictPoint(predictPoint);
#ifdef _DEBUG_
            debug_pub.push(SolverDebugData{predictPoint,
                                           Solver_PYD{pitch, yaw, distance},
                                           Solver_XYZ{x, y, z}});
#endif //!_DEBUG_

            // std::cout << "preyaw:" << yaw << " prepitch:" << pitch << " predistance:" << distance << "\n";
            // 预测了就暂停t s
            // std::this_thread::sleep_for(std::chrono::microseconds((int)t));
        }
        // else
        // {
        //     predictor.clear();
        // }

        // angleSolver.showDebugInfo(1, 0, 0, 0, 0, 0);
        bool is_middle = (abs(pitch) < 0.01) && (abs(yaw) < 0.1) && findTarget;

        robot_cmd_pub.push(VisionData{(float)pitch, (float)yaw, (float)distance, is_middle, findTarget, 0});

        frame_count++;
        time(&end_time);
        if (end_time - start_time >= 1)
        {
            printf("<AngleSolver: FrameCount: %u>\n\n", frame_count);
            frame_count = 0;
        }
        //         cv::waitKey(1);
    }
}

#endif //! _SOLVER_HPP_
