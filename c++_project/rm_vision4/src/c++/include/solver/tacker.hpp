// /**
//  * @file tacker.hpp
//  * @author luoyebai (2112216825@qq.com)
//  * @brief 一个简易的追踪器
//  * @version 0.1
//  * @date 2023-04-14
//  *
//  * @copyright Copyright (c) 2023
//  *
//  */

// //! 未完成
// // #TODO 车辆追踪器编写(包括小陀螺)

// #ifndef SOLVER_TACKER_HPP_
// #define SOLVER_TACKER_HPP_

// // std
// #include <memory>

// // eigen
// #include <Eigen/Eigen>

// // general
// #include "message/message.hpp"

// // solver
// #include "predictor_adaptive_ekf.hpp"

// /**
//  * @brief 追踪器
//  *
//  */
// class Tracker
// {
// public:
//     int tracked_id;
//     enum State
//     {
//         LOST,
//         DETECTING,
//         TRACKING,
//         TEMP_LOST,
//     } tracker_state_;
//     PredictorAdaptiveEKF ekf;
//     Detection_pack tracked_armor;
//     Eigen::VectorXd target_state;

//     // To store another pair of armors message
//     double last_z, last_r;

//     /**
//      * @brief Construct a new Tracker object
//      *
//      * @param max_match_distance    匹配距离
//      * @param tracking_threshold    追踪的阈值
//      * @param lost_threshold        丢失的阈值
//      */
//     Tracker(double max_match_distance, int tracking_threshold, int lost_threshold);

//     /**
//      * @brief tracker初始化
//      *
//      * @param armor_data
//      */
//     void Init(const Detection_pack &armor_data);

//     /**
//      * @brief 更新状态
//      *
//      * @param armor_data
//      */
//     void Update(const Detection_pack &armor_data);

// private:
// };

// #endif //! SOLVER_TACKER_HPP_