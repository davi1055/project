#ifndef _CUSTOM_DATA_HPP_
#define _CUSTOM_DATA_HPP_

#include "serial_data.hpp"

#ifdef _DEBUG_
struct DebugData
{
    int detector_color;
    int detector_threshold;
    float num_threshold;
    float amplify;
};
struct DetectorDebugData
{
    Stm32Data stm32_data;
    cv::Mat detector_image;
    cv::Mat detector_binary_img;
};
struct Solver_PYD
{
    double pitch;
    double yaw;
    double distance;
};
struct Solver_XYZ
{
    double x;
    double y;
    double z;
};

struct SolverDebugData
{
    cv::Point2f predict_point;
    Solver_PYD solver_pyd;
    Solver_XYZ solver_xyz;
    // cv::Mat solver_img;
    // cv::Mat detector_binary_img;
};
#endif //!_DEBUG_

#endif //! _CUSTOM_DATA_HPP_