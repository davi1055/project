#ifndef GENERAL_CUSTOM_DATA_LOWER_COMPUTER_DATA_HPP_
#define GENERAL_CUSTOM_DATA_LOWER_COMPUTER_DATA_HPP_

// opencv
#include <opencv2/opencv.hpp>

#ifndef _DEBUG_
// 乘 amplify
#define WIDTH 640 * 2
#define HEIGHT 512 * 2
// #define HEIGHT 480
#else
int WIDTH = 640;
int HEIGHT = 512;
#endif // _DEBUG_

// 2左右较为适合(160-200帧)
float amplify = 2;

enum Color
{
    BLUE = 0,
    RED = 1,
    PURPLE =2
};

struct Imu
{
    float yaw;
    float roll;
    float pitch;
    float time; // 系统时间
};

struct SensorsData
{               // 传感器数据
    cv::Mat img; // 图像
    Imu imu;
    float speed;
    uint mode;
    float t;
};

#endif //! GENERAL_CUSTOM_DATA_LOWER_COMPUTER_DATA_HPP_
