#ifndef GENERAL_CUSTOM_DATA_SERIAL_DATA_HPP_
#define GENERAL_CUSTOM_DATA_SERIAL_DATA_HPP_

#include "detector_data.hpp"

// 字节数为4的联合体
typedef union
{
    float f;
    unsigned char c[4];
} FloatUChar4;

// 字节数为2的uchar数据类型
typedef union
{
    int16_t d;
    unsigned char c[2];
} Int16UChar2;

// 用于保存目标相关角度和距离信息及瞄准情况
typedef struct
{
    FloatUChar4 pitch;    // 俯仰角
    FloatUChar4 yaw;      // 偏航角
    FloatUChar4 distance; // 目标距离
    // 设置1表示目标进入了可以开火的范围，
    // 设置0则表示目标尚未进入可开火的范围，目前暂不使用，默认置0
    bool is_middle;
    bool is_find_target; // 当识别的图片范围内有目标且电控发来的信号不为0x00（关闭视觉）置为1，否则置0
    bool is_find_buff;
} VisionData;

struct Stm32Data // 接受数据的结构体
{
    unsigned char IsHave = false; // 是否有数据
    uint mode=0;                   // 0 击打红色装甲板  1 击打蓝色装甲板  2 击打红色能量机关  3 击打蓝色能量机关
    float speed;
    Imu imu;
};

#endif //! GENERAL_CUSTOM_DATA_SERIAL_DATA_HPP_
