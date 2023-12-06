/**
 * @file main.cpp
 * @author donghao,luoyebai (2112216825@qq.com)
 * @brief 主函数
 * @version 04.0
 * @date 2023-04-12
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "detector/detector.hpp"
#include "image_updating/image_updating.hpp"
#include "serial/serial.hpp"
#include "solver/solver.hpp"
#include "record/record.hpp"

#ifdef _DEBUG_
#include "debug/debug.hpp"
#endif //! _DEBUG_

int main(int argc, char *argv[])
{

    std::thread(ImageUpdating).detach();     // 图像采集线程
    std::thread(DetectorTargeting).detach(); // 识别
    std::thread(AngleSolving).detach();      // 解算
    std::thread(SerialTalking).detach();     // 串口数据收发
    std::thread(Recording).detach();         // 录像

#ifdef _DEBUG_
    Debuging();
#else
    getchar(); // 阻塞
#endif //! _DEBUG_

    return 0;
}
