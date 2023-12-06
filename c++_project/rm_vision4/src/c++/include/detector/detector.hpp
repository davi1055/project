/**
 * @file detector.hpp
 * @author luoyebai (2112216825@qq.com)
 * @brief 无
 * @version 0.1
 * @date 2023-04-14
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef _DETECTOR_HPP_
#define _DETECTOR_HPP_

#include <typeinfo>

// detector
#include "armor_tacker.hpp"
#include "buff_detector.hpp"

#ifdef _DEBUG_
/// @brief 更新识别的数据
/// @param debug_sub    debug的sub
/// @param detector     识别器
/// @param classifier   数字分拣器
void DetectorDebugUpdata(Subscriber<DebugData> &debug_sub, std::unique_ptr<ArmorDetector> &detector,
                         std::unique_ptr<NumberClassifier> &classifier)
{
    DebugData debug_data = debug_sub.pop();
    detector->min_lightness = debug_data.detector_threshold;
    detector->detect_color = debug_data.detector_color;
    classifier->threshold = debug_data.num_threshold;
    return;
}
#endif //!_DEBUG_

/**
 * @brief 发送装甲板信息
 *
 */
void DetectorTargeting()
{
    time_t start_time;
    time_t end_time;
    uint32_t frame_count = 0;
    Subscriber<SensorsData> data_sub("sensors_data");
    Publisher<Detection_pack> armor_pub("armor_set");

#ifdef _DEBUG_
    Publisher<DetectorDebugData> debug_pub("detector_debug");
    Subscriber<DebugData> debug_sub("debug");
    sleep(2); //
#endif        //!_DEBUG_

    //****************************************初始化****************************************
    std::vector<Armor> armor_detector;
    int init_min_l = 100;
    int init_color = 0; //'0'为红色 '1'为蓝色

    // 灯条长宽最小/大比 角度
    ArmorDetector::LightParams init_l{0.05, 0.8, 45};
    // 灯条长宽最小   小装甲比值  大装甲比值    旋转角度max
    ArmorDetector::ArmorParams init_a{0.7, 1.5, 3.5, 3.5, 5.5, 45};
    // 识别参数初始化
    std::unique_ptr<ArmorDetector> detector =
        std::make_unique<ArmorDetector>(init_min_l, init_color, init_l, init_a);

    std::vector<std::string> ignore_classes{"negative"};
    std::string model_path("../src/data_file/data/model/mlp.onnx");
    std::string label_path("../src/data_file/data/model/label.txt");
    double threshold = 0.5;
    std::unique_ptr<NumberClassifier> classifier =
        std::make_unique<NumberClassifier>(
            model_path,
            label_path, threshold,ignore_classes);
    std::unique_ptr<Tracker> armor_tacker = std::make_unique<Tracker>();
    //****************************************初始化****************************************
    
    while (true)
    {
        if (!frame_count)
        {
            time(&start_time);
        }
        const auto &[image, imu, speed, mode, timestamp] = data_sub.pop();

        cv::Mat src;
        cv::Mat binary_img;
        // cv::resize(image, src, cv::Size(WIDTH, HEIGHT));
        cv::cvtColor(image, src, cv::COLOR_BGR2RGB);
        //cv::flip(src, src, -1);
#ifdef _DEBUG_
        DetectorDebugUpdata(debug_sub, detector, classifier);
#endif //!_DEBUG_

        if (mode == 0 || mode == 1) // 打装甲板模式
        {
            detector->detect_color = mode;
            //! armor_pub.push(detector.getTargetInfo());
            armor_detector = detectArmors(src, binary_img, detector, classifier);
            Armor result_armor = armor_tacker->GetTrackedVehicle(armor_detector);
            // armor_pub.push(Detection_pack{result_armor.is_armor, result_armor.armor_type,
            //                               result_armor.center ,
            //                               std::vector<cv::Point2f>{
            //                                   result_armor.left_light.bottom ,
            //                                   result_armor.left_light.top ,
            //                                   result_armor.right_light.top ,
            //                                   result_armor.right_light.bottom },
            //                               image, timestamp, imu, speed});
            if (!armor_detector.empty())
            {
                armor_pub.push(
                    Detection_pack{true, armor_detector[0].armor_type,
                                   armor_detector[0].center,
                                   std::vector<cv::Point2f>{
                                       armor_detector[0].left_light.bottom,
                                       armor_detector[0].left_light.top,
                                       armor_detector[0].right_light.top,
                                       armor_detector[0].right_light.bottom},
                                   image,
                                   timestamp,
                                   imu,
                                   speed});
            }
            else
            {
                armor_pub.push(
                    Detection_pack{false,
                                   ArmorType::SMALL,
                                   cv::Point2f(0, 0),
                                   std::vector<cv::Point2f>{
                                       cv::Point2f(0, 0), cv::Point2f(0, 0),
                                       cv::Point2f(0, 0), cv::Point2f(0, 0)},
                                   image,
                                   timestamp,
                                   imu,
                                   speed});
            }

#ifdef _DEBUG_
            const std::string tracker_state[4]{
                "LOST", "DETECTING", "TRACKING", "TEMP_LOST"};
            // 状态下来
            cv::putText(src, tracker_state[armor_tacker->tracker_state_], cv::Point(25, 25),
                        cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
            debug_pub.push(DetectorDebugData{
                Stm32Data{
                    !armor_detector.empty(),
                    mode,
                    speed,
                    imu},
                src, binary_img});
#endif //!_DEBUG_
        }

        else if (mode == 2 || mode == 3) // 打能量机关模式
        {
            cv::Mat binary_img;
            cv::cvtColor(image, binary_img, cv::COLOR_BGR2GRAY);
            Buff buff(mode);
            buff.Process(image);
            if (buff.IfFindBuff())
            {
                armor_pub.push(
                    Detection_pack{true,
                                   ArmorType::BUFF,
                                   buff.GetArmorCenter(),
                                   buff.GetImagePoints(),
                                   image,
                                   timestamp,
                                   imu,
                                   speed});
            }
            else
            {
                armor_pub.push(
                    Detection_pack{false,
                                   ArmorType::SMALL,
                                   cv::Point2f(0, 0),
                                   std::vector<cv::Point2f>{
                                       cv::Point2f(0, 0), cv::Point2f(0, 0),
                                       cv::Point2f(0, 0), cv::Point2f(0, 0)},
                                   image,
                                   timestamp,
                                   imu,
                                   speed});
            }
#ifdef _DEBUG_
            // const std::string tracker_state[4]{
            //     "LOST", "DETECTING", "TRACKING", "TEMP_LOST"};
            // // 状态下来
            // cv::putText(src, tracker_state[armor_tacker->tracker_state_], cv::Point(25, 25),
            //             cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
            debug_pub.push(DetectorDebugData{
                Stm32Data{
                    buff.IfFindBuff(),
                    mode,
                    speed,
                    imu},
                image, binary_img});
#endif //!_DEBUG_
        }

        frame_count++;
        time(&end_time);
        if (end_time - start_time >= 1)
        {
            printf("<ArmorDetector: FrameCount: %u>\n\n",
                   frame_count); // 输出检测线程的帧率
            frame_count = 0;
        }

        // char chKey = cv::waitKey(1);
    }
    return;
}

#endif // _DETECTOR_HPP_
