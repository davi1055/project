/**
 * @file armor_tacker.hpp
 * @author luoyebai (2112216825@qq.com)
 * @brief 一个简易的追踪器
 * @version 0.1
 * @date 2023-04-14
 *
 * @copyright Copyright (c) 2023
 *
 */

//! 未完成
// #TODO 装甲板追踪器编写

#ifndef DETECTOR_ARMOR_TACKER_HPP_
#define DETECTOR_ARMOR_TACKER_HPP_

// detector
#include "armor_detector.hpp"

constexpr int switch_threshold = 10; // 持续10帧识别(未识别)进入跟随(丢失)

/**
 * @brief 追踪器
 *
 */
class Tracker
{
private:
    size_t detector_times;
    size_t lost_times_;

public:
    using AllArmor = std::vector<Armor>;
    enum State
    {
        LOST,
        DETECTING,
        TRACKING,
        TEMP_LOST,
    } tracker_state_;

    Tracker() : detector_times(0), lost_times_(0), tracker_state_{LOST} {};

    /**
     * @brief  对某些状态决策改变识别或解算方式
     *
     */
    void DecisionMaking();

    /**
     * @brief Get the Tracked Vehicle object
     *
     * @param all_armor
     * @return Armor
     */
    Armor GetTrackedVehicle(const AllArmor &all_armor);

private:
    Armor last_tracked_armor_;
    Armor tracked_armor_;
    void Updata(const AllArmor &all_armor)
    {
        // 无目标
        if (all_armor.empty())
        {
            // 识别次数清理
            detector_times = 0;
            // 丢失不到10次
            if (lost_times_ != switch_threshold)
            {
                ++lost_times_;
                tracker_state_ = TEMP_LOST;
            }
            // 进入lost
            else
            {
                tracker_state_ = LOST;
                cv::Point2f point{0., 0.};
                Light l1(cv::RotatedRect{point, point, point});
                Light l2(cv::RotatedRect{point, point, point});
                last_tracked_armor_ = Armor(l1, l2);
                last_tracked_armor_.is_armor = false;
            }
        }
        else
        {
            // 丢失次数清理
            lost_times_ = 0;
            // 连续识别次数不到10次
            if (detector_times != switch_threshold)
            {
                ++detector_times;
                tracker_state_ = DETECTING;
            }
            // 连续识别到5次了
            else
            {
                tracker_state_ = TRACKING;
            }
        }
    }
};

Armor Tracker::GetTrackedVehicle(const AllArmor &all_armor)
{
    Updata(all_armor);
    // 丢失则返回上次
    if (tracker_state_ == TEMP_LOST || tracker_state_ == LOST)
        return last_tracked_armor_;
    // 识别到了但是唯一
    if (all_armor.size() == 1)
        return all_armor[0];
    // 不唯一时
    tracked_armor_ = all_armor[0];
    for (const auto &armor : all_armor)
    {
        // 数字相同选择置信率高的armor
        bool number_ok = (tracked_armor_.number == armor.number) &&
                         (tracked_armor_.confidence < armor.confidence);
        // 数字不同则选择更有价值的数字
        // 置信率更高且不是数字0
        number_ok = number_ok ||
                    ((tracked_armor_.confidence < armor.confidence) &&
                     armor.number != "0");
        bool updata_tracker_ok = number_ok;
        if (updata_tracker_ok)
            tracked_armor_ = armor;
        last_tracked_armor_ = tracked_armor_;
    }
    return tracked_armor_;
}

#endif //! DETECTOR_ARMOR_TACKER_HPP_