/**
 * @file armor_solver.hpp
 * @author
 * @brief 出自上交21年开源
 * @version 0.1
 * @date 2023-04-14
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef PREDICTOR_ADAPTIVE_EKF_HPP_
#define PREDICTOR_ADAPTIVE_EKF_HPP_

// eigen
#include <eigen3/Eigen/Dense>
#include <opencv2/core/eigen.hpp>

// general
#include "message/message.hpp"

// solver
#include "adaptive_ekf.hpp"

struct Predict
{
    /*
     * 此处定义匀速直线运动模型
     */
    template <class T>
    void operator()(const T x0[5], T x1[5])
    {
        x1[0] = x0[0] + delta_t * x0[1]; // 0.1
        x1[1] = x0[1];                   // 100
        x1[2] = x0[2] + delta_t * x0[3]; // 0.1
        // x1[2] = x0[2];
        x1[3] = x0[3]; // 100
        x1[4] = x0[4]; // 0.01
    }

    double delta_t;
};

/**
 * @brief
 * 工具函数：将 xyz 转化为 pitch、yaw、distance
 * @tparam T 数组类型
 * @param xyz
 * @param pyd
 */
template <class T>
void xyz2pyd(T xyz[3], T pyd[3])
{
    pyd[0] = ceres::atan2(
        xyz[2], ceres::sqrt(xyz[0] * xyz[0] + xyz[1] * xyz[1])); // pitch
    pyd[1] = ceres::atan2(xyz[1], xyz[0]);                       // yaw
    pyd[2] = ceres::sqrt(xyz[0] * xyz[0] + xyz[1] * xyz[1] +
                         xyz[2] * xyz[2]); // distance
}

/**
 * @brief
 *工具函数的类封装
 *
 */
struct Measure
{
    template <class T>
    void operator()(const T x[5], T y[3])
    {
        T x_[3] = {x[0], x[2], x[4]};
        // xyz2pyd(x_, y);
        y[0] = x[0];
        y[1] = x[2];
        y[2] = x[4];
    }
};

/**
 * @brief 预测ekf
 *
 */
class PredictorAdaptiveEKF
{
private:
    AdaptiveEKF<5, 3> ekf; // 创建ekf

    double last_time;

    Eigen::Matrix3d R_CI;          // 陀螺仪坐标系到相机坐标系旋转矩阵EIGEN-Matrix
    Eigen::Matrix3d F;             // 相机内参矩阵EIGEN-Matrix
    Eigen::Matrix<double, 1, 5> C; // 相机畸变矩阵EIGEN-Matrix
    cv::Mat R_CI_MAT;              // 陀螺仪坐标系到相机坐标系旋转矩阵CV-Mat
    cv::Mat F_MAT;                 // 相机内参矩阵CV-Mat
    cv::Mat C_MAT;                 // 相机畸变矩阵CV-Mat
    // std::vector<bbox_t> last_boxes;  // 上一帧识别到的所有装甲板
    // bbox_t last_sbbox;               // 上一帧击打的装甲板
    Eigen::Matrix3d last_R_IW; // 上一帧击陀螺仪数据
    Eigen::Vector3d last_m_pw; // 上一帧击打的装甲板的世界坐标
    bool last_shoot = false;   // 判断上一次是否击打
    double last_yaw = 0, last_pitch = 0;
    Imu last_imu;
    Eigen::Vector3d last_s_pc;

    bool distant = false, antitop = false,
         exist_hero = false; // 远距离、反陀螺、英雄存在标志位
    bool last_is_one_armor = false, last_is_two_armors = false, right = true,
         anticlockwise = true; // clockwise：逆时针

    int dead_buffer = 0; // 灰色buffer

    struct Record
    {
        /*
         * 候选装甲板
         */
        // bbox_t bbox;
        Eigen::Vector3d last_m_pw;
        AdaptiveEKF<5, 3> ekf; // 创建ekf
        bool updated = false, init = false;
        double last_yaw = 0, last_pitch = 0;
        float yaw_angle = 0., pitch_angle = 0., yaw_speed = 0, pitch_speed = 0;
        int distance = 0.;
        bool distant = false;
        int age = 0; // 装甲板存活时间

        // Record(bbox_t &armor): bbox(armor), updated(false), init(true),
        // last_yaw(0.), last_pitch(0.),
        //                        yaw_angle(0.), pitch_angle(0.), yaw_speed(0.),
        //                        pitch_speed(0.), distant(false), age(0) {}
    };
    std::vector<Record> antitop_candidates;

    bool debug = true;
    int DEAD_BUFFER = 20;

    // 相机坐标系内坐标--->世界坐标系内坐标   //点xyz的顺序也是需要考虑的问题
    // 现在是传入xyz
    inline Eigen::Vector3d pc_to_pw(const Eigen::Vector3d &pc,
                                    Eigen::Matrix3d &R_IW)
    {
        // R_IW << 0,0,0,0,0,0,0,0,0;
        Eigen::Vector3d offset;
        offset << -0.00171, -0.11799,
            0.0259;       // 相机坐标系（CMOS的成像平面中心）到imu坐标系原点的平移量
        R_CI << -1, 0, 0, // 二者的旋转量
            0, 0, -1, 0, -1, 0;
        Eigen::Vector3d pi = R_CI * pc + offset; // 将点转移到imu坐标系下 xyz

        Eigen::Vector3d pw = R_IW * pi;
        // std::cout<<R_IW<<"\n\n";

        Eigen::Vector3d pw_zxy(
            pw[2], pw[0],
            pw[1]); // 因为在上交使用滤波器用的点顺序是zxy，为了方便使用，也专程这种顺序
        // std::cout<<pw<<"\n\n";

        // 显示 imu坐标系 和 imu(world)坐标系下的三位点坐标
        // double distance = sqrt(pi[0]*pi[0] +  pi[1]*pi[1] +
        // pi[2]*pi[2])*1000; Mat angleImage = Mat::zeros(450,900,CV_8UC3);
        // putText(angleImage, "distance"+to_string(distance), Point(50, 50),
        // FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 0, 255), 1, 8, false);
        // putText(angleImage, "pi"+to_string(pi[0]*1000) + " " +
        // to_string(pi[1]*1000) + " " + to_string(pi[2]*1000), Point(50, 100),
        // FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 0, 255), 1, 8, false);
        // putText(angleImage, "pw"+to_string(pw[0]*1000) + " " +
        // to_string(pw[1]*1000) + " " + to_string(pw[2]*1000), Point(50, 150),
        // FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 0, 255), 1, 8, false);
        // imshow("Point",angleImage);

        return pw_zxy;
    }

    // 世界坐标系内坐标--->相机坐标系内坐标
    inline Eigen::Vector3d pw_to_pc(const Eigen::Vector3d &pw_zxy,
                                    const Eigen::Matrix3d &R_IW)
    {
        Eigen::Vector3d offset;
        offset << -0.00171, -0.11799, 0.0259;
        R_CI << -1, 0, 0, // xyz
            0, 0, -1, 0, -1, 0;

        Eigen::Vector3d pw(pw_zxy[1], pw_zxy[2], pw_zxy[0]); // 转回XYZ的顺序

        Eigen::Vector3d pi = R_IW.transpose() * pw;

        Eigen::Vector3d pc = R_CI.transpose() * (pi - offset);

        Eigen::Vector3d pc_zxy(pc[2], pc[0], pc[1]); // 转会XYZ的顺序

        return pc_zxy;
    }

    // 相机坐标系内坐标--->图像坐标系内像素坐标
    inline Eigen::Vector3d pc_to_pu(const Eigen::Vector3d &pc)
    {
        return F * pc / pc(2, 0);
    }

    // 将世界坐标系内一点，投影到图像中，并绘制该点
    inline void re_project_point(cv::Mat &image, const Eigen::Vector3d &pw,
                                 const Eigen::Matrix3d &R_IW,
                                 const cv::Scalar &color)
    {
        Eigen::Vector3d pc = pw_to_pc(pw, R_IW);
        Eigen::Vector3d pu = pc_to_pu(pc);
        cv::circle(image, {int(pu(0, 0)), int(pu(1, 0))}, 3, color, 2);
    }

    // 计算任意四边形的中心 -- 简单做法
    inline cv::Point2f points_center(cv::Point2f pts[4])
    {
        cv::Point2f center;
        center.x = (pts[0].x + pts[1].x + pts[2].x + pts[3].x) / 4;
        center.y = (pts[0].y + pts[1].y + pts[2].y + pts[3].y) / 4;
        return center;
    }

    // 判断交并比
    float bbOverlap(const cv::Rect2f &box1, const cv::Rect2f &box2);

    // 四点转化为矩形
    // inline cv::Rect2f get_ROI(bbox_t &armor, float coefficient = 1.0f) {
    //     auto center = points_center(armor.pts);
    //     auto w = std::max({armor.pts[0].x, armor.pts[1].x, armor.pts[2].x,
    //     armor.pts[3].x}) -
    //             std::min({armor.pts[0].x, armor.pts[1].x, armor.pts[2].x,
    //             armor.pts[3].x});
    //     auto h = std::max({armor.pts[0].y, armor.pts[1].y, armor.pts[2].y,
    //     armor.pts[3].y}) -
    //             std::min({armor.pts[0].y, armor.pts[1].y, armor.pts[2].y,
    //             armor.pts[3].y});
    //     return cv::Rect2f(center.x-w/2, center.y-h/2, w*coefficient,
    //     h*coefficient);
    // }

    // 海伦公式获得四边形面积
    // double get_quadrangle_size(const bbox_t &bx);

    // 根据距离判断是否是同一块装甲板
    // inline bool is_same_armor_by_distance(const Eigen::Vector3d old_m_pw,
    // const bbox_t &new_armor, const Eigen::Matrix3d &R_IW, const double
    // distance_threshold = 0.15) {
    //     Eigen::Vector3d new_m_pc = pnp_get_pc(new_armor.pts,
    //     new_armor.tag_id); Eigen::Vector3d new_m_pw = pc_to_pw(new_m_pc,
    //     R_IW); Eigen::Vector3d m_pw_delta = new_m_pw - old_m_pw; double
    //     distance = m_pw_delta.norm(); if (distance < distance_threshold)
    //     return true; else return false;
    // }

    // 根据第五层防抖匹配两个装甲板
    // void match_armors(bbox_t &armor, bool &selected, const
    // std::vector<bbox_t> &detections, const Eigen::Matrix3d &R_IW, const bool
    // right);

    // pnp解算:获取相机坐标系内装甲板坐标
    Eigen::Vector3d pnp_get_pc(const cv::Point2f p[4], int armor_number);

public:
    inline void load_param(bool update_all = true)
    {
        /*
         * 通过文件读入，实现实时调参
         */
        cv::FileStorage fin("../src/data_file/data/init/autoaim-param.yml",
                            cv::FileStorage::READ);

        if (!antitop)
        {
            // 设置对角线的值
            // 预测过程协方差
            fin["Q00"] >> ekf.Q(0, 0);
            fin["Q11"] >> ekf.Q(1, 1);
            fin["Q22"] >> ekf.Q(2, 2);
            fin["Q33"] >> ekf.Q(3, 3);
            fin["Q44"] >> ekf.Q(4, 4);
            // 观测过程协方差
            fin["R00"] >> ekf.R(0, 0);
            fin["R11"] >> ekf.R(1, 1);
            fin["R22"] >> ekf.R(2, 2);

            if (update_all)
            {
                fin["DEBUG"] >> debug; // 对反陀螺几乎没什么用
                fin["DEAD_BUFFER"] >> DEAD_BUFFER;
                for (auto &a : antitop_candidates)
                {
                    // 预测过程协方差
                    fin["Q00"] >> a.ekf.Q(0, 0);
                    fin["Q11"] >> a.ekf.Q(1, 1);
                    fin["Q22"] >> a.ekf.Q(2, 2);
                    fin["Q33"] >> a.ekf.Q(3, 3);
                    fin["Q44"] >> a.ekf.Q(4, 4);
                    // 观测过程协方差
                    fin["R00"] >> a.ekf.R(0, 0);
                    fin["R11"] >> a.ekf.R(1, 1);
                    fin["R22"] >> a.ekf.R(2, 2);
                }
            }
        }
        else
        {
            // 设置对角线的值
            // 预测过程协方差
            fin["Q00_AC"] >> ekf.Q(0, 0);
            fin["Q11_AC"] >> ekf.Q(1, 1);
            fin["Q22_AC"] >> ekf.Q(2, 2);
            fin["Q33_AC"] >> ekf.Q(3, 3);
            fin["Q44_AC"] >> ekf.Q(4, 4);
            // 观测过程协方差
            fin["R00_AC"] >> ekf.R(0, 0);
            fin["R11_AC"] >> ekf.R(1, 1);
            fin["R22_AC"] >> ekf.R(2, 2);

            if (update_all)
            {
                fin["DEBUG"] >> debug; // 对反陀螺几乎没什么用
                fin["DEAD_BUFFER"] >> DEAD_BUFFER;
                for (auto &a : antitop_candidates)
                {
                    // 预测过程协方差
                    fin["Q00_AC"] >> a.ekf.Q(0, 0);
                    fin["Q11_AC"] >> a.ekf.Q(1, 1);
                    fin["Q22_AC"] >> a.ekf.Q(2, 2);
                    fin["Q33_AC"] >> a.ekf.Q(3, 3);
                    fin["Q44_AC"] >> a.ekf.Q(4, 4);
                    // 观测过程协方差
                    fin["R00_AC"] >> a.ekf.R(0, 0);
                    fin["R11_AC"] >> a.ekf.R(1, 1);
                    fin["R22_AC"] >> a.ekf.R(2, 2);
                }
            }
        }
    }

    explicit PredictorAdaptiveEKF()
    {
        // cv::FileStorage fin("../General/camera-param.yml",
        // cv::FileStorage::READ); fin["Tcb"] >> R_CI_MAT; fin["K"] >> F_MAT;
        // fin["D"] >> C_MAT;
        // cv::cv2eigen(R_CI_MAT, R_CI);
        // cv::cv2eigen(F_MAT, F);
        // cv::cv2eigen(C_MAT, C);

        load_param();

        std::cout << "Finish create a new EKF.\n";
    }

    cv::Point2f estimate(double &x, double &y, double &z, double &yaw,
                         double &pitch, double &distance, Imu &imu, float &t);

    // bool predict(Detection_pack &, RobotCmd &, cv::Mat &, bool);

    inline void clear()
    {
        // last_boxes.clear();
        last_shoot = false;
        antitop = false;
        dead_buffer = 0;
        antitop_candidates.clear();
    }

    ~PredictorAdaptiveEKF() = default;
};

/// 射击延迟
// 旧哨兵下云台：0.105
// 新哨兵下云台：0.090
// 新哨兵上云台：0.110
constexpr double shoot_delay = 0.210; // 上下云台的射击延迟是不一样的

/// 选择高度限制
constexpr double height_thres = 0.;
/// 识别双阈值
constexpr float high_thres = 0.6f;
constexpr float low_thres = 0.2f;

/// dead_buffer_max_size
int dead_buffer_max_size = 20;

/// 最重要装甲板
constexpr int important_id = 1;

/// 斩杀线
constexpr uint8_t killer_point = 80;

/// 远距离弹量控制
constexpr double distant_threshold = 6.;

/// 反陀螺模式下速度限制幅度
constexpr double antitop_x_v_proportion = 0.;
constexpr double antitop_y_v_proportion = 0.;

/// 反陀螺速度继承
constexpr float ac_x_v_coefficient = 0.5f;
constexpr float ac_y_v_coefficient = 0.5f;

/// 切换装甲板大小限制
/// 预测
constexpr float switch_armor_size_proportion = 1.1;

/// 初始化候选ekf大小阈值比例
constexpr double ac_init_min_age = 1; // +1

/// 反陀螺频率统计记忆窗口
constexpr double beta = 0.25; // 记忆窗口大小 T = 1/beta

/// DEBUG
bool DEBUG = true;
////////////////////////// 调参区 end //////////////////////////

struct LastTimeHelper
{
    LastTimeHelper(double current_time, double &ref_time)
        : c_time(current_time), r_time(ref_time){};

    ~LastTimeHelper() { r_time = c_time; }

    double c_time;
    double &r_time;
};

cv::Point2f PredictorAdaptiveEKF::estimate(double &x, double &y, double &z,
                                           double &yaw, double &pitch,
                                           double &distance, Imu &imu,
                                           float &t)
{

    t = t * 1e-3;
    static bool init = false;
    LastTimeHelper helper(t, last_time);
    double delta_t = t - last_time; // s  图片之间的时间戳之差

    //
    double final_yaw = imu.yaw - yaw;
    double final_pitch = imu.roll - pitch;

    if (abs(final_yaw - last_yaw) > 5. / 180. * M_PI ||
        abs(final_pitch - last_pitch) > 5. / 180. * M_PI) // 防止装甲板发生切换
    {
        init = false;
    }
    last_yaw = final_yaw;
    last_pitch = final_pitch;
    Eigen::Vector3d m_pyd(final_pitch, final_yaw, distance);
    if (!init)
    {
        // std::cout << "init" << std::endl;
        Eigen::Matrix<double, 5, 1> Xr;
        Xr << m_pyd(0, 0), 0, m_pyd(1, 0), 0, m_pyd(2, 0);
        ekf.init(Xr);
        init = true;
        return cv::Point2f(0., 0.);
    }

    //*预测在这里
    Predict predictfunc;
    Measure measure;

    // Xr是pitch pitch加速度 yaw yaw加速度 distance
    Eigen::Matrix<double, 5, 1> Xr;
    Xr << m_pyd(0, 0), 0, m_pyd(1, 0), 0, m_pyd(2, 0);
    Eigen::Matrix<double, 3, 1> Yr;
    measure(Xr.data(), Yr.data());
    predictfunc.delta_t = delta_t;
    ekf.predict(predictfunc);
    Eigen::Matrix<double, 5, 1> Xe = ekf.update(measure, Yr);
    // std::cout << "Xe" << Xe << "\n";
    double predict_time = shoot_delay; // 预测时间=(发射延迟)+飞行时间（单位:s）110ms
    predictfunc.delta_t = predict_time;
    Eigen::Matrix<double, 5, 1> Xp;
    predictfunc(
        Xe.data(),
        Xp.data()); // 根据滤波得出的pyd速度以及预测时间，计算处下一时刻的pyd

    // Eigen::Vector3d c_pw{Xe(0, 0), Xe(2, 0), Xe(4, 0)};
    Eigen::Vector3d p_pyd{Xp(0, 0), Xp(2, 0), Xp(4, 0)}; // predict
    //std::cout << "预测的pyd:" << p_pyd << "\n\n";
    double pre_z = p_pyd(2, 0) * p_pyd(2, 0) / hypot(tan(p_pyd(0, 0)), tan(p_pyd(1, 0)), 1);
    double cx = 318.7508; // 内参系数
    double cy = 261.4049;
    double fx = 606.8072;
    double fy = 607.5783;
    Eigen::Vector3d p_pw(pre_z * tan(p_pyd(1, 0)), pre_z * tan(p_pyd(0, 0)), pre_z);
    // Eigen::Vector3d s_pc = pw_to_pc(p_pw, R_IW); // imu(world) --> camera
    // Eigen::Vector3d s_pc(0, 0, 0);

    cv::Point2f point_predict = cv::Point(fx * (p_pw(1, 0) / p_pw(0, 0)) + cx,
                                          fy * (p_pw(2, 0) / p_pw(0, 0)) +
                                              cy); // 把三维点投影到二维图像上
    return point_predict;
}

float PredictorAdaptiveEKF::bbOverlap(const cv::Rect2f &box1,
                                      const cv::Rect2f &box2)
{
    if (box1.x > box2.x + box2.width)
    {
        return 0.0;
    }
    if (box1.y > box2.y + box2.height)
    {
        return 0.0;
    }
    if (box1.x + box1.width < box2.x)
    {
        return 0.0;
    }
    if (box1.y + box1.height < box2.y)
    {
        return 0.0;
    }
    float colInt = std::min(box1.x + box1.width, box2.x + box2.width) -
                   std::max(box1.x, box2.x);
    float rowInt = std::min(box1.y + box1.height, box2.y + box2.height) -
                   std::max(box1.y, box2.y);
    float intersection = colInt * rowInt;
    float area1 = box1.width * box1.height;
    float area2 = box2.width * box2.height;
    return intersection / (area1 + area2 - intersection);
}

#endif //! PREDICTOR_ADAPTIVE_EKF_HPP_
