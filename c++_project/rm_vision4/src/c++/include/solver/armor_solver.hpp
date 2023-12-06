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

#ifndef SOLVER_ARMOR_SOLVER_HPP_
#define SOLVER_ARMOR_SOLVER_HPP_

// general
#include "message/message.hpp"

// solver
#include "predictor_adaptive_ekf.hpp"

class AngleSolver
{
public:
    AngleSolver();
    ~AngleSolver();

    /**
     * @brief Set camera params
     * @param camera_matrix: camera IntrinsicMatrix
     * @param distortion_coeff: camera DistortionCoefficients
     */
    void setCameraParam(const cv::Mat &camera_matrix, const cv::Mat &distortion_coeff);
    // overload function. Params set by xml file
    int setCameraParam(const char *filePath, int camId);

    /**
     * @brief Set armor size
     * @param type: input target type small/big
     * @param width: the width of armor (mm)
     * @param height: the height of armor (mm)
     */
    void setArmorSize(ArmorType type, double width, double height);

    /**
     * @brief Set bullet speed
     * @param bulletSpeed: the speed of bullet(mm/s)
     */
    void setBulletSpeed(int bulletSpeed);

    /**
     * @brief set the target armor contour points and centerPoint
     * @param points2d image points set with following order : left_up, right_up, left_down, right_down
     * @param type target armor type
     */
    void setTarget(std::vector<cv::Point2f> contoursPoints, cv::Point2f centerPoint, ArmorType type);

    /**
     * @brief solve the angles using P4P or PinHole according to the distance
     */
    void solveAngles();

    /**
     * @brief solve the angles using P4P method
     */
    void AngleSolveResult();

    /**
     * @brief solve the angles using PinHole method
     */
    void PinHole_solver();

    /**
     * @brief compensation of pitch
     */
    void compensateAngle();

    /**
     * @brief compensation of pitch for y_offset between barrel and camera
     */
    void compensateOffset();

    /**
     * @brief compensation of pitch for gravity
     */
    void compensateGravity();

    /**
     * @brief according to the target2D points to get the yaw and pitch and distance towards the certain type target using solvePnP
     * @param inputArrayOfPoints cornerPoints, the vertices of target armor
     * @param inputPoint centerPoint the center point of target armor
     * @param input type the type of armor BIG_ARMOR or SMALL_ARMOR
     * @param output y_yaw angle     the angle that yaw revolve     '<-' left is minus-       '->' right is positive+
     * @param output x_pitch angle   the angle that pitch revolve   '下' down is minus-       '上' up is positive+
     * @param output distance  unit is mm
     */
    void getAngle(Detection_pack detection, double &x, double &y, double &z, double &yaw, double &pitch, double &evaluateDistance);

    void addPredictPoint(cv::Point2f prePoint);
    /**
     * @brief show debug information
     */
    void showDebugInfo(bool showCurrentResult, bool showTVec, bool showP4P, bool showPinHole, bool showCompensation, bool showCameraParams);

private:
    // Camera params
    cv::Mat CAMERA_MATRIX;    // IntrinsicMatrix		  fx,fy,cx,cy
    cv::Mat DISTORTION_COEFF; // DistortionCoefficients k1,k2,p1,p2,k3

    // Object points in world coordinate
    std::vector<cv::Point3f> SMALL_ARMOR_POINTS_3D;
    std::vector<cv::Point3f> BIG_ARMOR_POINTS_3D;
    std::vector<cv::Point3f> BUFF_POINTS_3D;

    // speed of bullet (compensation for gravity and air fru)
    float BULLET_SPEED;

    // distance between camera and barrel in y axis(positive when camera is under barrel)  barrel_y = camera_y + barrel_camera_y
    float GUN_CAM_DISTANCE_Y;

    // Targets
    cv::Point2f predictPoint;
    cv::Point2f targetCenter;
    ArmorType targetType;
    std::vector<cv::Point2f> cornerPoints;
    // calculated by solvePnP
    // s[R|t]=s'  s->world coordinate;s`->camera coordinate
    cv::Mat rVec_; // rot rotation between camera and target center
    cv::Mat tVec_; // trans tanslation between camera and target center

    // Results;
    double x_pos;
    double y_pos;
    double z_pos;
    float y_yaw;
    float x_pitch;
    double distance;
};
AngleSolver::AngleSolver()
{
}

AngleSolver::~AngleSolver()
{
}

void AngleSolver::setCameraParam(const cv::Mat &camera_matrix, const cv::Mat &distortion_coeff)
{
    camera_matrix.copyTo(CAMERA_MATRIX);
    distortion_coeff.copyTo(DISTORTION_COEFF);
}

int AngleSolver::setCameraParam(const char *filePath, int camId)
{
    cv::FileStorage fsRead;
    fsRead.open(filePath, cv::FileStorage::READ);
    if (!fsRead.isOpened())
    {
        puts("Failed to open xml\n");
        return -1;
    }

    fsRead["Y_DISTANCE_BETWEEN_GUN_AND_CAM"] >> GUN_CAM_DISTANCE_Y;

    cv::Mat camera_matrix;
    cv::Mat distortion_coeff;
    switch (camId)
    {
    case 1:
        fsRead["CAMERA_MATRIX_1"] >> camera_matrix;
        fsRead["DISTORTION_COEFF_1"] >> distortion_coeff;
        break;
    case 2:
        fsRead["CAMERA_MATRIX_2"] >> camera_matrix;
        fsRead["DISTORTION_COEFF_2"] >> distortion_coeff;
        break;
    case 3:
        fsRead["CAMERA_MATRIX_3"] >> camera_matrix;
        fsRead["DISTORTION_COEFF_3"] >> distortion_coeff;
        break;
    case 4:
        fsRead["CAMERA_MATRIX_4"] >> camera_matrix;
        fsRead["DISTORTION_COEFF_4"] >> distortion_coeff;
        break;
    case 5:
        fsRead["CAMERA_MATRIX_5"] >> camera_matrix;
        fsRead["DISTORTION_COEFF_5"] >> distortion_coeff;
        break;
    default:
        puts("WRONG CAMID GIVEN!");
        break;
    }
    setCameraParam(camera_matrix, distortion_coeff);
    fsRead.release();
    return 0;
}

void AngleSolver::setArmorSize(ArmorType type, double width, double height)
{
    double half_x = width / 2.0;
    double half_y = height / 2.0;
    switch (type)
    {
    case SMALL:
        SMALL_ARMOR_POINTS_3D.push_back(cv::Point3f(-half_x, half_y, 0));  // tl top left
        SMALL_ARMOR_POINTS_3D.push_back(cv::Point3f(half_x, half_y, 0));   // tr top right
        SMALL_ARMOR_POINTS_3D.push_back(cv::Point3f(half_x, -half_y, 0));  // br below right
        SMALL_ARMOR_POINTS_3D.push_back(cv::Point3f(-half_x, -half_y, 0)); // bl below left
        break;

    case LARGE:
        BIG_ARMOR_POINTS_3D.push_back(cv::Point3f(-half_x, half_y, 0));  // tl top left
        BIG_ARMOR_POINTS_3D.push_back(cv::Point3f(half_x, half_y, 0));   // tr top right
        BIG_ARMOR_POINTS_3D.push_back(cv::Point3f(half_x, -half_y, 0));  // bl below left
        BIG_ARMOR_POINTS_3D.push_back(cv::Point3f(-half_x, -half_y, 0)); // br below right
    case BUFF:
        BUFF_POINTS_3D.push_back(cv::Point3f(-165, 183, 0)); // tl top left
        BUFF_POINTS_3D.push_back(cv::Point3f(165, 183, 0));  // tr top right
        BUFF_POINTS_3D.push_back(cv::Point3f(-165, 49, 0));  // bl below left
        BUFF_POINTS_3D.push_back(cv::Point3f(165, 49, 0));   // br below right
        break;
    default:
        break;
    }
}

void AngleSolver::setBulletSpeed(int bulletSpeed)
{
    BULLET_SPEED = bulletSpeed;
}

void AngleSolver::setTarget(std::vector<cv::Point2f> cornerPoints, cv::Point2f centerPoint, ArmorType type)
{
    AngleSolver::cornerPoints = cornerPoints; // 设定目标顶点
    AngleSolver::targetCenter = centerPoint;    // 设定目标中心点
    AngleSolver::targetType = type;
    // std::cout<<targetType<<std::endl;
}

void AngleSolver::solveAngles()
{
    cv::Mat _rvec;
    switch (targetType)
    {
    case SMALL:
        solvePnP(SMALL_ARMOR_POINTS_3D, cornerPoints, CAMERA_MATRIX, DISTORTION_COEFF, _rvec, tVec_, false, cv::SOLVEPNP_IPPE);
        break;
    case LARGE:
        solvePnP(BIG_ARMOR_POINTS_3D, cornerPoints, CAMERA_MATRIX, DISTORTION_COEFF, _rvec, tVec_, false, cv::SOLVEPNP_IPPE);
        break;
    case BUFF:
        solvePnP(BUFF_POINTS_3D, cornerPoints, CAMERA_MATRIX, DISTORTION_COEFF, _rvec, tVec_, false, cv::SOLVEPNP_IPPE);
    default:
        break;
    }
    // std::cout<<"tVec_:"<<tVec_<<"\n";
    tVec_.at<double>(1, 0) -= GUN_CAM_DISTANCE_Y; // 枪口与摄像头之间的距离 mm camera_parma.xml
    // std::cout<<"tVec_"<<tVec_<<"\n";

    x_pos = tVec_.at<double>(0, 0) * 1e-3;
    y_pos = tVec_.at<double>(1, 0) * 1e-3;
    z_pos = tVec_.at<double>(2, 0) * 1e-3;
    distance = sqrt(x_pos * x_pos + y_pos * y_pos + z_pos * z_pos); // m

    // Target is too far, using PinHole solver
    //    if (distance > 8000)
    // {
    // 	PinHole_solver();
    // }
    // Target is moderate, using PnP solver
    // else
    // {
    AngleSolveResult();
    // }
}

void AngleSolver::AngleSolveResult()
{

    double tan_pitch = y_pos / hypot(x_pos, z_pos);
    double tan_yaw = x_pos / z_pos;
    x_pitch = -atan(tan_pitch);
    y_yaw = atan(tan_yaw); // 是弧度
    PinHole_solver();
}

void AngleSolver::PinHole_solver()
{
    double fx = CAMERA_MATRIX.at<double>(0, 0);
    double fy = CAMERA_MATRIX.at<double>(1, 1);
    double cx = CAMERA_MATRIX.at<double>(0, 2);
    double cy = CAMERA_MATRIX.at<double>(1, 2);
    cv::Point2f pnt;
    std::vector<cv::Point2f> in;
    std::vector<cv::Point2f> out;
    in.push_back(targetCenter);

    // 对像素点去畸变
    undistortPoints(in, out, CAMERA_MATRIX, DISTORTION_COEFF, cv::noArray(), CAMERA_MATRIX);
    pnt = out.front();

    // 去畸变后的比值
    double rxNew = (pnt.x - cx) / fx;
    double ryNew = (pnt.y - cy) / fy;

    y_yaw = atan(rxNew);
    x_pitch = -atan(ryNew);
}

void AngleSolver::compensateAngle()
{
    // std::cout<<"bullet: "<<BULLET_SPEED<<" m/s"<<std::endl;
    // 角度：弧度  长度：米
    // d_a d_b d_c为关于t^2(子弹运动时间平方)的方程的abc三个系数
    BULLET_SPEED = 20;
    double d_a = 9.8 * 9.8 * 0.25;
    double d_b = -BULLET_SPEED * BULLET_SPEED; //- 9.8 * y_pos; // distance * 9.8 * cos(M_PI_2 + x_pitch); //速度 m/s 高度是用y轴体现的，所以偏移量补偿在y轴上
    double d_c = distance * distance;
    double d_t_2 = (-sqrt(d_b * d_b - 4 * d_a * d_c) - d_b) / (2 * d_a); // 求根公式
    // double d_fly_time = sqrt(d_t_2);
    double d_height = 0.5 * 9.8 * d_t_2; // 换算成高度补偿
    x_pitch = -atan2((y_pos - d_height), sqrt(x_pos * x_pos + z_pos * z_pos));
}

// void AngleSolver::compensateOffset()
// {
//     float camera_target_height = distance * sin(x_pitch / 180 * CV_PI);
//     float gun_target_height = camera_target_height + GUN_CAM_DISTANCE_Y;
//     float gun_pitch_tan = gun_target_height / (distance * cos(x_pitch / 180 * CV_PI));
//     x_pitch = atan(gun_pitch_tan) / CV_PI * 180;
// }

// void AngleSolver::compensateGravity()
// {
//     float compensateGravity_pitch_tan = tan(x_pitch/180*CV_PI) + (0.5*9.8*(distance / BULLET_SPEED)*(distance / BULLET_SPEED)) / cos(x_pitch/180*CV_PI);
//     x_pitch = atan(compensateGravity_pitch_tan)/CV_PI*180;
// }

void AngleSolver::getAngle(Detection_pack detection, double &x, double &y, double &z, double &yaw, double &pitch, double &evaluateDistance)
{
    if (detection.findArmor)
    {
        setTarget(detection.cornerPoints, detection.centerPoint, detection.type);
        solveAngles();
        compensateAngle(); // 重力补偿
        x = x_pos;
        y = y_pos;
        z = z_pos;
        yaw = y_yaw;
        pitch = -x_pitch;
        evaluateDistance = distance;
    }
    else
    {
        yaw = 0.0;
        pitch = 0.0;
        evaluateDistance = 0.0;
    }
}

void AngleSolver::addPredictPoint(cv::Point2f prePoint)
{
    predictPoint = prePoint;
}

void AngleSolver::showDebugInfo(bool showCurrentResult, bool showTVec, bool showP4P, bool showPinHole, bool showCompensation, bool showCameraParams)
{
    if (cornerPoints.empty())
        return;
    cv::Mat points_img = cv::Mat::zeros(cv::Size(WIDTH, HEIGHT), CV_8UC3);
    for (size_t i = 0; i < cornerPoints.size(); ++i) // 将roi还原到原图片坐标系下
    {
        line(points_img, cornerPoints[i], cornerPoints[(i + 1) % 4], cv::Scalar(255, 255, 255), 2);
    }
    circle(points_img, cornerPoints[0], 2, cv::Scalar(255, 255, 255), 2, 8, 0);
    circle(points_img, cv::Point2f((cornerPoints[0].x + cornerPoints[2].x) / 2, (cornerPoints[0].y + cornerPoints[2].y) / 2), 2, cv::Scalar(255, 0, 255), 2, 8, 0);
    // 预测的
    circle(points_img, predictPoint, 2, cv::Scalar(0, 255, 255), 2, 8, 0);

    line(points_img, cv::Point(0, points_img.rows / 2), cv::Point(points_img.cols, points_img.rows / 2), cv::Scalar(0, 255, 0), 2);
    line(points_img, cv::Point(points_img.cols / 2, 0), cv::Point(points_img.cols / 2, points_img.rows), cv::Scalar(0, 255, 0), 2);
    imshow("data", points_img);

    if (showCurrentResult)
    {
        cv::Mat angleImage = cv::Mat::zeros(250, 600, CV_8UC3);
        putText(angleImage, "Yaw: " + std::to_string(y_yaw / 3.14 * 180), cv::Point(100, 50), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 0, 255), 1, 8, false);
        putText(angleImage, "Pitch: " + std::to_string(x_pitch / 3.14 * 180), cv::Point(100, 100), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 0, 255), 1, 8, false);
        putText(angleImage, "Distance: " + std::to_string(distance), cv::Point(100, 150), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 0, 255), 1, 8, false);
        putText(angleImage, "X:" + std::to_string((int)(tVec_.at<double>(0, 0))), cv::Point(50, 200), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 0, 255), 1, 8, false);
        putText(angleImage, "Y:" + std::to_string((int)(tVec_.at<double>(1, 0))), cv::Point(250, 200), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 0, 255), 1, 8, false);
        putText(angleImage, "Z:" + std::to_string((int)(tVec_.at<double>(2, 0))), cv::Point(400, 200), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 0, 255), 1, 8, false);
        imshow("AngleSolver", angleImage);
    }
    if (showTVec)
    {
        printf("tVec_:\n X:%lf Y:%lf  Z:%lf\n", tVec_.at<double>(0, 0), tVec_.at<double>(1, 0), tVec_.at<double>(2, 0));
        puts("-----------------------------------------------\n");
    }
    float yaw_temp = y_yaw, pitch_temp = x_pitch;
    if (showP4P)
    {
        AngleSolveResult();
        puts("AngleSolveResult:\n");
        printf("Yaw:%lf Pitch:%lf\n", y_yaw, x_pitch);
        puts("-----------------------------------------------\n");
        y_yaw = yaw_temp;
        x_pitch = pitch_temp;
    }
    if (showPinHole)
    {
        PinHole_solver();
        puts("PinHole Solver:");
        printf("Yaw:%lf Pitch:%lf\n", y_yaw, x_pitch);
        puts("-----------------------------------------------\n");

        y_yaw = yaw_temp;
        x_pitch = pitch_temp;
    }
    if (showCompensation)
    {
        // solveAngles();
        // compensateOffset();
        // cout<<"Gun-Camera Offset:"<<x_pitch - raw_pitch<<endl;
        // cout<<"Yaw: " << y_yaw << "Pitch: " << x_pitch <<endl;
        // cout << "-----------------------------------------------" << endl;

        // solveAngles();
        // compensateGravity();
        // cout<<"Gravity:"<<x_pitch - raw_pitch<<endl;
        // cout<<"Yaw: " << y_yaw << "Pitch: " << x_pitch <<endl;
        // cout << "-----------------------------------------------" << endl;

        solveAngles();
        float raw_pitch;
        raw_pitch = x_pitch;
        compensateAngle();
        puts("Compensation:\n");
        printf("Pitch Compensation:%lf\n", x_pitch - raw_pitch);
        printf("Yaw:%lf Pitch:%lf\n", y_yaw, x_pitch);
        puts("-----------------------------------------------\n");

        y_yaw = yaw_temp;
        x_pitch = pitch_temp;
    }
    if (showCameraParams) // 输出相机内参和畸变系数
    {
        puts("CANERA_MATRIX:\n");
        std::cout << CAMERA_MATRIX << "\n";
        puts("-----------------------------------------------\n");
        puts("DISTORTION_COEFF:\n");
        std::cout << DISTORTION_COEFF << "\n";
        puts("-----------------------------------------------\n");
    }
    cv::waitKey(1);
}

#endif //! SOLVER_ARMOR_SOLVER_HPP_
