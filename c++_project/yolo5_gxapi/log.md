截至2023.3.8代码完成情况阶段性总结：
        1.c++调用yolo5完成
        2.yolo5检测到的坐标数据写入Yolodata.xml完成
        3.读取Yolodata.xml中的数据并获得装甲板中心坐标完成
        4.从xml文件中读取相机内参和畸变参数完成
    下一步目标总结：
        1.获取相机坐标系：需要条件：相机内参（已知）、相机畸变参数（已知）、物体在图像中的像素坐标（像素坐标系）【已知】、相机坐标系与世界坐标系的变换关系（未知）。因此，问题转化为如何获取相机坐标系与世界坐标系的转化。
        2.获取相机坐标系与世界坐标系的转化：需要条件：相机标定（已知）、物体检测，确定物体在图像中的位置（未知）、姿态估计（未知）。
2023.3.9:
        1.pnp解算完成（相机还没标定）
        2.开始写EKF。预计4.1号勉强完成新代码（没时间调试）
2023.3.10：