# !/home/liu/miniconda3/envs/dahengcamera/bin/python 
# coding=utf-8

import sys
import cv2  # 导入opencv包
import gxipy as gx # 导入大恒相机Python包
import time

# 打开设备
# 枚举设备
device_manager = gx.DeviceManager() 
dev_num, dev_info_list = device_manager.update_device_list()
if dev_num == 0:
    sys.exit(1)
# 获取设备基本信息列表
str_sn = dev_info_list[0].get("sn")
# 通过序列号打开设备
cam = device_manager.open_device_by_sn(str_sn)
# 导入配置信息
# cam.import_config_file("./import_config_file.txt")
# 开始采集
cam.BalanceWhiteAuto.set(20000)
cam.BalanceWhiteAuto.set(2)
cam.stream_on()

# 视频存储的格式 
fourcc = cv2.VideoWriter_fourcc(*'XVID' )
# 帧率
fps = cam.AcquisitionFrameRate.get()  
# 视频的宽高
size = (cam.Width.get(),cam.Height.get())
print(cam.Width.get(),cam.Height.get())
# 文件名定义
filename = '/home/davi/camera/Video_'+time.strftime("%Y%m%d_%H%M%S", time.localtime())+'.avi'
#视频存储
out = cv2.VideoWriter(filename, fourcc, fps, size)
  
while out.isOpened():
    raw_image = cam.data_stream[0].get_image()    # 使用相机采集一张图片
    rgb_image = raw_image.convert("RGB")    # 从彩色原始图像获取 RGB 图像
    numpy_image = rgb_image.get_numpy_array()   # 从 RGB 图像数据创建 numpy 数组
    if rgb_image is None:
        continue
    numpy_image = cv2.cvtColor(numpy_image, cv2.COLOR_RGB2BGR)  # opencv采用的是BGR图像， 讲RGB转为BGR
    cv2.namedWindow('video',cv2.WINDOW_NORMAL)#创建一个名为video的窗口
    cv2.imshow('video',numpy_image)   # 将捕捉到的图像在video窗口显示
    out.write(numpy_image)    # 将捕捉到的图像存储

    #按esc键退出程序
    if cv2.waitKey(1) & 0xFF ==27:
        break

# 停止录制,关闭设备
cam.stream_off()
cam.close_device()