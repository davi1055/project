from train import trainner
from detect import detector
import os

#训练数据集
current_path = os.path.dirname(__file__)#获取脚本当前所在的目录
cnn=trainner.SimpleCNN()
trainer=trainner.Trainer()
trainer.Train(f"{current_path}/data")

#使用训练好的模型进行识别
detector =detector.Detector()
detector.Detect("./model.pth")