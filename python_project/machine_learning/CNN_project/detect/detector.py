import cv2
import numpy as np
import joblib
import torch
import torchvision.transforms as transforms

global drawing ,last_point,background,canvas

def draw_number(event, x, y, flags, param):
        global drawing  ,last_point,background,canvas
        draw_color = (255, 255,255)

        if event == cv2.EVENT_LBUTTONDOWN:
            drawing = True
            last_point = (x, y)
        elif event == cv2.EVENT_MOUSEMOVE:
            if drawing:
                cv2.line(canvas, last_point, (x, y), draw_color, 10)
                last_point = (x, y)
        elif event == cv2.EVENT_LBUTTONUP:
            drawing = False

class Detector():
    def __init__(self):
        super(Detector,self).__init__()

    def Detect(self,model_path):
        global drawing,last_point,background,canvas
        background = (200,200,1)
        canvas = np.ones(background, dtype=np.uint8)
        drawing = False
        last_point = (0, 0)
         # 创建窗口并设置回调函数
        cv2.namedWindow("Draw a Number")
        cv2.setMouseCallback("Draw a Number", draw_number)

        while True:
            cv2.imshow("Draw a Number", canvas)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('c'):
                canvas = np.ones(background, dtype=np.uint8)  
            elif key == ord('s'):
                loaded_model = torch.load(model_path)
                transform = transforms.Compose([transforms.ToTensor(), transforms.Normalize((0.5,), (0.5,))])
                input_image = transform(cv2.resize(canvas,(28,28)))
                input_image = input_image.unsqueeze(0) 
                with torch.no_grad():  # 在预测时不需要计算梯度
                    output = loaded_model(input_image)
                predicted_class = torch.argmax(output, dim=1).item()  # 获取预测结果的类别索引
                print('Predicted label:' ,output)
                print(f'Predicted number: {predicted_class}')
            elif key == ord('q'):
                break
    cv2.destroyAllWindows()