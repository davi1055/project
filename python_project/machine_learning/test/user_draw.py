import cv2
import numpy as np
import joblib

def useSVM(module,feat):
    # 使用模型进行预测
    output = module.predict(feat)
    # 使用模型获取置信度
    decision_values = module.decision_function(feat)
    print('Predicted label:' ,output)
    print('Decision values:',decision_values)


# 回调函数，用于绘制数字
def draw_number(event, x, y, flags, param):
    global drawing, last_point
    if event == cv2.EVENT_LBUTTONDOWN:
        drawing = True
        last_point = (x, y)
    elif event == cv2.EVENT_MOUSEMOVE:
        if drawing:
            cv2.line(canvas, last_point, (x, y), draw_color, 10)
            last_point = (x, y)
    elif event == cv2.EVENT_LBUTTONUP:
        drawing = False

if __name__=="__main__":
    background = (100,100,1)
    # 创建一个白色的画布
    canvas = np.ones(background, dtype=np.uint8)
    # 设置绘制的颜色
    draw_color = (255, 255,255)
    # 初始化绘制状态
    drawing = False
    last_point = (0, 0)

    # 创建窗口并设置回调函数
    cv2.namedWindow("Draw a Number")
    cv2.setMouseCallback("Draw a Number", draw_number)

    module = joblib.load("./svm_model.pkl")

    while True:
        cv2.imshow("Draw a Number", canvas)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('c'):
            canvas = np.ones(background, dtype=np.uint8)  
        elif key == ord('s'):
            image = cv2.resize(canvas,(28, 28))
            vector = image.reshape(-1)/255.0
            features=np.array([vector], dtype=np.float32)
            useSVM(module,features)
        elif key == ord('q'):
            break
    cv2.destroyAllWindows()

