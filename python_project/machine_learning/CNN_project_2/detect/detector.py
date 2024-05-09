# cv2 numpy pytorch
import cv2
import numpy as np
import torch
import torchvision.transforms as transforms

# custom
from logger.logger import log


class Detector:
    def __init__(self):
        super(Detector, self).__init__()

    def Detect(self, model_path, input):
        loaded_model = torch.load(model_path)
        transform = transforms.Compose(
            [transforms.ToTensor(), transforms.Normalize((0.5,), (0.5,))]
        )
        input_image = transform(cv2.resize(input, (28, 28)))
        input_image = input_image.unsqueeze(0)
        with torch.no_grad():  
            output = loaded_model(input_image)
        predicted_class = torch.argmax(output, dim=1).item()
        result = [item for sublist in output.tolist() for item in sublist]
        log(f"Predicted label:{result}","DEBUG")
        log(f"Predicted number:{predicted_class}","DEBUG")
        return result
