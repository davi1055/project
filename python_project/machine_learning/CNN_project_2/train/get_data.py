# std
import os
import time
import random
import struct

# cv2 pytorch numpy
import cv2
import torch
from torchvision import datasets, transforms
import numpy as np

# custom
from logger.logger import log


class MnistData:
    def __init__(self):
        self.transform = transforms.Compose(
            [transforms.ToTensor(), transforms.Normalize((0.5,), (0.5,))]
        )
        self.train_dataset = datasets.MNIST(
            root="./data", train=True, transform=self.transform, download=True
        )
        self.test_dataset = datasets.MNIST(
            root="./data", train=False, transform=self.transform
        )

    def randomFillSize(self, range_min, range_max):
        random.seed(time.time())
        return random.randint(range_min, range_max)

    def zeroFill(self, image, fill_size):
        random_bool_list = [self.randomFillSize(0, 1) for _ in range(4)]
        randow_var_list = [ self.randomFillSize(0, fill_size[0] // 2) for _ in range(4) ]
        top, bottom, left, right = [a * b for a, b in zip(random_bool_list,randow_var_list)]
        padded_image = cv2.copyMakeBorder(
            image, top, bottom, left, right, cv2.BORDER_CONSTANT, value=(0, 0, 0)
        )
        padded_image = cv2.resize(padded_image, fill_size)
        return padded_image

    def getLoader(self, batch_size=64):
        self.trainloader = torch.utils.data.DataLoader(
            self.train_dataset, batch_size=batch_size, shuffle=True
        )
        self.testloader = torch.utils.data.DataLoader(
            self.test_dataset, batch_size=batch_size, shuffle=True
        )
        return self.trainloader, self.testloader

    def tensorToImage(self, tensor):
        numpy_array = tensor.numpy()
        image = numpy_array[0, :, :]
        return image

    def toZeroFillMnist(self, fill_size, is_show=False, train=True):
        if train:
            dataset = self.train_dataset
        else:
            dataset = self.test_dataset
        # torch_tensor:1x28x28
        for i in range(len(dataset)):
            torch_tensor, _ = dataset[i]
            log(f"b:{dataset.data[i]}","UNKNOW")
            src = self.tensorToImage(torch_tensor)
            dst = self.zeroFill(src, fill_size)
            # change
            dataset.data[i] = torch.from_numpy(dst)*255.0
            log(f"b:{dataset.data[i]}","UNKNOW")

            if not is_show:
                continue
            # show
            src = cv2.resize(src, (300, 300))
            dst = cv2.resize(dst, (300, 300))
            cv2.imshow("src", src)
            cv2.imshow("dst", dst)
            if cv2.waitKey(1) & 0xFF == ord("s"):
                cv2.waitKey(0)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                cv2.destroyAllWindows()
                is_show = False
