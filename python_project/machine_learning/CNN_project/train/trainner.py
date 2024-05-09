import torch
from torchvision import datasets, transforms
import numpy as np
import pickle
import torch.nn as nn
import torch.optim as optim

# global data_path
data_path='./'

class SimpleCNN(torch.nn.Module):
    def __init__(self):
        super(SimpleCNN, self).__init__()
        self.conv1 = nn.Conv2d(1, 32, kernel_size=3, stride=1, padding=1)
        self.conv2 = nn.Conv2d(32, 64, kernel_size=3, stride=1, padding=1)
        self.pool = nn.MaxPool2d(kernel_size=2, stride=2, padding=0)
        self.fc1 = nn.Linear(64 * 7 * 7, 128)
        self.fc2 = nn.Linear(128, 10)

    def forward(self, x):
        x = self.pool(torch.relu(self.conv1(x)))
        x = self.pool(torch.relu(self.conv2(x)))
        x = x.view(-1, 64 * 7 * 7)  # 将特征图展平为一维向量
        x = torch.relu(self.fc1(x))
        x = self.fc2(x)
        return x
    
class Trainer():
    def __init__(self):
        super(Trainer, self).__init__()
    def Train(self,data_path):
        # 定义数据预处理步骤,将输入的图片转化为tensor类型并进行归一化
        transform = transforms.Compose([transforms.ToTensor(),transforms.Normalize((0.5,), (0.5,))])

        trainset = datasets.MNIST(data_path,True,transform)
        trainloader = torch.utils.data.DataLoader(trainset, batch_size=64, shuffle=True)

        testset = datasets.MNIST(data_path,train=False,transform=transform)
        testloader = torch.utils.data.DataLoader(testset, batch_size=64, shuffle=True)

        cnn=SimpleCNN()
        criterion = nn.CrossEntropyLoss()
        optimizer = optim.Adam(cnn.parameters(), lr=0.001)

        # 训练模型
        num_epochs = 50
        for epoch in range(num_epochs):
            running_loss = 0.0
            for i, data in enumerate(trainloader, 0):
                inputs, labels = data
                optimizer.zero_grad()
                outputs = cnn(inputs)
                loss = criterion(outputs, labels)
                loss.backward()
                optimizer.step()
                running_loss += loss.item()
                if i % 100 == 99:
                    print('[%d, %5d] loss: %.3f' %
                        (epoch + 1, i + 1, running_loss / 100))
                    running_loss = 0.0

        print('Finished Training')
        torch.save(cnn, 'model.pth')