import torch
import torch.nn as nn
import torch.optim as optim
import torchvision
import torchvision.transforms as transforms
import matplotlib.pyplot as plt
from PIL import Image
import numpy as np

# 定义神经网络模型
class SimpleNN(nn.Module):
    def __init__(self):
        super(SimpleNN, self).__init__()
        self.fc = nn.Sequential(
            nn.Linear(28 * 28, 128),
            nn.ReLU(),
            nn.Linear(128, 10)
        )

    def forward(self, x):
        x = x.view(x.size(0), -1)
        x = self.fc(x)
        return x

# 加载MNIST数据集
transform = transforms.Compose([transforms.ToTensor(), transforms.Normalize((0.5,), (0.5,))])
trainset = torchvision.datasets.MNIST(root='./data', train=True, download=True, transform=transform)
trainloader = torch.utils.data.DataLoader(trainset, batch_size=32, shuffle=True)

# 定义模型、损失函数和优化器
net = SimpleNN()
criterion = nn.CrossEntropyLoss()
optimizer = optim.SGD(net.parameters(), lr=0.001, momentum=0.9)

# 训练网络
num_epochs = 10
losses = []

for epoch in range(num_epochs):
    running_loss = 0.0
    for i, data in enumerate(trainloader, 0):
        inputs, labels = data
        optimizer.zero_grad()
        outputs = net(inputs)
        loss = criterion(outputs, labels)
        loss.backward()
        optimizer.step()

        running_loss += loss.item()
        if i % 1000 == 999:  # 每1000个小批量数据打印一次损失
            print('[%d, %5d] loss: %.3f' % (epoch + 1, i + 1, running_loss / 1000))
            running_loss = 0.0
    losses.append(running_loss)

print('Finished Training')

# # 绘制损失曲线
# plt.plot(losses)
# plt.title('Training Loss')
# plt.xlabel('Epoch')
# plt.ylabel('Loss')
# plt.show()

image = Image.open('./test_picture/1.jpg')
image = image.convert('L')
image = image.resize((28, 28))
image = np.array(image) / 255.0

# 将NumPy数组转换为PyTorch张量# 定义数据预处理步骤,将输入的图片转化为tensor类型并进行归一化
image = torch.from_numpy(image)

image = image.view(1, 1, 28, 28)

# 将四维张量转换为二维数组
image = image.view(1, -1)

# 使用模型进行预测
output = clf.predict(image)
# 使用模型获取置信度
decision_values = clf.decision_function(image)

print('Predicted label:' ,output)
print('Decision values:',decision_values)