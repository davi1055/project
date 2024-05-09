import torch
from torchvision import datasets, transforms
from sklearn import svm
from PIL import Image
import numpy as np
import pickle

class SimpleCNN(torch.nn.Module):
    def __init__(self):
        super(SimpleCNN, self).__init__()
        self.conv1 = torch.nn.Conv2d(1, 32, kernel_size=3)
        self.pool = torch.nn.MaxPool2d(2, 2)
        self.conv2 = torch.nn.Conv2d(32, 64, kernel_size=3)
        self.fc1 = torch.nn.Linear(64*5*5, 128)

    def forward(self, x):
        x = self.conv1(x)
        x = torch.relu(x)
        x = self.pool(x)
        x = self.conv2(x)
        x = torch.relu(x)
        x = self.pool(x)
        x = torch.flatten(x, 1)
        x = self.fc1(x)
        return x

# 定义数据预处理步骤,将输入的图片转化为tensor类型并进行归一化
transform = transforms.Compose([transforms.ToTensor(),transforms.Normalize((0.5,), (0.5,))])

trainset = datasets.MNIST('./MNIST_data/',True,transform)
trainloader = torch.utils.data.DataLoader(trainset, batch_size=64, shuffle=True)

testset = datasets.MNIST('./MNIST_data/',train=False,transform=transform)
testloader = torch.utils.data.DataLoader(testset, batch_size=64, shuffle=True)

# dataiter = iter(trainloader)
# images, labels = dataiter.__next__()

# # 将训练数据转换为二维数组，以适应SVM
# features = images.view(images.shape[0], -1)

clf = svm.SVC()
cnn=SimpleCNN()

features = []
labels = []
for images, targets in trainloader:
    with torch.no_grad():
        feat = cnn(images)
    features.append(feat.numpy())
    labels.append(targets.numpy())

features = np.concatenate(features)
labels = np.concatenate(labels)

# 训练SVM分类器
clf = svm.SVC()
# 使用训练数据和标签训练SVM分类器
clf.fit(features, labels)
with open('./svm_model.pickle', 'wb') as f:
    pickle.dump(clf, f)


