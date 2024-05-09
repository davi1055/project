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

cnn=SimpleCNN()
clf = svm.SVC()

# 加载模型
with open('./svm_model.pickle', 'rb') as f:
    clf = pickle.load(f)

# 定义数据预处理步骤,将输入的图片转化为tensor类型并进行归一化
transform = transforms.Compose([transforms.ToTensor(),transforms.Normalize((0.5,), (0.5,))])

trainset = datasets.MNIST('./MNIST_data/',True,transform)
trainloader = torch.utils.data.DataLoader(trainset, batch_size=64, shuffle=True)

testset = datasets.MNIST('./MNIST_data/',train=False,transform=transform)
testloader = torch.utils.data.DataLoader(testset, batch_size=64, shuffle=True)

dataiter_test = iter(testloader)
images_test, labels_test = dataiter_test.__next__()

features_test = images_test.view(images_test.shape[0], 1, 28 ,28)
with torch.no_grad():
    features_test = cnn(features_test)
predictions = clf.predict(features_test)

#计算预测准确率
accuracy = sum(int(a == y) for a, y in zip(predictions, labels_test)) / float(len(predictions))
print(f'Accuracy: {accuracy}')

image = Image.open('./test_picture/num.jpg')
image = image.convert('L')
image = image.resize((28, 28))
image = np.array(image) / 255.0

image = torch.from_numpy(image).float()

image = image.view(1, 1, 28, 28)
with torch.no_grad():
    feat = cnn(image)

# 使用模型进行预测
output = clf.predict(feat)
# 使用模型获取置信度
decision_values = clf.decision_function(feat)

print('Predicted label:' ,output)
print('Decision values:',decision_values)