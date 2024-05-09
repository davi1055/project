from torchvision import datasets, transforms
from sklearn import svm, metrics
from sklearn.model_selection import train_test_split
import torch
import pickle

# 定义数据预处理步骤，将输入的图片转化为tensor类型并进行归一化
transform = transforms.Compose([transforms.ToTensor(),transforms.Normalize((0.5,), (0.5,))])

# 加载MNIST数据集
trainset = datasets.MNIST('./MNIST_data/', download=True, train=True, transform=transform)
testset = datasets.MNIST('./MNIST_data/', download=True, train=False, transform=transform)

# 数据预处理，将图片转换为一维向量
X_train = trainset.data.numpy().reshape((len(trainset), -1))
y_train = trainset.targets.numpy()

X_test = testset.data.numpy().reshape((len(testset), -1))
y_test = testset.targets.numpy()

# 创建SVM分类器
# clf = svm.SVC(gamma=0.001)
clf = svm.SVC()

# 使用训练数据和标签训练分类器
clf.fit(X_train, y_train)
with open('./model/svm_model.pickle', 'wb') as f:
    pickle.dump(clf, f)
    
# 使用测试数据进行预测
predicted = clf.predict(X_test)