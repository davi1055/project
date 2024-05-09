from sklearn import datasets
from sklearn.model_selection import train_test_split
from sklearn.svm import SVC
from sklearn.metrics import accuracy_score
import joblib

def logD(msg):
    global is_debug
    if is_debug == True:
        print(f"\033[32m{msg}\033[0m")

def loadData(name,version,x_type,y_type,cache=True,parser='auto'):
    logD("开始加载数据集")
    mnist = datasets.fetch_openml(name,version=version,cache=cache,parser=parser)
    x = mnist.data.astype(x_type)
    y = mnist.target.astype(y_type)
    logD(f"{x.shape},{y.shape}")
    logD("加载完毕")
    return mnist,x,y

def trainSVM(x_train,y_train,kernel='rbf',C=1,gamma='scale'):
    logD("创建SVM模型")
    # 创建支持向量机（SVM）模型
    svm_classifier = SVC(kernel=kernel, C=C, gamma=gamma)
    # 训练SVM模型
    logD("SVM模型开始训练")
    svm_classifier.fit(x_train, y_train)
    logD("SVM模型训练结束")
    return svm_classifier

def testSVM(svm,x_test,y_test):
    logD("测试开始")
    # 预测并评估模型
    y_pred = svm.predict(x_test)
    accuracy = accuracy_score(y_test, y_pred)
    logD(f'准确度(SVM): {accuracy*100:.4f}%')

def saveSVM(svm,path = 'svm_model.pkl'):
    logD("保存SVM模型")
    joblib.dump(svm,path)



if __name__ == "__main__":
    is_debug = True

    # 加载MNIST数据集
    mnist,x,y = loadData('mnist_784',1,'float32','int')

    # 划分数据集为训练集和测试集
    x_train, x_test, y_train, y_test = train_test_split(x, y,test_size=0.2, random_state=42)

    # 数据标准化
    x_train /= 255.0
    x_test /= 255.0

    svm = trainSVM(x_train,y_train)

    testSVM(svm,x_test,y_test)

    saveSVM(svm)



