# std
import sys

# pytorch
import torch

# custom
from logger.logger import log
from show import showConfusionMatrix


# test
def testModel(model_path, testloader):
    model = torch.load(model_path)
    correct = 0
    total = 0
    predicted_list = []
    labels_list = []
    with torch.no_grad():
        for data in testloader:
            images, labels = data
            outputs = model(images)
            _, predicted = torch.max(outputs, 1)
            predicted_list += predicted
            labels_list += labels
            total += labels.size(0)
            correct += (predicted == labels).sum().item()
    accuracy = 100 * correct / total
    log(f"Accuracy on test set: {accuracy}%", "DEBUG")
    showConfusionMatrix(labels_list, predicted_list)
    return accuracy
