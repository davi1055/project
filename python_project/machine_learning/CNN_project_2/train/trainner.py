# pytorch
import torch
from torchvision import datasets, transforms
import torch.nn as nn
import torch.optim as optim

# custom
from logger.logger import log


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
        # Convert to one dimension
        x = x.view(-1, 64 * 7 * 7)
        x = torch.relu(self.fc1(x))
        x = self.fc2(x)
        return x


class Trainer:
    def __init__(self):
        self.cnn = SimpleCNN()

    def train(self, trainloader):
        criterion = nn.CrossEntropyLoss()
        optimizer = optim.Adam(self.cnn.parameters(), lr=0.001)

        # train
        num_epochs = 5
        for epoch in range(num_epochs):
            running_loss = 0.0
            for i, data in enumerate(trainloader, 0):
                inputs, labels = data
                optimizer.zero_grad()
                outputs = self.cnn(inputs)
                loss = criterion(outputs, labels)
                loss.backward()
                optimizer.step()
                running_loss += loss.item()
                if i % 100 == 99:
                    log(f"[epoch:{epoch+1}/{i+1}] loss:{running_loss:.2f}%", "DEBUG")
                    running_loss = 0.0
        log("Finished Training.", "DEBUG")

    def saveModel(self, path="./model.pth"):
        torch.save(self.cnn, path)
        log(f"Save {path}", "DEBUG")
