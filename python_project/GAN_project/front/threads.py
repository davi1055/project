from PyQt5.Qt import QThread
from back.main import main


class TrainThread(QThread):
    def __init__(self, config):
        super().__init__()
        self.config = config

    def run(self):
        print("开始训练\n")
        main(self.config)


class GenerateThread(QThread):
    def __init__(self, config):
        super().__init__()
        self.config = config

    def run(self):
        main(self.config)
