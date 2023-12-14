import sys
import random

from PyQt5 import QtWidgets
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QWidget, QSizePolicy, QFileDialog
from PyQt5.QtGui import QFont

from qfluentwidgets import CaptionLabel, PushButton, LineEdit, TextEdit

from signals import TextSignal
from threads import TrainThread


class TrainForm(QWidget):
    def __init__(self):
        super().__init__()
        self.model_save_path = ""
        self.dataset_path = ""
        self.epochs = 1
        self.GPU_nums = 0

        sys.stdout = TextSignal(text_signal=self.outputWrite)
        sys.stderr = TextSignal(text_signal=self.outputWrite)
        self.setupUI()

    def setupUI(self):
        self.setObjectName("训练模型")

        # 模型路径标签
        self.model_path_label = CaptionLabel("模型保存路径")
        self.model_path_label.resize(20, 15)
        self.model_path_label.setSizePolicy(
            QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Fixed
        )
        self.model_path_label.setFont(QFont("Hack Nerd Font", 12))

        # 模型路径输入框
        self.model_path_edit = LineEdit()
        self.model_path_edit.resize(20, 15)
        self.model_path_edit.setSizePolicy(
            QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Fixed
        )
        self.model_path_edit.setFont(QFont("Hack Nerd Font", 12))

        # 选择模型保存路径按钮
        self.select_save_path_button = PushButton("选择模型保存路径")
        self.select_save_path_button.clicked.connect(self.selectSavePath)
        self.select_save_path_button.resize(20, 15)
        self.select_save_path_button.setSizePolicy(
            QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Fixed
        )
        self.select_save_path_button.setFont(QFont("Hack Nerd Font", 12))

        # 水平布局1
        self.hlayout1 = QtWidgets.QHBoxLayout()
        self.hlayout1.addWidget(self.model_path_label)
        self.hlayout1.addWidget(self.model_path_edit)
        self.hlayout1.addWidget(self.select_save_path_button)

        # --------------------------------------------------------------------------------

        # 数据集路径标签
        self.dataset_path_label = CaptionLabel("数据集路径")
        self.dataset_path_label.resize(20, 15)
        self.dataset_path_label.setSizePolicy(
            QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Fixed
        )
        self.dataset_path_label.setFont(QFont("Hack Nerd Font", 12))

        # 模型路径选框
        self.dataset_path_edit = LineEdit()
        self.dataset_path_edit.resize(20, 15)
        self.dataset_path_edit.setSizePolicy(
            QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Fixed
        )
        self.dataset_path_edit.setFont(QFont("Hack Nerd Font", 12))

        # 添加模型保存路径按钮
        self.select_dataset_path_button = PushButton("选择数据集路径")
        self.select_dataset_path_button.clicked.connect(self.selectDataset)
        self.select_dataset_path_button.resize(20, 15)
        self.select_dataset_path_button.setSizePolicy(
            QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Fixed
        )
        self.select_dataset_path_button.setFont(QFont("Hack Nerd Font", 12))

        # 水平布局2
        self.hlayout2 = QtWidgets.QHBoxLayout()
        self.hlayout2.addWidget(self.dataset_path_label)
        self.hlayout2.addWidget(self.dataset_path_edit)
        self.hlayout2.addWidget(self.select_dataset_path_button)

        # --------------------------------------------------------------------------------

        # 循环数标签
        self.epochs_label = CaptionLabel("循环次数")
        self.epochs_label.resize(20, 15)
        self.epochs_label.setSizePolicy(
            QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Fixed
        )
        self.epochs_label.setFont(QFont("Hack Nerd Font", 12))

        # 循环数选框
        self.epochs_edit = LineEdit()
        self.epochs_edit.resize(20, 15)
        self.epochs_edit.setSizePolicy(
            QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Fixed
        )
        self.epochs_edit.setFont(QFont("Hack Nerd Font", 12))

        # GPU数标签
        self.GPU_label = CaptionLabel("GPU数量")
        self.GPU_label.resize(20, 15)
        self.GPU_label.setSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Fixed)
        self.GPU_label.setFont(QFont("Hack Nerd Font", 13))

        # GPU数选框
        self.GPU_edit = LineEdit()
        self.GPU_edit.resize(20, 15)
        self.GPU_edit.setSizePolicy(
            QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Fixed
        )
        self.GPU_edit.setFont(QFont("Hack Nerd Font", 12))

        # 添加模型保存路径按钮
        self.train_button = PushButton("开始训练")
        self.train_button.clicked.connect(self.train)
        self.train_button.resize(20, 15)
        self.train_button.setSizePolicy(
            QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Fixed
        )
        self.train_button.setFont(QFont("Hack Nerd Font", 12))

        # 水平布局3
        self.hlayout3 = QtWidgets.QHBoxLayout()
        self.hlayout3.addWidget(self.epochs_label)
        self.hlayout3.addWidget(self.epochs_edit)
        self.hlayout3.addWidget(self.GPU_label)
        self.hlayout3.addWidget(self.GPU_edit)
        self.hlayout3.addWidget(self.train_button)

        # --------------------------------------------------------------------------------

        # 信息显示框
        self.message_edit = TextEdit()
        self.message_edit.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding
        )
        self.message_edit.setFocusPolicy(Qt.FocusPolicy.NoFocus)

        # --------------------------------------------------------------------------------

        # 窗体布局
        self.window_layout = QtWidgets.QVBoxLayout()
        self.window_layout.addLayout(self.hlayout1)
        self.window_layout.addLayout(self.hlayout2)
        self.window_layout.addLayout(self.hlayout3)
        self.window_layout.addWidget(self.message_edit)
        self.setLayout(self.window_layout)

    def selectSavePath(self):
        path = QFileDialog.getExistingDirectory(self, "选择模型保存路径", ".")
        self.model_path_edit.setText(path)

    def selectDataset(self):
        path = QFileDialog.getExistingDirectory(self, "选择数据集", ".")
        self.dataset_path_edit.setText(path)

    def outputWrite(self, text):
        self.message_edit.insertPlainText(text)

    def train(self):
        self.train_button.setEnabled(False)
        self.message_edit.clear()

        self.model_save_path = self.model_path_edit.text()
        self.dataset_path = self.dataset_path_edit.text()
        self.GPU_nums = self.GPU_edit.text()
        self.epochs = self.epochs_edit.text()

        self.config = {
            "mode": "train",
            "model_save_path": self.model_save_path,
            "dataset_path": self.dataset_path,
            "GPU_nums": int(self.GPU_nums),
            "epochs": int(self.epochs),
            "seed": random.randint(1, 10000),
        }

        self.train_thread = TrainThread(self.config.copy())
        self.train_thread.start()
        self.train_thread.finished.connect(self.finishTrain)

    def finishTrain(self):
        self.train_button.setEnabled(True)
        print("训练结束")
