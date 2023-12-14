import os
import random

from PyQt5 import QtWidgets, QtGui
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QWidget, QSizePolicy, QFileDialog
from PyQt5.QtGui import QFont

from qfluentwidgets import CaptionLabel, PushButton, LineEdit

from threads import GenerateThread


class GeneratorForm(QWidget):
    def __init__(self):
        super().__init__()
        self.model_path = ""
        self.seed = 1
        self.setupUI()

    def setupUI(self):
        self.setObjectName("GeneratorForm")
        self.setWindowTitle("生成图片")

        # 模型标签
        self.model_label = CaptionLabel("模型路径")
        self.model_label.resize(20, 15)
        self.model_label.setSizePolicy(
            QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Fixed
        )
        self.model_label.setFont(QFont("Hack Nerd Font", 12))

        # 模型路径输入框
        self.model_path_edit = LineEdit()
        self.model_path_edit.resize(30, 15)
        self.model_path_edit.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed
        )
        self.model_path_edit.setFont(QFont("Hack Nerd Font", 12))

        # 选择模型保存路径按钮
        self.select_model_path_button = PushButton("选择模型路径")
        self.select_model_path_button.clicked.connect(self.selectModelPath)
        self.select_model_path_button.resize(20, 15)
        self.select_model_path_button.setSizePolicy(
            QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Fixed
        )
        self.select_model_path_button.setFont(QFont("Hack Nerd Font", 12))

        # 水平布局1添加控件
        self.hlayout1 = QtWidgets.QHBoxLayout()
        self.hlayout1.addWidget(self.model_label)
        self.hlayout1.addWidget(self.model_path_edit)
        self.hlayout1.addWidget(self.select_model_path_button)

        # --------------------------------------------------------------------------------

        # 随机种子标签
        self.seed_label = CaptionLabel("随机种子")
        self.seed_label.resize(20, 15)
        self.seed_label.setSizePolicy(
            QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Fixed
        )
        self.seed_label.setFont(QFont("Hack Nerd Font", 12))

        # 随机种子输入框
        self.seed_edit = LineEdit()
        self.seed_edit.resize(10, 15)
        self.seed_edit.setFont(QFont("Hack Nerd Font", 12))
        self.seed_edit.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed
        )

        # 生成按钮
        self.generate_button = PushButton("生成图片")
        self.generate_button.resize(20, 15)
        self.generate_button.setSizePolicy(
            QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Fixed
        )
        self.generate_button.setFont(QFont("Hack Nerd Font", 12))
        self.generate_button.clicked.connect(self.generateNumber)

        # 水平布局2添加控件
        self.hlayout2 = QtWidgets.QHBoxLayout()
        self.hlayout2.addWidget(self.seed_label)
        self.hlayout2.addWidget(self.seed_edit)
        self.hlayout2.addWidget(self.generate_button)

        # --------------------------------------------------------------------------------

        # 图片展示标签
        self.image_label = QtWidgets.QLabel()
        self.image_label.setSizePolicy(
            QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Ignored
        )
        self.image_label.setScaledContents(True)

        # 设置窗口布局
        self.window_layout = QtWidgets.QVBoxLayout()
        self.window_layout.addLayout(self.hlayout1)
        self.window_layout.addLayout(self.hlayout2)
        self.window_layout.addWidget(self.image_label)
        self.setLayout(self.window_layout)

    def setImage(self, path: str):
        img_path = os.path.abspath(os.path.expandvars(path))
        pix = QtGui.QPixmap(img_path)
        self.image_label.setPixmap(pix)

    def getModelPath(self):
        return self.model_path_edit.currentText()

    def addModelPath(self, path: str):
        self.model_path_edit.addItem(path)

    def getSelectedNumber(self):
        return self.seed_edit.currentText()

    def selectModelPath(self):
        path = QFileDialog.getExistingDirectory(self, "选择模型", ".")
        self.model_path_edit.setText(path)

    def generateNumber(self):
        self.generate_button.setEnabled(False)

        self.model_path = self.model_path_edit.text()
        self.seed = self.seed_edit.text()
        self.config = {
            "mode": "generate",
            "model_save_path": "./models/",
            "dataset_path": "./dataset/",
            "GPU_nums": 0,
            "epochs": 1,
            "seed": self.seed,
        }

        self.generate_thread = GenerateThread(self.config)
        self.generate_thread.start()
        self.generate_thread.finished.connect(self.finishGenerate)

    def finishGenerate(self):
        self.setImage("./picture/test/generate_image_0.png")
        self.generate_button.setEnabled(True)
