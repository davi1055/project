import sys

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QWidget, QApplication

from qfluentwidgets import FluentWindow, FluentIcon

from front.generator_form import GeneratorForm
from front.train_form import TrainForm


class MainWindow(FluentWindow):
    def __init__(self):
        super().__init__()

        self.train_form = TrainForm()
        self.addSubInterface(self.train_form, FluentIcon.PLAY, "训练模型")

        self.generator_form = GeneratorForm()
        self.addSubInterface(self.generator_form, FluentIcon.PHOTO, "生成图片")


if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.aboutToQuit.connect(app.deleteLater)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
