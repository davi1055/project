from PyQt5.QtCore import QObject, pyqtSignal


class TextSignal(QObject):
    text_signal = pyqtSignal(str)

    def write(self, text):
        self.text_signal.emit(str(text))
