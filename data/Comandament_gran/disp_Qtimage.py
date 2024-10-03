from PyQt5.QtWidgets import (QWidget, QHBoxLayout, QLabel, QApplication)
from PyQt5.QtGui import QPixmap
import sys


class Example(QWidget):

    def __init__(self):
        super().__init__()

        self.initUI()

    def initUI(self):
        hbox = QHBoxLayout(self)
        pixmap = QPixmap()
        pixmap.loadFromData(self.load_operation_that_might_fail(), 'jpg')

        lbl = QLabel(self)
        lbl.setPixmap(pixmap)

        hbox.addWidget(lbl)
        self.setLayout(hbox)

        self.setWindowTitle('Example')
        self.show()


    def load_operation_that_might_fail(self):
        with open('C:\Dades\Projectes\Actius\Rovinya\Mapes\maset.jpg', 'rb') as f:
            return f.read()


if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = Example()
    sys.exit(app.exec_())