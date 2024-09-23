import sys
import globals
import MapaView


from PyQt5.QtWidgets import (
    QApplication, QDialog, QFileDialog, QMainWindow, QMessageBox,
      QPushButton, QGraphicsScene, QGraphicsPixmapItem, QGraphicsView, QLabel,
      QCheckBox, QGraphicsEllipseItem, QGraphicsItem
)


app = QApplication(sys.argv)
w = MapaView.Mapa()
globals.iniglob()
w.show()
sys.exit(app.exec_())
