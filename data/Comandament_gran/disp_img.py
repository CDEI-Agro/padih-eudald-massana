import sys
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
import rasterio
import numpy as np
import geopandas as gpd
import utm
 
class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        self.centralwidget = QWidget(MainWindow)
        self.gridLayout = QGridLayout(self.centralwidget)
        self.scrollArea = QScrollArea(self.centralwidget)
        self.scrollArea.setWidgetResizable(True)
        self.scrollAreaWidgetContents = QWidget()
        self.gridLayout_2 = QGridLayout(self.scrollAreaWidgetContents)
        self.label = mouseLabel(self.scrollAreaWidgetContents)
        self.label.setText("")
        self.gridLayout_2.addWidget(self.label)
        self.scrollArea.setWidget(self.scrollAreaWidgetContents)
        self.gridLayout.addWidget(self.scrollArea)
        # self.graphicsView
        MainWindow.setCentralWidget(self.centralwidget)
 
 
class mouseLabel(QLabel):
      
    def __init__(self, parent=None):
        super(mouseLabel, self).__init__(parent)
        self.setMouseTracking(True)
        self.clicked = False
     
    def mousePressEvent(self, event):
        if event.button() == 1:
            self.clicked = True
            self.oldPos = event.globalPos()
 
    def mouseMoveEvent(self, event):
            if self.clicked:
                delta = QPoint(event.globalPos() - self.oldPos)
                self.move(self.x() + delta.x(), self.y() + delta.y())
                self.oldPos = event.globalPos()
 
    def mouseReleaseEvent(self, event):
        if event.button() == 1:
            self.clicked = False
  
  
class AppWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        fp = r'maset_2.tif'     # imatge georeferenciada geotiff
        img = rasterio.open(fp) 
        ima2 = img.read()
        ima2 = np.transpose(ima2, (1,2,0)).copy() # adecua la imatge per ser mostrada amb Qt
        print(ima2.shape)
        qimage = QImage(ima2, ima2.shape[1], ima2.shape[0], QImage.Format_RGB888)

        pixmap = QPixmap(qimage)

        features = gpd.read_file('coord_042.geojson')   # obre el fitxer de punts en format geojson
        print(features)
        for id, punt in features.iterrows():    # va llegint els punts
            print(id, punt.ord, punt.geometry.x, punt.geometry.y)
            u = utm.from_latlon(float(punt.geometry.y), float(punt.geometry.x)) # tranforma Latitut, longitut a coordenades UTM
            row, column = img.index(u[0], u[1]) # converteig les coordenades UTM a pixels de la imatge
            painter = QPainter()
            painter.begin(pixmap)
            verd = QBrush(Qt.green)
            negre = QPen(Qt.black)
            painter.setPen(negre)
            painter.setBrush(verd)
            painter.drawEllipse(column, row, 10,10) # pinta els punts a la imatge
            painter.end()

        self.ui.label.setPixmap(pixmap)
        self.ui.hsb = self.ui.scrollArea.horizontalScrollBar()
        self.resize(1000, 800)
        self.show()


def main():
    app = QApplication(sys.argv)
    w   = AppWindow()
    w.show()
    sys.exit(app.exec_())
  
  
if __name__ == '__main__':
    main()