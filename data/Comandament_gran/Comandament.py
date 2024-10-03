
import sys
from PyQt5.QtWidgets import (QDialog, QApplication, QGraphicsScene, QGraphicsPixmapItem, QFileDialog,
                             QGraphicsEllipseItem, QGraphicsItem, QGestureEvent, QGesture, QGraphicsPolygonItem)
from PyQt5.QtGui import (QPixmap, QImage, QPainter, QPen, QBrush,  QPolygonF, QPainterPath, QTransform, QColor)
from PyQt5.QtCore import (Qt, QPoint, QDataStream, QIODevice, QThread, QPointF, QRect, QRectF, QAbstractAnimation,
                           QVariantAnimation,pyqtSignal, pyqtSlot)
from PyQt5.QtNetwork import QHostAddress, QTcpSocket, QAbstractSocket
from comandamentView import *
import rasterio
import numpy as np
import csv
import utm
import geopandas
import ManWorker


class MeuGraphicsScene(QGraphicsScene):
    puntSeleccionat = pyqtSignal(QPointF)

    def __init__(self, parent=None):
        QGraphicsScene.__init__(self, parent)
        #self.setSceneRect(-100, -100, 200, 200)

    def mousePressEvent(self, event):
        self.puntSeleccionat.emit(event.scenePos())
        # pen = QPen(Qt.black)
        # brush = QBrush(Qt.black)
        x = event.scenePos().x()
        y = event.scenePos().y()
        # print(event.type())
        # self.addEllipse(x, y, 4, 4, pen, brush)
        super().mousePressEvent(event)

class Ball(QGraphicsEllipseItem):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.anim = QVariantAnimation()
        self.setRect(QRectF(-3, -3, 6, 6))
        self.setPen(QPen())

    def move_to(self, pos, duration=100):
        if self.anim.state() == QAbstractAnimation.Running:
            self.anim.stop()
        sp = self.pos()
        self.anim.setStartValue(sp)
        self.anim.setEndValue(pos)
        self.anim.valueChanged.connect(self.setPos)
        self.anim.setDuration(duration)
        self.anim.start()

class Triangle(QGraphicsPolygonItem):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.anim = QVariantAnimation()
        self.setPolygon(QPolygonF([QPointF(0, 0), QPointF(-10, -10), QPointF(-10, 10)]))
        #self.setRect(QRectF(-5, -5, 10, 10))
        self.setPen(QPen())

    def move_to(self, pos, duration=100):
        if self.anim.state() == QAbstractAnimation.Running:
            self.anim.stop()
        self.anim.setStartValue(self.pos())
        self.anim.setEndValue(pos)
        self.anim.valueChanged.connect(self.setPos)
        self.anim.setDuration(duration)
        self.anim.start()


class Arrow(QGraphicsPolygonItem):
    def __init__(self, parent = None):
        super().__init__(parent)
        self.anim = QVariantAnimation()
        self.setFlag(QGraphicsItem.ItemIsSelectable, True)
        self.setFlag(QGraphicsItem.ItemIsFocusable, True)
        self.setPen(QPen(Qt.black))
        self.setBrush(QBrush(Qt.blue, Qt.SolidPattern))

    def move_to(self, pos, angle=0.0):
        head_polygon=QPolygonF([QPointF(0, 0), QPointF(5, -3), QPointF(0, 8), QPointF(-5, -3)])
        head_transform = QTransform()
#        head_transform.translate(pos.x(), pos.y())
        head_transform.rotate((angle + 180) % 360)
        self.head = head_transform.map(head_polygon)
        self.setPolygon(self.head)
        if self.anim.state() == QAbstractAnimation.Running:
            self.anim.stop()
        self.anim.setStartValue(self.pos())
        self.anim.setEndValue(pos)
        self.anim.valueChanged.connect(self.setPos)
        self.anim.setDuration(10)
        self.anim.start()

class Comandament(QDialog):

    punts = []
    rutaactual = []
    puntinici = False
    puntfi = False
    reginici = None
    regfi = None
    interval = 20
    angleactual = 0.0
    pintaPosicioActual = False
    bola = Ball()
    desti = Ball()
    fletxa = Arrow()
    triangle = Triangle()
    mouAuto = False
    centrat = False

    def __init__(self):
        super().__init__()
        try:
            self.ui = Ui_Dialog()
            self.ui.setupUi(self)
            # self.scene = QGraphicsScene(self)
            self.scene = MeuGraphicsScene(self)
            self.ui.mesButton.clicked.connect(self.zoomMes)
            self.ui.menysButton.clicked.connect(self.zoomMenys)
            self.ui.obreMapaButton.clicked.connect(self.obreMapa)
            self.ui.obrePuntsButton.clicked.connect(self.obrePunts)
            self.ui.netejaButton.clicked.connect(self.neteja)
            self.ui.amuntButton.clicked.connect(self.amunt)
            self.ui.avallButton.clicked.connect(self.avall)
            self.ui.esquerraButton.clicked.connect(self.esquerra)
            self.ui.dretaButton.clicked.connect(self.dreta)
            self.ui.camiButton.clicked.connect(self.fescami)
            #self.ui.connectaButton.clicked.connect(self.connecta)
            self.ui.enviaButton.clicked.connect(self.enviaRuta)
            self.ui.posicioCB.clicked.connect(self.pintaPos)
            self.ui.puls2.setText('Mou Auto')
            self.ui.puls1.setText('+ coor.')

            self.ui.graphicsView.setScene(self.scene)
            self.scene.puntSeleccionat.connect(self.on_positionChanged)
            # fp = r'bcn.tif'
            fp = r'maset_2.tif'
            # fp = r'major33-2.tif'
            self.img = rasterio.open(fp)
            ima2 = self.img.read()
            ima2 = np.transpose(ima2, (1,2,0)).copy()
            # print(ima2.shape)

            self.qimage = QImage(ima2, ima2.shape[1], ima2.shape[0], QImage.Format_RGB888)
            self.imalayer = QGraphicsPixmapItem(QPixmap(self.qimage))
            self.drawlayer =  QGraphicsPixmapItem(QPixmap(self.qimage.size()))
            self.neteja()
            self.scene.addItem(self.imalayer)
            self.scene.addItem(self.drawlayer)
            self.bola.setBrush(QBrush(QColor(200, 200, 200)))
            self.scene.addItem(self.bola)
            self.desti.setBrush(QBrush(QColor(250, 250, 0)))
            self.scene.addItem(self.desti)
            self.scene.addItem(self.fletxa)

            # Step 2: Create a QThread object
            self.manthread = QThread()
            # Step 3: Create a worker object
            self.manworker = ManWorker.ManWorker()
            # Step 4: Move worker to the thread
            self.manworker.moveToThread(self.manthread)
            # Step 5: Connect signals and slots
            self.manthread.started.connect(self.manworker.run)
            self.manworker.finished.connect(self.manthread.quit)
            self.manworker.finished.connect(self.manworker.deleteLater)
            self.manthread.finished.connect(self.manthread.deleteLater)
            # Step 6: Start the thread
            self.manthread.start()

            self.pintaPosicioActual = False
            #self.manworker.manval.connect(self.manVal)
            self.manworker.vbateria.connect(self.vBat)
            self.manworker.rebut.connect(self.rebut)
            self.manworker.joypos.connect(self.dispJoypos)
            self.temapa = True

        except Exception as ex:
            print("Err: %s" % ex)
            
        
    @pyqtSlot(QPointF)
    def on_positionChanged(self, pos):
        # delta = QPoint(30, -15)
        # self.label_position.show()
        # self.label_position.move(pos + delta)
        # self.mousePos.setText("(%f, %f)" % (pos.x(), pos.y()))
        # self.label_position.adjustSize()
        rect = QRectF(pos.x()-5, pos.y()-5, 10, 10)
        posicio = QPointF(pos.x(),pos.y())
        if self.pintaPosicioActual is True:
            #self.bola.move_to(posicio,100)
            self.angleactual = (self.angleactual + 45.0) % 360.0
            self.fletxa.move_to(posicio,self.angleactual)
            #print(self.angleactual)
            #self.triangle.move_to(posicio,100)
        for reg in self.punts:
            punt = QPointF(reg[3],reg[4])
            if rect.contains(punt):
                if self.puntinici is True:
                    self.bola.setBrush(QBrush(QColor(0, 250, 0)))
                    self.bola.move_to(punt,100)
                    self.reginici = reg
                    self.puntinici = False
                    self.puntfi = True
                    self.ui.camiButton.setStyleSheet("background-color: red")
                    self.ui.camiButton.setText("Fi")
                    #self.ui.fiButton.setStyleSheet("background-color: green")
                    self.desti.setBrush(QBrush(QColor(0, 250, 0)))
                elif self.puntfi is True:
                    self.desti.move_to(punt,100)
                    self.regfi = reg
                    self.puntinici = False
                    self.puntfi = False
                    self.ui.camiButton.setStyleSheet("background-color: none")
                    self.ui.camiButton.setText("Inici")
                    #self.ui.fiButton.setStyleSheet("background-color: none")
                    self.desti.setBrush(QBrush(QColor(0, 200, 200)))
                    self.fesrecorregut(self.reginici,self.regfi)
                else:
                    print(reg)
                self.desti.move_to(punt,100)
                break

    def fesrecorregut(self,rini,rfi):
        '''Envia el recorregut marcat a la llista i al displayList'''
        print(rini,rfi)
        if int(rini[0]) >= int(rfi[0]):
            print("Error: ordre inicial major que ordre final")
            return
        self.rutaactual.clear()
        ract = 0
        for reg in self.punts:
            if int(reg[0]) >= int(rini[0]) and int(reg[0]) <= int(rfi[0]):
                print(reg)
                ract += 1
                #self.pintaPunt(reg[2],reg[1],self.drawlayer)
                self.pintaPunt(reg[2],reg[1])
                # self.insertaLlista.emit("%d, %0.8f, %0.8f" %(ract, reg[1], reg[2]))
                #racts = "%s" %ract
                self.rutaactual.append((ract, reg[1], reg[2]))
        print(self.rutaactual)

    def fescami(self):
        self.puntinici = True
        self.puntfi = False
        self.ui.camiButton.setStyleSheet("background-color: green")

    def dispJoypos(self,val):
        # self.ui.coorMsg.setText(val)
        self.ui.joypos.setText(val)

    def vBat(self,val):
        self.ui.vbat.setText(val)

    def rebut(self, cmdStr):
        # print("Rebut|%s|\n" % cmdStr)
        global rotary_value
        try:
            if cmdStr.startswith('$CO'):
                cmd, lon, lat, angle, q, speed = cmdStr.split( ',' )
                # print(lon,lat)
                self.ui.KmhNumber.display(speed)
                if self.pintaPosicioActual is True:
                    self.neteja()
                    self.pintaRobot(float(lon), float(lat), float(angle))

            if cmdStr.startswith('$ST'):
                cmd, auto, inter = cmdStr.split(',')
                if auto == 'A' and self.mouAuto == False:
                    self.mouAuto = True
                if auto == 'a' and self.mouAuto == True:
                    self.movAuto = False
                    rotary_value = 0
                    self.manworker.e1.resetValue()
                # print(inter)
                if inter == 'I':
                    self.ui.puls6.setStyleSheet("background-color: red")
                else:
                    self.ui.puls6.setStyleSheet("background-color: white")
            self.ui.coorMsg.setText(cmdStr)
        except:
            self.ui.errorMsg.setText('Err: resposta')
        #self.ui.coorMsg.setText(str)
        
    def pintaPos(self):
        '''Check box Pinta posicio'''
        if self.ui.posicioCB.isChecked():
            self.pintaPosicioActual = True
        else:
            self.pintaPosicioActual = False
            self.centrat = False    

    def pintaRobot(self, lon, lat, angle):
        if self.temapa is False:
            return
        u = utm.from_latlon(lat, lon)
        row, column = self.img.index(u[0], u[1])
        # print("row %d, column %d" % (row, column))
        posicio = QPointF(column,row)
        self.fletxa.move_to(posicio,angle)
        if(self.centrat != True):
            self.ui.graphicsView.fitInView(column-250,row-150,500,300, Qt.KeepAspectRatio)
            self.centrat = True

    def neteja(self):
        pixmap = self.drawlayer.pixmap()
        pixmap.fill(Qt.transparent)
        self.drawlayer.setPixmap(pixmap)

    def pintaPunt(self, lat, lon):
        u = utm.from_latlon(lat, lon)
        row, column = self.img.index(u[0], u[1])
        pixmap = self.drawlayer.pixmap()
        #pixmap.fill(Qt.transparent)
        painter = QPainter()
        painter.begin(pixmap)
        # verd = QBrush(Qt.green)
        negre = QPen(Qt.black)
        painter.setPen(negre)
        painter.setBrush(Qt.blue)
        painter.drawEllipse(column-3, row-3, 6,6)
        painter.end()
        self.drawlayer.setPixmap(pixmap)

    def enviaRuta(self):
        if self.punts :
            if self.manworker.connexio.available():
                reg = b'$RU,INI\n'
                self.manworker.connexio.envia(reg)
                #ManWorker.Connexio.envia(reg)
                for row in self.punts:
                    reg = b'$RU,%d,%0.8f,%0.8f\n' % ( row[0], row[1], row[2])
                    self.manworker.connexio.envia(reg)
                reg = b'$RU,FI\n'
                self.manworker.connexio.envia(reg)

    def obrePunts(self):
        options = QFileDialog.Options()
        options |= QFileDialog.DontUseNativeDialog
        fileName, _ = QFileDialog.getOpenFileName(self,"Obre fitxer de Punts", ".","Fitxers CSV (*.csv);;Tots (*)", options=options)
        if fileName:
            # print(fileName)
            pixmap = self.drawlayer.pixmap()
            #pixmap.fill(Qt.transparent)
            painter = QPainter()
            painter.begin(pixmap)
            verd = QBrush(Qt.red)
            negre = QPen(Qt.black)
            painter.setPen(negre)
            painter.setBrush(verd)
            conta = 0

            if fileName.find("geojson") == -1:
            #     self.fitxer.emit(fileName)
                self.punts.clear()
                with open(fileName, newline='') as csvfile:
                    coordreader = csv.reader(csvfile) #, delimiter=' ', quotechar='|')
                    for row in coordreader:
                        a = row[0]
                        b = row[1]
                        c = row[2]
                        if a.isnumeric() is False: # Saltem la primera linia
                            continue
                        ordi = int(a)
                        lon = float(b)  #Longitud
                        lat = float(c)  #Latitud
                        u = utm.from_latlon(lat, lon)
                        row, column = self.img.index(u[0], u[1])
                        painter.setPen(negre)
                        painter.drawEllipse(column-3, row-3, 6,6)
                        if conta != 0:
                            painter.setPen(Qt.red)
                            painter.drawLine(column, row, columnant, rowant)
                        conta += 1
                        columnant = column
                        rowant = row
                        self.punts.append((ordi,lon,lat,column,row))
            else:
                features = geopandas.read_file(fileName)
                print(features)
                for id, punt in features.iterrows():
                    u = utm.from_latlon(punt.geometry.y, punt.geometry.x)
                    row, column = self.img.index(u[0], u[1])
                    painter.setPen(negre)
                    painter.drawEllipse(column-3, row-3, 6,6)
                    if conta != 0:
                        painter.setPen(Qt.red)
                        painter.drawLine(column, row, columnant, rowant)
                    conta += 1
                    columnant = column
                    rowant = row
                    self.punts.append((punt.ord,punt.geometry.x,punt.geometry.y,column,row))
            painter.end()
            self.drawlayer.setPixmap(pixmap)
            self.centraImatge()

    def centraImatge(self):
        reg = self.punts[0]
        column = reg[3]
        row = reg[4]
        self.ui.graphicsView.fitInView(self.drawlayer, Qt.KeepAspectRatio)
        # self.mapaView.verticalScrollBar().setValue(row)
        # self.mapaView.horizontalScrollBar().setValue(column)

    def obreMapa(self):
        options = QFileDialog.Options()
        options |= QFileDialog.DontUseNativeDialog
        fileName, _ = QFileDialog.getOpenFileName(self,"Obre mapa", ".","Fitxers Tiff (*.tif);;Tots (*)", options=options)
        if fileName:
            # fp = r'bcn.tif'
            # fp = r'maset_2.tif'
            # fp = r'major33-2.tif'
            self.img = rasterio.open(fileName)
            ima2 = self.img.read()
            ima2 = np.transpose(ima2, (1,2,0)).copy()
            # print(ima2.shape)

            self.qimage = QImage(ima2, ima2.shape[1], ima2.shape[0], QImage.Format_RGB888)
            self.imalayer = QGraphicsPixmapItem(QPixmap(self.qimage))
            self.drawlayer =  QGraphicsPixmapItem(QPixmap(self.qimage.size()))
            self.neteja()
            self.scene.addItem(self.imalayer)
            self.scene.addItem(self.drawlayer)


    def zoomMes(self):
        scale = 1.25
        self.ui.graphicsView.scale(scale, scale)

    def zoomMenys(self):
        scale = 0.75
        self.ui.graphicsView.scale(scale, scale)

    def amunt(self):
        self.ui.graphicsView.verticalScrollBar().setValue(self.ui.graphicsView.verticalScrollBar().value() + self.interval)

    def avall(self):
        self.ui.graphicsView.verticalScrollBar().setValue(self.ui.graphicsView.verticalScrollBar().value() - self.interval)
        
    def esquerra(self):
        self.ui.graphicsView.horizontalScrollBar().setValue(self.ui.graphicsView.horizontalScrollBar().value() + self.interval)

    def dreta(self):
        self.ui.graphicsView.horizontalScrollBar().setValue(self.ui.graphicsView.horizontalScrollBar().value() - self.interval)

if __name__=="__main__":
    app = QApplication(sys.argv)
    myapp = Comandament()
    #myapp.show()
    myapp.showMaximized()
    # myapp.showFullScreen()
    sys.exit(app.exec_())