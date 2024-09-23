import os
from typing import Any
from numpy import interp
from serial import Serial
import globals
import rasterio
import numpy as np
import csv
import geopandas
import utm

from PyQt5.QtWidgets import (
    QApplication, QDialog, QFileDialog, QMainWindow, QMessageBox, 
      QPushButton, QGraphicsScene, QGraphicsPixmapItem, QGraphicsView, QLabel,
      QCheckBox, QGraphicsEllipseItem, QGraphicsItem, QGestureEvent, QGesture, QGraphicsPolygonItem
)

from PyQt5.QtCore import (Qt, QObject, QThread, pyqtSignal, pyqtSlot, QPoint, QEvent, QPointF, QRect,
    QRectF, QAbstractAnimation, QVariantAnimation, QLineF
)
from PyQt5.QtGui import QPixmap, QImage, QPainter, QPen, QBrush, QPainterPath, QColor, QPolygonF, QTransform, QFont
# from Control_motor_ui import Ui_Control

from Mapa_ui import Ui_Mapa

from PyQt5.uic import loadUi

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

class Mapa(QDialog):
    insertaLlista = pyqtSignal(str)
    potenviar = pyqtSignal()
    netejaLlista = pyqtSignal()
    mouauto = pyqtSignal()

    puntinici = False
    puntfi = False
    reginici = None
    regfi = None

    punts = []
    temapa = False
    interval = 40
    angleactual = 0.0

    def __init__(self):
        super(Mapa, self).__init__()
        self.ui=Ui_Mapa()
        self.ui.setupUi(self)
        self.ui.obreMapaButton.clicked.connect(self.obreMapa)
        self.ui.obrePuntsButton.clicked.connect(self.obrePunts)
        self.ui.mesButton.clicked.connect(self.zoomMes)
        self.ui.menysButton.clicked.connect(self.zoomMenys)
        self.ui.amuntButton.clicked.connect(self.amunt)
#        self.ui.amuntButton.setIcon(QIcon('icons8-up-24.png'))
        self.ui.avallButton.clicked.connect(self.avall)
#        self.ui.avallButton.setIcon(QIcon('icons8-down-24.png'))
        self.ui.esquerraButton.clicked.connect(self.esquerra)
#        self.ui.esquerraButton.setIcon(QIcon('icons8-left-24.png'))
        self.ui.dretaButton.clicked.connect(self.dreta)
#        self.ui.dretaButton.setIcon(QIcon('icons8-right-24.png'))
        self.ui.netejaButton.clicked.connect(self.neteja)
        self.ui.posicioCheckBox.clicked.connect(self.pintaPos)
        self.ui.iniciButton.clicked.connect(self.inici)
        self.ui.mouAutoButton.clicked.connect(self.mouAuto)
        #self.ui.fiButton.clicked.connect(self.fi)
        self.scene = MeuGraphicsScene(self)
        #self.scene = QGraphicsScene(self)
        self.mousePos = self.findChild(QLabel, 'mousePos')
        self.mapaView = self.findChild(QGraphicsView, 'mapaView')
        # self.mapaView.setMouseTracking(True)
        self.mapaView.setScene(self.scene)
        self.scene.puntSeleccionat.connect(self.on_positionChanged)
        self.pintaPosicioActual = False
        self.bola = Ball()
        self.desti = Ball()
        self.fletxa = Arrow()
        self.triangle = Triangle()
        # self.bola.setBrush(QBrush(QColor(200, 200, 200)))
        # self.scene.addItem(self.bola)
        # self.desti.setBrush(QBrush(QColor(250, 250, 0)))
        # self.scene.addItem(self.desti)
        # self.scene.addItem(self.fletxa)

    @pyqtSlot(QPointF)
    def on_positionChanged(self, pos):
        # delta = QPoint(30, -15)
        # self.label_position.show()
        # self.label_position.move(pos + delta)
        self.mousePos.setText("(%f, %f)" % (pos.x(), pos.y()))
        # self.label_position.adjustSize()
        rect = QRectF(pos.x()-5, pos.y()-5, 10, 10)
        posicio = QPointF(pos.x(),pos.y())
        # if self.pintaPosicioActual is True:
        #     #self.bola.move_to(posicio,100)
        #     self.angleactual = (self.angleactual + 45.0) % 360.0
        #     self.fletxa.move_to(posicio,self.angleactual)
        #     #print(self.angleactual)
        #     #self.triangle.move_to(posicio,100)
        for reg in self.punts:
            punt = QPointF(reg[3],reg[4])
            if rect.contains(punt):
                if self.puntinici is True:
                    self.bola.setBrush(QBrush(QColor(0, 250, 0)))
                    self.bola.move_to(punt,100)
                    self.reginici = reg
                    self.puntinici = False
                    self.puntfi = True
                    self.ui.iniciButton.setStyleSheet("background-color: red")
                    self.ui.iniciButton.setText("Fi")
                    #self.ui.fiButton.setStyleSheet("background-color: green")
                    self.desti.setBrush(QBrush(QColor(0, 250, 0)))
                elif self.puntfi is True:
                    self.desti.move_to(punt,100)
                    self.regfi = reg
                    self.puntinici = False
                    self.puntfi = False
                    self.ui.iniciButton.setStyleSheet("background-color: none")
                    self.ui.iniciButton.setText("Inici")
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
        globals.rutaactual.clear()
        self.netejaLlista.emit()
        ract = 0
        for reg in self.punts:
            if int(reg[0]) >= int(rini[0]) and int(reg[0]) <= int(rfi[0]):
                print(reg)
                ract += 1
                self.pintaPunt(reg[2],reg[1],self.drawlayer)
                self.insertaLlista.emit("%d, %0.8f, %0.8f, %s" %(ract, reg[1], reg[2], reg[3]))
                #racts = "%s" %ract
                globals.rutaactual.append((ract, reg[1], reg[2], reg[3]))

    def dispVelocitat(self,vel):
        self.ui.KmhGPS.display("%0.2f" %vel)

    def obreMapa(self):
        options = QFileDialog.Options()
        options |= QFileDialog.DontUseNativeDialog
        fileName, _ = QFileDialog.getOpenFileName(self,"Obre mapa", ".","Fitxers Tiff (*.tif);;Tots (*)", options=options)
        if fileName:
            # os.system("aplay -D hw:2,0 Sensors/so1.wav &")
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
            # self.poslayer =  QGraphicsPixmapItem(QPixmap(self.qimage.size()))
            self.neteja()
            # self.netejapos()
            self.scene.addItem(self.imalayer)
            self.scene.addItem(self.drawlayer)
            # self.scene.addItem(self.poslayer)
            self.bola.setBrush(QBrush(QColor(200, 200, 200)))
            self.scene.addItem(self.bola)
            self.desti.setBrush(QBrush(QColor(250, 250, 0)))
            self.scene.addItem(self.desti)
            # self.triangle.setBrush(QBrush(QColor(0, 255, 255)))
            # self.scene.addItem(self.triangle)
            self.scene.addItem(self.fletxa)
            self.temapa = True
            self.potenviar.emit()

    def neteja(self):
        pixmap = self.drawlayer.pixmap()
        pixmap.fill(Qt.transparent)
        self.drawlayer.setPixmap(pixmap)
        self.punts.clear()

    # def netejapos(self):
    #     pixmap = self.poslayer.pixmap()
    #     pixmap.fill(Qt.transparent)
    #     self.poslayer.setPixmap(pixmap)

    def zoomMes(self):
        scale = 1.25
        self.mapaView.scale(scale, scale)

    def zoomMenys(self):
        scale = 0.75
        self.mapaView.scale(scale, scale)

    def amunt(self):
        self.mapaView.verticalScrollBar().setValue(self.mapaView.verticalScrollBar().value() - self.interval)

    def avall(self):
        self.mapaView.verticalScrollBar().setValue(self.mapaView.verticalScrollBar().value() + self.interval)
        
    def esquerra(self):
        self.mapaView.horizontalScrollBar().setValue(self.mapaView.horizontalScrollBar().value() - self.interval)

    def dreta(self):
        self.mapaView.horizontalScrollBar().setValue(self.mapaView.horizontalScrollBar().value() + self.interval)
        # self.pintaRobot(2.14997700,41.38899300,90.0)

    def inici(self):
        self.puntinici = True
        self.puntfi = False
        self.ui.iniciButton.setStyleSheet("background-color: green")

    def mouAuto(self):
        self.mouauto.emit()
    # def fi(self):
    #     if self.puntinici is True:
    #         self.puntinici = False
    #         self.puntfi = True
    #         self.ui.fiButton.setStyleSheet("background-color: red")

    def pintaRobot(self, lon, lat, angle):
        if self.temapa is False:
            return
        if self.pintaPosicioActual is True:
            u = utm.from_latlon(lat, lon)
            row, column = self.img.index(u[0], u[1])
            # print("row %d, column %d" % (row, column))
            posicio = QPointF(column,row)
            self.fletxa.move_to(posicio,angle)

    def obrePunts(self):
        if self.temapa is False:
            msg = QMessageBox()
            msg.setIcon(QMessageBox.Information)
            msg.setText("Primer obre el mapa")
            msg.setWindowTitle("Mapa")
            msg.setStandardButtons(QMessageBox.Ok)
            retval = msg.exec_()
            return
        if globals.rutaactual: # Si ja hi ha una ruta oberta
            self.pintaRuta()
            return
        options = QFileDialog.Options()
        options |= QFileDialog.DontUseNativeDialog
        #fileName, _ = QFileDialog.getOpenFileName(self,"Obre fitxer de Punts", ".","Fitxers CSV (*.csv);;Tots (*)", options=options)
        fileName, _ = QFileDialog.getOpenFileName(self,"Obre fitxer", ".","Fitxers (*.csv *.geojson);;Tots (*)", options=options)
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
            self.neteja()
            globals.rutaactual.clear()
            self.netejaLlista.emit()
            ft = QFont("Helvetica [Cronyx]", 7)

            conta=0
            # with open(fileName, newline='') as csvfile:
            #     coordreader = csv.reader(csvfile) #, delimiter=' ', quotechar='|')
            #     self.neteja()
            #     for reg in coordreader:
            #         ord = reg[0]  # Ordre
            #         if ord.isnumeric() is False: # Saltem la primera linia
            #             continue
            #         ordi = int(ord)
            #         lon = float(reg[1])  #Longitud
            #         lat = float(reg[2])  #Latitud
            #         u = utm.from_latlon(lat, lon)
            #         row, column = self.img.index(u[0], u[1])
            #         painter.drawEllipse(column-3, row-3, 6,6)
            #         self.punts.append((ordi,lon,lat,column,row))

            #         #self.insertaLlista.emit("%s, %.8f, %.8f" %(ord,lon,lat))
            #         #w.gpsworker.coordenades.append((ord,lon,lat))
            # print( self.punts)
        #print(fileName)
        # self.coordenades.clear()

            if fileName.find("geojson") == -1:
            #     self.fitxer.emit(fileName)
                with open(fileName, newline='') as csvfile:
                    coordreader = csv.reader(csvfile) #, delimiter=' ', quotechar='|')
                    for row in coordreader:
                        a = row[0]
                        b = row[1]
                        c = row[2]
                        tipus = row[3]
                        if a.isnumeric() is False: # Saltem la primera linia
                            continue
                        ordi = int(a)
                        lon = float(b)  #Longitud
                        lat = float(c)  #Latitud
                        u = utm.from_latlon(lat, lon)
                        row, column = self.img.index(u[0], u[1])
                        painter.setPen(negre)
                        painter.drawEllipse(column-3, row-3, 6,6)
                        rec = QRect(column+5, row+5, 7,7)
                        painter.setFont(ft)
                        painter.drawText(column+5, row+5, "%d" %ordi)
                        if conta != 0:
                            painter.setPen(Qt.red)
                            painter.drawLine(column, row, columnant, rowant)
                        conta += 1
                        columnant = column
                        rowant = row
                        self.punts.append((ordi,lon,lat,tipus,column,row))
                        self.insertaLlista.emit("%d, %0.8f, %0.8f, %s" %(ordi, lon, lat, tipus))
                        globals.rutaactual.append((ordi, lon, lat, tipus))

            #             self.insertaLlista.emit("%s, %s, %s" %(a,b,c))
            #             # (int(row(0)),float(b), float(c)) = row
            #             self.coordenades.append((int(a),float(b),float(c)))
            else:
                features = geopandas.read_file(fileName)
                print(features)
                for id, punt in features.iterrows():
                    u = utm.from_latlon(punt.geometry.y, punt.geometry.x)
                    row, column = self.img.index(u[0], u[1])
                    painter.setPen(negre)
                    painter.drawEllipse(column-3, row-3, 6,6)
                    rec = QRect(column+5, row+5, 7,7)
                    painter.setFont(ft)
                    painter.drawText(column+5, row+5, "%s" %punt.ord)
                    if conta != 0:
                        painter.setPen(Qt.red)
                        painter.drawLine(column, row, columnant, rowant)
                    conta += 1
                    columnant = column
                    rowant = row
                    self.insertaLlista.emit("%s, %0.8f, %0.8f, %s" %(punt.ord, punt.geometry.x, punt.geometry.y, punt.tipus))
                    self.punts.append((punt.ord,punt.geometry.x,punt.geometry.y,punt.tipus,column,row))
                    globals.rutaactual.append((punt.ord, punt.geometry.x, punt.geometry.y, punt.tipus))

                    # self.coordenades.append((punt.ord, punt.geometry.x, punt.geometry.y))
                    # print(id, punt.ord, punt.geometry.x, punt.geometry.y)
            painter.end()
            self.drawlayer.setPixmap(pixmap)
            self.centraImatge()

    def pintaRuta(self):
        pixmap = self.drawlayer.pixmap()
        #pixmap.fill(Qt.transparent)
        painter = QPainter()
        painter.begin(pixmap)
        verd = QBrush(Qt.red)
        negre = QPen(Qt.black)
        painter.setPen(negre)
        painter.setBrush(verd)
        self.neteja()
        ft = QFont("Helvetica [Cronyx]", 7)
        conta = 0
        for row in globals.rutaactual:
            ordi = row[0]
            lon = row[1]  #Longitud 
            lat = row[2]
            tipus = row[3]
            u = utm.from_latlon(lat, lon)
            row, column = self.img.index(u[0], u[1])
            painter.setPen(negre)
            painter.drawEllipse(column-3, row-3, 6,6)
            rec = QRect(column+5, row+5, 7,7)
            painter.setFont(ft)
            painter.drawText(column+5, row+5, "%d" %ordi)
            if conta != 0:
                painter.setPen(Qt.red)
                painter.drawLine(column, row, columnant, rowant)
            conta += 1
            columnant = column
            rowant = row
            self.punts.append((ordi,lon,lat,tipus,column,row))
        painter.end()
        self.drawlayer.setPixmap(pixmap)
        self.centraImatge()
            

    def centraImatge(self):
        reg = self.punts[0]
        column = reg[3]
        row = reg[4]
        self.mapaView.fitInView(self.drawlayer, Qt.KeepAspectRatio)
        # self.mapaView.verticalScrollBar().setValue(row)
        # self.mapaView.horizontalScrollBar().setValue(column)


    def pintaPunt(self, lat, lon, layer):
        u = utm.from_latlon(lat, lon)
        row, column = self.img.index(u[0], u[1])
        print("row %d, column %d" % (row, column))
        pixmap = layer.pixmap()
        #pixmap.fill(Qt.transparent)
        painter = QPainter()
        painter.begin(pixmap)
        vermell = QBrush(Qt.red)
        negre = QPen(Qt.black)
        painter.setPen(negre)
        painter.setBrush(Qt.blue)
        painter.drawEllipse(column-3, row-3, 6, 6)
        painter.end()
        layer.setPixmap(pixmap)
        # self.modified = True

    def pintaPos(self):
        '''Check box Pinta posicio'''
        if self.ui.posicioCheckBox.isChecked():
            self.pintaPosicioActual = True
            #self.pintaPunt(w.gpsworker.puntActual[1],w.gpsworker.puntActual[0],self.poslayer)
        else:
            self.pintaPosicioActual = False     
