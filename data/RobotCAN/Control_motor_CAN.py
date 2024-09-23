import sys
import time
import can
from typing import Any
from numpy import interp
from serial import Serial
import globals
import json
import smbus2 as smb
# from PCF8574 import PCF8574

from PyQt5.QtWidgets import (
    QApplication, QDialog, QFileDialog, QMainWindow, QMessageBox,
      QPushButton, QGraphicsScene, QGraphicsPixmapItem, QGraphicsView, QLabel,
      QCheckBox, QVBoxLayout, QLineEdit
)

from PyQt5.QtCore import Qt, QObject, QThread, pyqtSignal, pyqtSlot, QPoint, QEvent, QPointF, QRect, QRectF
from PyQt5.QtGui import QPixmap, QImage, QPainter, QPen, QBrush, QPainterPath
from Control_motor_ui import Ui_Control

# from Mapa_ui import Ui_Mapa

# from PyQt5.uic import loadUi
# from gpiozero import Motor, Robot, DistanceSensor
#import Gamepad
import GPSWorker
import MovWorker
import MapaView

from constants_ui import Ui_ConstantsDialog



class ConstantsDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.ui = Ui_ConstantsDialog()
        self.ui.setupUi(self)
        self.show()
        self.ui.buttonBox.accepted.connect(self.accepted)
        self.ui.buttonBox.rejected.connect(self.rejected)
        self.ui.Kd_aEdit.setText(str(GPSWorker.Kda))
        self.ui.Kp_aEdit.setText(str(GPSWorker.Kpa))
        self.ui.KdEdit.setText(str(GPSWorker.Kd))
        self.ui.KpEdit.setText(str(GPSWorker.Kp))
        self.ui.KiEdit.setText(str(GPSWorker.Ki))
        self.ui.Ki_aEdit.setText(str(GPSWorker.Kia))
        self.ui.distinterpEdit.setText(str(GPSWorker.interdist))
        self.ui.distcapturaEdit.setText(str(GPSWorker.distcaptura))
        self.ui.distmindestiEdit.setText(str(GPSWorker.distmindesti))
        self.ui.maxrpmEdit.setText(str(MovWorker.rpmMAX))
        self.ui.maxrpmgirEdit.setText(str(MovWorker.girMAX))
        self.ui.anglemingirEdit.setText(str(GPSWorker.minAngle))

    def accepted(self):
        GPSWorker.Kda = float(self.ui.Kd_aEdit.text())
        GPSWorker.Kpa = float(self.ui.Kp_aEdit.text())
        GPSWorker.Kd = float(self.ui.KdEdit.text())
        GPSWorker.Kp = float(self.ui.KpEdit.text())
        GPSWorker.Ki = float(self.ui.KiEdit.text())
        GPSWorker.Kia = float(self.ui.Ki_aEdit.text())
        GPSWorker.interdist = float(self.ui.distinterpEdit.text())
        GPSWorker.distcaptura = float(self.ui.distcapturaEdit.text())
        GPSWorker.distmindesti = float(self.ui.distmindestiEdit.text())
        MovWorker.rpmMAX = float(self.ui.maxrpmEdit.text())
        MovWorker.girMAX = float(self.ui.maxrpmgirEdit.text())
        GPSWorker.minAngle = float(self.ui.anglemingirEdit.text())
        w.write_constants()
        self.close()
    

class AppWindow(QDialog):
    engegat = False
    i2cbus = False
    estat: bytes = b'\x00'
    estat_on = True
    StopObstacle = 0

    def __init__(self):
        super().__init__()
        self.ui = Ui_Control()
        self.ui.setupUi(self)
        self.show()  
        self.ui.Boto_Engega.clicked.connect(self.engega)
        self.ui.Boto_Para.clicked.connect(self.para)
        self.ui.Gpsfix.setStyleSheet("background-color: red")
        self.ui.Boto_captura_log.clicked.connect(self.capturalog)
        self.ui.Boto_captura_log.setStyleSheet("background-color: green")
        self.ui.Boto_mes_coord.clicked.connect(self.capturaPunt)
        self.ui.Boto_guarda_coor.clicked.connect(self.grava)
        self.ui.Boto_mou_auto.clicked.connect(self.mou_auto)
        self.ui.Boto_pausa.clicked.connect(self.pausa_auto)
        self.ui.Boto_pausa.setHidden(True)
        self.ui.Boto_menys_coord.clicked.connect(self.treuPunt)
        self.ui.Boto_obre_coord.clicked.connect(self.obreCoord)
        self.ui.Boto_interlock.clicked.connect(self.interlock)
        self.ui.Boto_mapa.clicked.connect(self.Mapa)
        self.ui.Boto_esborra.clicked.connect(self.netejaLlista)
        self.ui.Boto_test_gir.clicked.connect(self.testGir)
        self.ui.Boto_obstaclebt.clicked.connect(self.obstaclebt)
        self.ui.Boto_constants.clicked.connect(self.open_constants_dialog)
        self.ui.gir_checkBox.stateChanged.connect(self.gir_checked)
        # self.ui.gir_checkBox.setChecked()
        self.mapa_dialog = None
        self.enviamapa = False

        # self.engega()

    def engega(self):
        '''Polsat botò Engega i crea els threads'''
        # Step 2: Create a QThread object
        self.movthread = QThread()
        self.gpsthread = QThread()
        # Step 3: Create a worker object
        self.movworker = MovWorker.MovWorker()
        self.gpsworker = GPSWorker.GpsWorker()
        # Step 4: Move worker to the thread
        self.movworker.moveToThread(self.movthread)
        self.gpsworker.moveToThread(self.gpsthread)
        # Step 5: Connect signals and slots
        self.movthread.started.connect(self.movworker.run)
        self.gpsthread.started.connect(self.gpsworker.run)
    #    self.ui.Boto_Para.clicked.connect(self.movworker.atura)
        self.movworker.finished.connect(self.movthread.quit)
        self.movworker.finished.connect(self.movworker.deleteLater)
        self.movthread.finished.connect(self.movthread.deleteLater)
        self.movworker.err.connect(self.dispErr) 
        self.movworker.joyval.connect(self.dispJoyval)
        # self.movworker.tempC1.connect(self.dispTempC1)
        # self.movworker.tempC2.connect(self.dispTempC2)
        # self.movworker.tempM1.connect(self.dispTempM1)
        # self.movworker.tempM2.connect(self.dispTempM2)
        self.movworker.rpmM1.connect(self.dispRpmM1)
        self.movworker.rpmM2.connect(self.dispRpmM2)
        self.movworker.currentM1.connect(self.dispCurrM1)
        self.movworker.currentM2.connect(self.dispCurrM2)
        self.movworker.vbateria.connect(self.dispVbat)
        self.movworker.kmh.connect(self.dispVel)
        self.movworker.vfix.connect(self.vfix)
        self.movworker.obstacle.connect(self.obstacle)
        self.movworker.botons.connect(self.botons)
        self.movworker.dispmsg.connect(self.dispMsg)
        self.movworker.insertaLlista.connect(self.insertaLlista)
        self.movworker.neteja.connect(self.netejaLlista)
        self.gpsworker.finished.connect(self.gpsthread.quit)
        self.gpsworker.finished.connect(self.gpsworker.deleteLater)
        self.gpsthread.finished.connect(self.gpsthread.deleteLater)
        self.gpsworker.err.connect(self.dispErr)
        #self.gpsworker.anglesMsg.connect(self.dispAngles)
        self.gpsworker.angleOk.connect(self.angleOk)
        self.gpsworker.coord.connect(self.dispCoord)
        self.gpsworker.kmhGPS.connect(self.dispVelGPS)
        self.gpsworker.enviacoor.connect(self.enviacoor)
        self.gpsworker.insertaLlista.connect(self.insertaLlista)
        self.gpsworker.neteja.connect(self.netejaLlista)
        self.gpsworker.somAdesti.connect(self.somAdesti)
        self.gpsworker.mouRobot.connect(self.MouRobot)
#        self.gpsworker.giraRobot.connect(self.GiraRobot)
        self.gpsworker.intermitent.connect(self.int_estat)
        try:   
#            self.bus = can.interface.Bus(bustype='socketcan', channel='can0', bitrate=125000)
            self.bus = can.interface.Bus(interface = 'canalystii', channel = (0, 1), bitrate = 125000)
            self.notifier = can.Notifier(self.bus, [MovWorker.parse_data])
        except can.CanError as error:
            print("Failed start: %s" % str(error))
        try:            
            self.stbus = smb.SMBus(5)
            self.estat = 0x00
            self.stbus.write_byte_data(0x38, 0x00, 0x00)
            self.i2cbus = True
        except:
            print("Failed start: SMBus")
            self.i2cbus = False

        # Step 6: Start the thread
        self.movthread.start()
        self.gpsthread.start()
        self.ui.Boto_Engega.setText("ENGEGAT")
        self.read_constants()
        self.set_estat(3, True) #tronja on
        self.engegat = True

    def dispErr(self, txt):
        self.ui.Error.setText(txt)

    def dispMsg(self, txt):
        self.ui.dispMsg.setText(txt)

    def dispJoyval(self, txt):
        self.ui.Joyval.setText(txt)

    # def dispTempC1(self, txt):
    #     self.ui.TempC1.display(txt)

    # def dispTempC2(self, txt):
    #     self.ui.TempC2.display(txt)

    # def dispTempM1(self, txt):
    #     self.ui.TempM1.display(txt)

    # def dispTempM2(self, txt):
    #     self.ui.TempM2.display(txt)

    def dispRpmM1(self, txt):
        self.ui.RpmM1.display(txt)

    def dispRpmM2(self, txt):
        self.ui.RpmM2.display(txt)

    def dispCurrM1(self, txt):
        self.ui.CurrentM1.display(txt)

    def dispCurrM2(self, txt):
        self.ui.CurrentM2.display(txt)

    def dispVbat(self, val):
        if val < 42:
            self.ui.Vbateria.setStyleSheet("background-color: red")
        txt = str("{:0.1f}".format(val))
        self.ui.Vbateria.display(txt)

    def dispVel(self, txt):
        self.ui.Kmh.display(txt)

    def dispVelGPS(self, val):
        if self.enviamapa is True:
            self.mapa_dialog.dispVelocitat(val)
        txt = str("{:0.2f}".format(val))
        self.ui.KmhGPS.display(txt)

    def angleOk(self,val):
        if val == 1:
            self.ui.angleok.setStyleSheet("background-color: green")
            self.set_estat(7, True)
        else:
            self.ui.angleok.setStyleSheet("background-color: red")
            self.set_estat(7, False)

    def vfix(self, val):
        if val == 1:
            self.ui.Kmh.setStyleSheet("background-color: orange")
        else:
            self.ui.Kmh.setStyleSheet("background-color: white")

    def dispCoord(self, txt):
        self.ui.Coord.setText(txt)


    def botons(self,tx):
        if tx == "A":
            self.capturaPunt()
        if tx == "B":
            self.mou_auto()
        if tx == "C":
            self.interlock()

    def Mapa(self):
        '''Polsat botò Mapa i obre el dialeg de mapa'''
        self.mapa_dialog = MapaView.Mapa()
        self.mapa_dialog.show()
        self.mapa_dialog.insertaLlista.connect(self.insertaLlista)
        self.mapa_dialog.netejaLlista.connect(self.netejaLlista)
        self.mapa_dialog.potenviar.connect(self.potenviar)
        self.mapa_dialog.mouauto.connect(self.mou_auto)
        self.mapa_dialog.accepted.connect(self.mapa_accept)

    def mapa_accept(self):
        self.enviamapa = False
        # self.mapa_dialog.atura()
        self.mapa_dialog.deleteLater()
        self.mapa_dialog = None

    def open_constants_dialog(self):
        dialog = ConstantsDialog(self)
        # dialog.exec()

    def potenviar(self):
        self.enviamapa = True

    def obstaclebt(self):
        if self.StopObstacle == 1:
            self.StopObstacle = 0
            self.ui.obstaclebt.setStyleSheet("background-color: white")
            self.set_estat(2, False) #vermell off

    def obstacle(self, val):
        if val == 1:
            self.StopObstacle = 1
            self.ui.obstaclebt.setStyleSheet("background-color: red")
            self.set_estat(2, True) #vermell on
        else:
            self.StopObstacle = 0
            self.ui.obstaclebt.setStyleSheet("background-color: white")
            self.set_estat(2, False) #vermell off

    def enviacoor(self, val):
        valors = val.split(',')
        q = int(valors[3])
        if q == 4:
            self.ui.Gpsfix.setStyleSheet("background-color: green")
        elif q == 5:
            self.ui.Gpsfix.setStyleSheet("background-color: orange")
        elif q == 0:
            self.ui.Gpsfix.setStyleSheet("background-color: red")
        else:
            self.ui.Gpsfix.setStyleSheet("background-color: yellow")

        if self.enviamapa is True:
            lon = float(valors[0])
            lat = float(valors[1])
            ang = float(valors[2])
            self.mapa_dialog.pintaRobot(lon,lat,ang)
#        val = '$CO,' + val + '\n'
        val = '$CO,' + val
        self.movworker.Mando.envia(bytes(val, encoding='utf-8'))
        status = '$ST,'
        if globals.mouAuto is True:
            status += 'A,'
        else:
            status += 'a,'
        if globals.interlock == 1:
            status += 'i,'
        else:
            status += 'I,'
        if self.StopObstacle == 1:
            status += 'O\n'
        else:
            status += 'o\n'
    
        self.movworker.Mando.envia(bytes(status, encoding='utf-8'))
        # print(status)

    def para(self):
        self.set_estat(3, False) #tronja off
        self.stbus.write_byte(0x38, 0x00) #apaga tot
        self.notifier.stop()
        self.engegat = False
        time.sleep(0.2)
        self.movworker.atura()
        time.sleep(0.2)
        self.gpsworker.atura()
        self.ui.Boto_Engega.setText("Engega")
        self.ui.Error.setText("Parat")
        self.bus.shutdown()


    def capturalog(self):   #Boto
        '''Grava log de parametres de motors.'''
        #global motorslog
        #global mflog
#
        if MovWorker.logmotors is True:
            MovWorker.logmotors = False
 #           mflog.close()
            self.ui.Boto_captura_log.setText("Captura")
            self.ui.Boto_captura_log.setStyleSheet("background-color: green")
            self.ui.Fitxer.setText("")
            print("Tanquem motors log")
        else:
            self.ui.Boto_captura_log.setText("CAPTURANT")
            self.ui.Boto_captura_log.setStyleSheet("background-color: red")
            MovWorker.initemps = time.time()
            print("Gravem motors log")
            MovWorker.motorslog = time.strftime("logs/Motors_%d-%m_%H-%M.csv")
            self.ui.Fitxer.setText(MovWorker.motorslog)
            MovWorker.mflog = open(MovWorker.motorslog, "w")
            iniline=("Time,Ct1,Mt1,Rm1,Mr1,Ma1,Ct2,Mt2,Rm2,Mr2,Ma2,Vbat\n")
            MovWorker.mflog.write(iniline)
            MovWorker.mflog.close()
            MovWorker.logmotors = True

    def insertaLlista(self, item): # ve de GPSWorker, Movworker i Mapa
        # print(item)
        self.ui.listWidget.setAutoScroll(True)
        self.ui.listWidget.addItem(item)
        self.ui.listWidget.scrollToBottom()

    def netejaLlista(self): # ve de GPSWorker, Movworker i Mapa
        self.ui.listWidget.clear()
        globals.rutaactual.clear()
        self.gpsworker.numord = 0

    def capturaPunt(self): # Boto
        '''Polsat botò Captura Punt + '''
        self.gpsworker.capturaPunt()

    def gir_checked(self):
        if self.ui.gir_checkBox.isChecked():
            self.gpsworker.gir_pas = True
        else:
            self.gpsworker.gir_pas = False

    def treuPunt(self):
        '''Pulsat boto treurepunt elimina l'ultim punt entrar a la llista '''
        self.gpsworker.treuPunt()

    def grava(self):
        '''Polsat botò Grava '''
        if self.engegat:
            self.gpsworker.grava()

    def obreCoord(self):
        '''Polsat botò Obre coordenades'''
        if self.engegat:
            self.ui.listWidget.clear()
            options = QFileDialog.Options()
            options |= QFileDialog.DontUseNativeDialog
            fileName, _ = QFileDialog.getOpenFileName(self,"Obre fitxer", ".","Fitxers (*.csv *.geojson);;Tots (*)", options=options)
            if fileName:
                print(fileName)
                self.gpsworker.obreFitxer(fileName)

    def MouRobot(self,cmd): # ve de GPSWorker
        v, g, r = cmd.split(',')
        velo = float(v)
        gir = float(g)
        rot = int(r)
        self.movworker.mouMotorsCAN(velo,gir,rot)
        #self.ui.velocitatMsg.setText(str(velo))
        #self.ui.girMsg.setText(str(gir))

    # def GiraRobot(self,cmd): # ve de GPSWorker
    #     gir = float(cmd)
    #     self.movworker.giraRobot(gir)
    #     #self.ui.velocitatMsg.setText(str(velo))
    #     #self.ui.girMsg.setText(str(gir))

    def testGir(self):
        if self.engegat:
            if globals.testGir is False:
                self.gpsworker.giraAuto()
                self.ui.Boto_test_gir.setStyleSheet("background-color: red")
                globals.testGir = True
            else:
                self.ui.Boto_test_gir.setStyleSheet("background-color: white")
                globals.testGir = False

    def mou_auto(self):
        '''Polsat botò Mou Auto'''
        if self.engegat and len(globals.rutaactual) > 0:
            if globals.mouAuto is False:
                self.gpsworker.mouAutomatic()
                self.ui.Boto_mou_auto.setStyleSheet("background-color: red")
                self.ui.Boto_pausa.setHidden(False)
                self.ui.Boto_pausa.setText("Pausa")
                self.set_estat(6,True) #blau on

                # self.ui.Boto_Mou.setText('Movent')
                # self.ui.editKd.setText(str(self.gpsworker.Kd))
                # self.ui.editKp.setText(str(self.gpsworker.Kp))
                # self.ui.Boto_ParaCont.setText("Atura")
            else:
                #self.gpsworker.mou()
                #self.ui.Boto_Mou.setText('Mou A')
                self.ui.Boto_mou_auto.setStyleSheet("background-color: white")
                self.ui.Boto_pausa.setHidden(True)
                globals.mouAuto = False
                self.set_estat(6,False) #blau off

    def pausa_auto(self):
        '''Polsat botò Pausa'''
        if self.engegat:
            if globals.pausaAuto is False:
                #self.gpsworker.pausaAuto()
                self.ui.Boto_pausa.setStyleSheet("background-color: red")
                self.ui.Boto_pausa.setText("Continua")
                globals.pausaAuto = True
            else:
                #self.gpsworker.continuaAuto()
                self.ui.Boto_pausa.setStyleSheet("background-color: white")
                self.ui.Boto_pausa.setText("Pausa")
                globals.pausaAuto = False

    def somAdesti(self): #ve de GPSWorker
        # self.ui.Boto_mou_auto.setText("Mou A")
        self.ui.Boto_mou_auto.setStyleSheet("background-color: white")
        self.ui.Boto_test_gir.setStyleSheet("background-color: white")
        self.movworker.mouMotorsCAN(0,0,0)
        self.movworker.velocitat_fixada = False
        self.vfix(0)
        #self.pcf.digitalWrite('P6',0)
        self.set_estat(6,False) #blau off
        
    def interlock(self):
        if globals.interlock == 1:
            globals.interlock = 0
            self.ui.Boto_interlock.setStyleSheet("background-color: red")
            self.movworker.mouMotorsCAN(0,0,0)
            self.movworker.velocitat_fixada = False
            self.vfix(0)
            self.set_estat(2, True) #vermell on
        else:
            globals.interlock = 1
            self.ui.Boto_interlock.setStyleSheet("background-color: white")
            self.set_estat(2, False) #vermell off

    def write_constants(self):
        constants = {
            'Kpa': GPSWorker.Kpa,
            'Kda': GPSWorker.Kda,
            'Kia': GPSWorker.Kia,
            'Kp': GPSWorker.Kp,
            'Kd': GPSWorker.Kd,
            'Ki': GPSWorker.Ki,
            'distinterp': GPSWorker.interdist,
            'distcaptura': GPSWorker.distcaptura,
            'distmindesti': GPSWorker.distmindesti,
            'maxrpm': MovWorker.rpmMAX,
            'maxrpmgir': MovWorker.girMAX,
            'angleminim': GPSWorker.minAngle
            }
        with open('constants.json', 'w') as f:
            json.dump(constants, f, indent=4)

    def read_constants(self):
        with open('constants.json', 'r') as f:
            constants = json.load(f)
            GPSWorker.Kda = constants['Kda']
            GPSWorker.Kpa = constants['Kpa']
            GPSWorker.Kp = constants['Kp']
            GPSWorker.Kd = constants['Kd']
            GPSWorker.Ki = constants['Ki']
            GPSWorker.Kia = constants['Kia']
            GPSWorker.interdist = constants['distinterp']
            GPSWorker.distcaptura = constants['distcaptura']
            GPSWorker.distmindesti = constants['distmindesti']
            MovWorker.rpmMAX = constants['maxrpm']
            MovWorker.girMAX = constants['maxrpmgir']
            GPSWorker.minAngle = constants['angleminim']


    def int_estat(self): # ve de GPSWorker fa intermitent
        if self.i2cbus is False:
            return
        try:
            if self.estat_on is True:
                self.estat_on = False
                #print("estat off")
                self.stbus.write_byte(0x38, 0x00)
            else:
                self.estat_on = True
                #print("estat on")
                self.stbus.write_byte(0x38, self.estat)
        except:
            print("Error a int_estat")
        
    def set_estat(self, idx, val):
        '''
        bits i colors
        2: vermell
        3: groc
        7: verd
        6: blau
        5: blanc
        4: sona xiulet
        '''
        try:
            if self.i2cbus is False:
                return
            self.estat = self.set_bit(self.estat, idx, val)
            self.stbus.write_byte(0x38, self.estat)
#            print("Estat: {0:08b}".format(self.estat) )
            # print("0b{:b}".format(number))
        except:
            print("Error a set_estat")

    def set_bit(self,v, index, x):
        """Set the index:th bit of v to 1 if x is truthy, else to 0, and return the new value."""
        mask = 1 << index   # Compute mask, an integer with just bit 'index' set.
        v &= ~mask          # Clear the bit indicated by the mask (if x is False)
        if x:
            v |= mask         # If x was True, set the bit indicated by the mask.
        return v            # Return the result, we're done.

app = QApplication(sys.argv)
w = AppWindow()
globals.iniglob()
globals.mwin = w
w.show()
sys.exit(app.exec_())
