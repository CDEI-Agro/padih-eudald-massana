import sys
import os, struct, array
import time
import can
from typing import Any
from numpy import interp
from serial import Serial
import globals

from PyQt5.QtWidgets import (
    QApplication, QDialog, QFileDialog, QMainWindow, QMessageBox
)
from PyQt5.QtCore import QObject, QThread, pyqtSignal, pyqtSlot
from Control_motor_ui import Ui_Control

rpmMAX = 2000
girMAX = 300
distsec = 50

def convert(x):
    c = (x >> 8) & 0xff
    f = x & 0xff
    return c, f

motorslog: str
valm1:bool = False
valm2:bool = False
valma:bool = False
logmotors:bool = False
mflog: Any
ct1: int
ct2: int
mt1:int
mt2:int
mr1:int
mr2:int
ma1: int
ma2: int
vbat: int
rm1: int
rm2: int
rmdif1: int
rmdif2: int
initemps: float
distinfra1: int = 100
distso1: int = 100
distlaser: int = 100
distinfra2: int = 100
distso2: int = 100

def parse_data(msg):
    global ct1
    global ct2 
    global mt1
    global mt2
    global mr1
    global mr2
    global rm1
    global rm2
    global ma1
    global ma2
    global rmdif1
    global rmdif2
    global vbat
    global valm1
    global valm2
    global valma
    global distso1
    global distinfra1
    global distlaser
    global distso2
    global distinfra2

    # global distinfra, distso, distlaser   
    # global motorslog
    # global initemps

    if msg.arbitration_id == 0x185:
        ct1, mt1, mr1, ma1 = struct.unpack_from('<4h', msg.data)
        #print('ct= %d, mt= %d, mr=%d, ma=%d' %(ct1, mt1, mr1, ma1))
        # globals.mwin.movworker.tempC1.emit(ct1/10)
        # globals.mwin.movworker.tempM1.emit(mt1/10)
        globals.mwin.movworker.rpmM1.emit(mr1)
        globals.mwin.movworker.currentM1.emit(ma1/10)
        valm1 = True
    if msg.arbitration_id == 0x186:
        ct2, mt2, mr2, ma2 = struct.unpack_from('<4h', msg.data)
        #print('ct= %d, mt= %d, mr=%d, ma=%d' %(ct2, mt2, mr2, ma2))
        # globals.mwin.movworker.tempC2.emit(ct2/10)
        # globals.mwin.movworker.tempM2.emit(mt2/10)
        globals.mwin.movworker.rpmM2.emit(mr2)
        globals.mwin.movworker.currentM2.emit(ma2/10)
        valm2 = True
    if msg.arbitration_id == 0x385:
        vcap, vbat, status, res = struct.unpack_from('<4h', msg.data)
        globals.mwin.movworker.vbateria.emit(vbat/100)
        valma = True
    if valm1 and valm2 and valma:
        # print("%s;%d;%d;%d;%d;%d;%d;%d;%d;%d" % (time.asctime(),ct1,mt1,mr1,ma1,ct2,mt2,mr2,ma2,vbat))
        valm1 = valm2 = valma = False
        vmax = round(max(mr1, mr2)/300, 1)
        globals.mwin.movworker.kmh.emit(vmax)
        
        # rmdif1 = rm1 - mr1  # diferencia entre les revolucions demanade i les reals
        # rmdif2 = rm2 - mr2
        if logmotors is True:
            #print("%s;%d;%d;%d;%d;%d;%d;%d;%d;%d" % (time.asctime(),ct1,mt1,mr1,ma1,ct2,mt2,mr2,ma2,vbat))
            #linelog = ("%f;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d\n" % (time.time()-initemps,ct1,mt1,rm1,mr1,ma1,ct2,mt2,rm2,mr2,ma2,vbat))
            linelog = ("%f,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n" % (time.time()-initemps,ct1,mt1,rm1,mr1,ma1,ct2,mt2,rm2,mr2,ma2,vbat))
            mflog = open(motorslog,'a')
            mflog.write(linelog)
            mflog.close()

    if msg.arbitration_id == 0x120:
        distinfra1, distso1, distlaser, res = struct.unpack_from('<4h', msg.data)
        #globals.mwin.movworker.dispmsg.emit("Dinfra: %d, Dso: %d, Dlaser: %d" %(distinfra, distso, distlaser))
        globals.mwin.dispMsg("Dinfra: %d, Dso: %d, Dlaser: %d" %(distinfra1, distso1, distlaser))
        # print("Dinfra: %d, Dso: %d, Dlaser: %d\n" %(distinfra, distso, distlaser))

    if msg.arbitration_id == 0x121:
        distinfra2, distso2, distlaser, res = struct.unpack_from('<4h', msg.data)
        #globals.mwin.movworker.dispmsg.emit("Dinfra: %d, Dso: %d, Dlaser: %d" %(distinfra, distso, distlaser))
        globals.mwin.dispMsg("Dinfra: %d, Dso: %d" %(distinfra2, distso2))
        # print("Dinfra: %d, Dso: %d, Dlaser: %d\n" %(distinfra, distso, distlaser))


class Comandament(QObject):
    OrangePi = True
    conectat = False
    try:
        # os.system('sudo -S <<< macabeu systemctl stop serial-getty@ttyS3')
        # os.system('sudo -S <<< macabeu chmod 666 /dev/ttyS3')
        if OrangePi == True:
            stream1 = Serial('/dev/ttyS3', 115200, timeout=0.05)
        else:
            os.system('sudo systemctl stop serial-getty@ttyAMA2')
            os.system('sudo chmod 666 /dev/ttyAMA2')
            stream1 = Serial('/dev/ttyAMA2', 115200, timeout=0.1)

        serie=True
    except:
        print('Port serie comandament no trobat')
        serie=False

    def available(self):
        return self.serie
    
    def readcmd(self):
        comanda = self.stream1.readline()
        return comanda
    
    def envia(self,txt):
        self.stream1.write(txt)
        # print(txt)

    def clear(self):
        self.stream1.reset_input_buffer()
        self.stream1.reset_output_buffer()


class MovWorker(QObject):
    finished = pyqtSignal()
    err = pyqtSignal(str)
    joyval = pyqtSignal(str)
    currentM1 = pyqtSignal(float)
    currentM2 = pyqtSignal(float)
    rpmM1 = pyqtSignal(float)
    rpmM2 = pyqtSignal(float)
    # tempM1 = pyqtSignal(float)
    # tempM2 = pyqtSignal(float)
    # tempC1 = pyqtSignal(float)
    # tempC2 = pyqtSignal(float)
    kmh = pyqtSignal(float)
    vfix = pyqtSignal(int)
    obstacle = pyqtSignal(int)
    vbateria = pyqtSignal(float)
    botons = pyqtSignal(str)
    dispmsg = pyqtSignal(str)
    insertaLlista = pyqtSignal(str)
    neteja = pyqtSignal()

    velocitat_fixada = False
    velocitat_old = 0
    gir_old = 0
    Mando = Comandament()
    comMan = False

    # def __init__(self, win):
    #     bus = can.interface.Bus(bustype='socketcan', channel='can0', bitrate=125000)
    #     # bus = can.interface.Bus(bustype='socketcan', channel='can1', bitrate=125000)
    #     #listen = self.parse_data
    #     notifier = can.Notifier(bus, [parse_data])
    #     #self.w = win

    def run(self):
        """Bucle principal"""
        if self.Mando.available():
            self.comMan = True # ULL
            self.Mando.clear()
        else:
            print('Conecta Joystick...')
            self.err.emit("Joystick no connectat!")
            return
        self.volta = True
        velocitat = 0
        gir = 0
        self.pulsat_0 = 0
        self.err.emit("Engegat!")
        pulsatA = 0
        pulsatB = 0
        pulsatC = 0

        darrertemps = time.time()
        global distinfra1
        global distso1
        global distlaser
        global distinfra2
        global distso2

        while self.volta:
            if self.comMan: # comandament radio control
                comanda_b = self.Mando.readcmd()
                comanda = str(comanda_b.decode())
                comanda = comanda.rstrip()
                if (distso1 < distsec and distinfra1 < distsec) or (distso2 < distsec and distinfra2 < distsec):
                    self.mouMotorsCAN(0.0, 0.0, 0)
                    self.velocitat_fixada = False
                    self.vfix.emit(0)
                    self.obstacle.emit(1)
                    self.joyval.emit('Obstacle detectat')
                if comanda == '':
                    if time.time() - darrertemps > 0.5:
                    # print('Comanda buida')
                        self.velocitat_fixada = False
                        self.vfix.emit(0)
                        if globals.mwin.engega is True:
                            self.mouMotorsCAN(0.0, 0.0, 0) #Cal revisar
                        self.joyval.emit('Comanda no rebuda')
                        # darrertemps = time.time()
                        continue
                # print(comanda)
                if (comanda.startswith('$CMD')):
                    darrertemps = time.time()
                    cmd_array = [element.strip() for element in comanda.split(',')]
                    i_vel = int(cmd_array[1])
                    if i_vel > 0:
                        n_vel = interp(i_vel,[0,1947],[0,1])
                    else:
                        n_vel = interp(i_vel,[-1948,0],[-1,0])
                    i_gir = int(cmd_array[2])
                    if i_gir > 0:
                        n_gir = interp(i_gir,[0,1947],[0,1])
                    else:
                        n_gir = interp(i_gir,[-1948,0],[-1,0])
                    rot_gir = int(cmd_array[3])
                    if globals.mouAuto is False:    # Si no està en mode automàtic gira el robot
                        if rot_gir != 0:
                            if rot_gir > 0:
                                r_gir = interp(rot_gir,[0,1000],[0,1])                        
                                self.mouMotorsCAN(0.0, r_gir, 1)
                                # self.giraRobot(r_gir)
                            else:
                                r_gir = interp(rot_gir,[-1000,0],[-1,0])
                                self.mouMotorsCAN(0.0, r_gir, 1)
                                #self.giraRobot(r_gir)
                    puls_0 = int(cmd_array[4])
                    if puls_0 == 1:
                        self.pulsat_0 = 1
                    else:
                        if self.pulsat_0 == 1:
                            self.pulsat_0 = 0
                            if self.velocitat_fixada == True:
                                self.velocitat_fixada = False
                                self.vfix.emit(0)
                            else:
                                self.velocitat_fixada = True
                                globals.velofix = n_vel
                                self.vfix.emit(1)
                    if self.velocitat_fixada is False:
                        velocitat = n_vel
                    boto_A = int(cmd_array[5])
                    if boto_A == 1 and pulsatA == 0:
                        pulsatA = 1
                        self.botons.emit("A")
                    else:
                        if pulsatA == 1 and boto_A == 0:
                            pulsatA = 0

                    boto_B = int(cmd_array[6])
                    if boto_B == 1 and pulsatB == 0:
                        pulsatB = 1
                        self.botons.emit("B")
                    else:
                        if pulsatB == 1 and boto_B == 0:
                            pulsatB = 0

                    boto_C = int(cmd_array[7])  # equival al boto6 del comandament
                    if boto_C == 1 and pulsatC == 0:
                        pulsatC = 1
                        self.botons.emit("C")
                    else:
                        if pulsatC == 1 and boto_C == 0:
                            pulsatC = 0

                    # print("Vel {:.3f}, Gir{:.3f}".format(velocitat,n_gir))
                    # self.joyval.emit("V: %.3f G: %.3f" % (velocitat, n_gir))
                    if self.velocitat_fixada is True:
                        globals.deltavelo = globals.velofix
                    else:
                        globals.deltavelo = velocitat
                    if globals.testGir is False:
                        if globals.mouAuto is True and globals.pausaAuto is False:
                            if globals.movent is True:
                                self.mouMotorsCAN(globals.deltavelo, globals.deltagir, 0) #El gir el controla l'automatic
                        else:
                            if rot_gir == 0: # Si no està girant el robot
                                self.mouMotorsCAN(velocitat, n_gir, 0)
                if (comanda.startswith('$RU')):
                    print('{'+comanda+'}')
                    cmd_array = [element.strip() for element in comanda.split(',')]
                    if cmd_array[1] == 'INI':
                        self.neteja.emit()
                    elif cmd_array[1] == 'FI':
                        print(len(globals.rutaactual))
                    else:
                        reg = comanda.strip('$RU,')
                        # print(reg)
                        self.insertaLlista.emit(reg)
                        globals.rutaactual.append((int(cmd_array[1]),float(cmd_array[2]),float(cmd_array[3]),cmd_array[4]))


        print("Acaba bucle")
        self.finished.emit()

    def mouMotorsCAN(self, velocitat, gir, rot_gir = 0):
        if rot_gir == 0:
            if globals.reverse == True:
                velocitat = -velocitat  # en cas d'anar invertit
                gir = -gir
            if velocitat >=0 and gir >=0:
                velocitat1 = velocitat * (1 - gir)
                rpm1 = int(interp(velocitat1,[0,1],[0,rpmMAX]))
                rpm2 = int(interp(velocitat,[0,1],[0,rpmMAX]))
                self.motor1(-rpm2)
                self.motor2(-rpm1)

            if velocitat >=0 and gir <0:
                velocitat2 = velocitat * (1 - abs(gir))
                rpm2 = int(interp(velocitat2,[0,1],[0,rpmMAX]))
                rpm1 = int(interp(velocitat,[0,1],[0,rpmMAX]))
                self.motor2(-rpm1)
                self.motor1(-rpm2)

            if velocitat < 0 and gir >=0:
                velocitat1 = abs(velocitat) * (1 - gir)
                rpm1 = int(interp(velocitat1,[0,1],[0,rpmMAX]))
                rpm2 = int(interp(abs(velocitat),[0,1],[0,rpmMAX]))
                # difmot1 = rpm2 - vm1
                self.motor1(rpm2)
                self.motor2(rpm1)

            if velocitat < 0 and gir <0:    # endevant i cap a la dreta
                velocitat2 = abs(velocitat) * (1 - abs(gir))
                rpm2 = int(interp(velocitat2,[0,1],[0,rpmMAX]))
                rpm1 = int(interp(abs(velocitat),[0,1],[0,rpmMAX]))
                self.motor2(rpm1)
                self.motor1(rpm2)
        else:   # rot_gir és True i el gir és el valor de la rotació
            if gir > -0.001 and gir < 0.001:
                gir = 0.0
            rpms = int(interp(abs(gir),[0,1],[0,girMAX]))
            #print(rpms)
            if gir >= 0:
                self.motor1(rpms)
                self.motor2(-rpms)
            else:
                self.motor1(-rpms)
                self.motor2(rpms)

        self.joyval.emit("V: %.3f G: %.3f R: %d" % (velocitat, gir, rot_gir))
        # print('mouMotors V: %+.3f, G: %+.3f, rpm1 %d, rpm2 %d' % (velocitat, gir, rpm1, rpm2))
        # self.gir_old = gir
        # self.velocitat_old = velocitat

    # def giraRobot(self, gir):
    #     # if globals.deltavelo <= 0:
    #     #     gir = 0
    #     if gir > -0.001 and gir < 0.001:
    #         gir = 0.0
    #     rpms = int(interp(abs(gir),[0,1],[0,girMAX]))
    #     #print(rpms)
    #     if gir >= 0:
    #         self.motor1(rpms)
    #         self.motor2(-rpms)
    #     else:
    #         self.motor1(-rpms)
    #         self.motor2(rpms)

    def motor1(self, rpms):
        #print("Motor1: %d" % (rpms))
        #print(rpms)
        global rm1
        rm1 = rpms
        c, f = convert(rpms)
        msg = can.Message(arbitration_id=0x205, channel = 0,
            data=[f, c, globals.interlock, 0, 0, 0, 0, 0],
            is_extended_id=False)
        try:
            # self.bus.send(msg)
            ret = globals.mwin.bus.state
            if globals.mwin.bus.state == can.BusState.ACTIVE:
                globals.mwin.bus.send(msg)
            # print(f"Message sent on {self.bus.channel_info}")
        except can.CanError as e:
            print("Failed to send CAN message: %s" % str(e))
    
    def motor2(self, rpms):
        #print("Motor2: %d" % (rpms))
        global rm2
        rm2 = rpms
        c, f = convert(rpms)
        msg = can.Message(arbitration_id=0x206, channel = 0,
            data=[f, c, globals.interlock, 0, 0, 0, 0, 0],
            is_extended_id=False)
        try:
            # self.bus.send(msg)
            ret = globals.mwin.bus.state
            if ret == can.BusState.ACTIVE:
                globals.mwin.bus.send(msg)
            # print(f"Message sent on {self.bus.channel_info}")
        except can.CanError as e:
            print("Failed to send CAN message: %s" % str(e))
        

    def atura(self):
        print("Surt del bucle Mov")
#        self.mouMotorsCAN(0,0)
        self.volta = False
        # globals.mwin.notifier.stop()
        # globals.mwin.notifier.remove_listener(parse_data)
        # globals.mwin.bus.shutdown()
