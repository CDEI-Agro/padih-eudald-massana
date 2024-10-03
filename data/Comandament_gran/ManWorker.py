from PyQt5.QtCore import QObject, QThread, pyqtSignal, pyqtSlot
import time
import os
import board
import busio
import json
#import adafruit_ads1x15.ads1015 as ADS
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
from serial import Serial
from numpy import interp
from gpiozero import Button
import RPi.GPIO as GPIO
from encoder import Encoder

GPIO.setmode(GPIO.BCM)
rotary_value = 0

class Connexio(QObject):
    conectat = False
    # try:
    #     os.system('sudo systemctl stop serial-getty@ttyAMA2')
    #     os.system('sudo chmod 666 /dev/ttyAMA2')
    #     stream1 = Serial('/dev/ttyAMA2', 115200, timeout=0.1)
    #     serie=True
    # except:
    #     print('Port serie comandament no trobat')
    #     serie=False
    try:
        os.system('sudo systemctl stop serial-getty@ttyS0')
        os.system('sudo chmod 666 /dev/ttyS0')
        stream1 = Serial('/dev/ttyS0', 115200, timeout=0.1)
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

    def envialn(self,txt):
        self.stream1.writelines(txt)

    def clear(self):
        self.stream1.reset_input_buffer()
        self.stream1.reset_output_buffer()


class ManWorker(QObject):
    finished = pyqtSignal()
    err = pyqtSignal(str)
    manval = pyqtSignal(str)
    vbateria = pyqtSignal(str)
    rebut = pyqtSignal(str)
    joypos = pyqtSignal(str)
    
    connexio = Connexio()


    # Create the I2C bus
    i2c = busio.I2C(board.SCL, board.SDA)

    # Create the ADC object using the I2C bus
    ads = ADS.ADS1115(i2c)

    def valueChanged(value, direction):
        global rotary_value
        if value >= 0:
            print("New value: {}, Direction: {}".format(value, direction))
            rotary_value = value
    
            

    e1 = Encoder(27, 22, valueChanged)


    # Create single-ended input on channel 0
    chan0 = AnalogIn(ads, ADS.P0)
    chan1 = AnalogIn(ads, ADS.P1)
    chan2 = AnalogIn(ads, ADS.P2)
    chan3 = AnalogIn(ads, ADS.P3)
    boto1 = Button(21,bounce_time=0.1)
    boto2 = Button(20,bounce_time=0.1,hold_time=0.3)
    boto3 = Button(5,bounce_time=0.2)
    boto4 = Button(6,bounce_time=0.2)
    boto5 = Button(13,bounce_time=0.2)
    boto6 = Button(19,bounce_time=0.2)
    boto7 = Button(26,bounce_time=0.2) # boto joystick
    boto8 = Button(17,bounce_time=0.2) # boto encoder
            

    def run(self):
        """Bucle principal"""
        self.volta = True
        val0vell = 0.1
        val1vell = 0.1
        vbatvell = '0.0'
        val2 = 0
        global rotary_value
        but1, but2, but3, but4, but5, but6, but7, but8 = 0, 0, 0, 0, 0, 0, 0, 0
        try:
            while self.volta is True:
                if self.chan0.value > 14000:    #Gir
                    val0 = interp(self.chan0.value,[14000,26460],[0,1947])
                    #val0 -= 100
                elif self.chan0.value < 12300:
                    val0 = interp(self.chan0.value,[0,12300],[-1948,0])
                    #val0 += 100
                else:
                    val0 = 0

                if rotary_value <= 0:
                    if self.chan1.value > 14500:    #Velocitat
                        val1 = interp(self.chan1.value,[14500,26460],[0,1947])
                        val1 = -val1
                    elif self.chan1.value < 12300:
                        val1 = interp(self.chan1.value,[0,12300],[-1948,0])
                        val1 = -val1
                    else:
                        val1 = 0
                else:
                    val1 = rotary_value * 100
                    if val1 > 1947: val1 = 1947
                    val1 = -val1
                if val1 == 0:   #velocitat = 0
                    valchan2 = self.chan2.value
                    if valchan2 > 14000:    # gir rotacio a la dreta. Max 26000
                        val2 = interp(self.chan2.value, [14000,26000],[0,1000])
                        #print(valchan2)
                    elif valchan2 < 11500:
                        #print(valchan2)
                        val2 = interp(valchan2, [0,11500],[1000,0])
                        val2 = -val2
                    else:
                        val2 = 0
                else:
                    val2 = 0
                # print(comanda)
                vbat = '{:.2f}'.format(self.chan3.voltage * 3.05)
                if vbat != vbatvell:
                    vbatvell = vbat
                    self.vbateria.emit(vbat)

                if self.boto1.is_pressed and but1 == 0:
                    but1 = 1
                    print('Premut 1')
                    print(rotary_value)
                    rotary_value = False
                else:
                    if but1 == 1 and self.boto1.is_pressed == False:
                        but1 = 0
                if self.boto2.is_pressed and but2 == 0:
                    but2 = 1
                    print('Premut 2')
                else:
                    if but2 == 1 and self.boto2.is_pressed == False:
                        but2 = 0

                if self.boto3.is_pressed and but3 == 0:
                    but3 = 1
                    print('Premut 3')
                else:
                    if but3 == 1 and self.boto3.is_pressed == False:
                        but3 = 0

                if self.boto4.is_pressed and but4 == 0:
                    but4 = 1
                    print('Premut 4')
                else:
                    if but4 == 1 and self.boto4.is_pressed == False:
                        but4 = 0

                if self.boto5.is_pressed and but5 == 0:
                    but5 = 1
                    print('Premut 5')
                else:
                    if but5 == 1 and self.boto5.is_pressed == False:
                        but5 = 0

                if self.boto6.is_pressed and but6 == 0:
                    but6 = 1
                    print('Premut 6')
                else:
                    if but6 == 1 and self.boto6.is_pressed == False:
                        but6 = 0
                        print('No premut 6')

                if self.boto7.is_pressed and but7 == 0:
                    but7 = 1
                    print('Premut 7')
                    self.e1.resetValue()
                    rotary_value = 0
                else:
                    if but7 == 1 and self.boto7.is_pressed == False:
                        but7 = 0

                if self.boto8.is_pressed and but8 == 0:
                    but8 = 1
                    print('Premut 8')
                    self.e1.resetValue()
                    rotary_value = 0
                else:
                    if but8 == 1 and self.boto8.is_pressed == False:
                        but8 = 0

                if self.connexio.available():
                    resposta_b = self.connexio.readcmd()
#                    print(resposta_b)
                    resposta = str(resposta_b.decode())
                    resposta = resposta.rstrip()
                    if resposta == '':
                        connectat = False
                    else:
                        #print(resposta)
                        self.rebut.emit(resposta)
                        connectat = True
                    self.connexio.envia(b'$CMD,%d,%d,%d,%d,%d,%d,%d\n'
                                         % (val1,val0,val2,but7,but1,but2,but6))
                    self.joypos.emit('v:%d, g:%d, r:%d' % (val1,val0,val2))
                    # print('$CMD,%d,%d,%d,%d,%d,%d\n'% (val1,val0,but7,but1,but2,but3))
                    # print('Ch0-gir: %d, Ch1-vel: %d\n' %(self.chan0.value, self.chan1.value))
                #time.sleep(0.05)

        finally:
            print("Acaba bucle")
            self.finished.emit()

    # @pyqtSlot(str)
    def acaba(self):
        self.volta = False
