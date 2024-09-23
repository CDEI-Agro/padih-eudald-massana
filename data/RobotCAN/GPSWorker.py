import time
import csv
import os
from serial import Serial
# from ublox_gps import UbloxGps
# from pynmeagps import NMEAReader
import math
from numpy import interp
from PyQt5.QtCore import QObject, QThread, pyqtSignal
import globals
import geopandas as gpd
from geojson import Feature, FeatureCollection, Point, dump
import json
# from geopy.distance import geodesic
from shapely.geometry import LineString

minAngle = 1.0
Kp = 4.0
Kd = 3.0
Ki = 0.0
Kpa = 6.0
Kda = 5.0
Kia = 0.0
interdist = 3.0
distmindesti = 0.3
distcaptura = 0.8

# Calcula l'angle entre dos coordenades
def angle2punts(PuntA, PuntB):
    '''Torna l'angle entre dues coordenades
    d'una manera correcte'''
    lon1 = math.radians(PuntA[0])
    lat1 = math.radians(PuntA[1])
    lon2 = math.radians(PuntB[0])
    lat2 = math.radians(PuntB[1])

    delta1 = lon2-lon1
    term1 = math.sin(delta1) * math.cos(lat2)
    term2 = math.cos(lat1) * math.sin(lat2)
    term3 = math.sin(lat1) * math.cos(lat2) * math.cos(delta1)
    rad = math.atan2(term1, (term2-term3))
    bearing = math.degrees(rad)
    return (bearing + 360) % 360

# Mesura la distancia entre dues coordenades
def haversine(coord1: object, coord2: object):
    '''Torna la distancia euclidiana entre dos punts en metres'''

    # Coordinates in decimal degrees (e.g. 2.89078, 12.79797)
    lon1, lat1 = coord1
    lon2, lat2 = coord2

    R = 6371000  # radius of Earth in meters
    phi_1 = math.radians(lat1)
    phi_2 = math.radians(lat2)

    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)

    a = math.sin(delta_phi / 2.0) ** 2 + math.cos(phi_1) * math.cos(phi_2) * math.sin(delta_lambda / 2.0) ** 2
    
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    meters = R * c  # output distance in meters
    return meters


def graus_decimal(x):
    '''Tranforma la coordenada de format NMEA ddmm.mmmm a graus en decimal'''
    graus = int(x) // 100
    minuts = x -100*graus
    num = graus + minuts/60
    return math.trunc(100000000.0 * num) / 100000000.0
    # return float(graus + minuts/60)

def truncate(number, digits) -> float:
    # Improve accuracy with floating point operations, to avoid truncate(16.4, 2) = 16.39 or truncate(-1.13, 2) = -1.12
    nbDecimals = len(str(number).split('.')[1]) 
    if nbDecimals <= digits:
        return number
    stepper = 10.0 ** digits
    return math.trunc(stepper * number) / stepper


class GpsWorker(QObject):
    finished = pyqtSignal()
    err = pyqtSignal(str)
    coord = pyqtSignal(str)
    fitxer = pyqtSignal(str)
    insertaLlista = pyqtSignal(str)
    fitxgrava = pyqtSignal(str)
    neteja = pyqtSignal()
    mouRobot = pyqtSignal(str)
#    giraRobot = pyqtSignal(str)
    mouMsg = pyqtSignal(str)
    pasMsg = pyqtSignal(str)
    pasPara = pyqtSignal(str)
    destiMsg = pyqtSignal(str)
    enviacoor = pyqtSignal(str)
    kmhGPS = pyqtSignal(float)
    # anglesMsg = pyqtSignal(str)
    angleOk = pyqtSignal(int)
    somAdesti = pyqtSignal()
    intermitent = pyqtSignal()
    
    NoSerie = False

    OrangePi = True
    try:
        if OrangePi == True:
            GPSstream = Serial('/dev/ttyS0', 115200, timeout=0.2) # port serie del GPS
        else:
            os.system('sudo systemctl stop serial-getty@ttyAMA1')
            os.system('sudo chmod 666 /dev/ttyAMA1')
            GPSstream = Serial('/dev/ttyAMA1', 115200, timeout=0.2)

    except Exception as ex:
        print('Port serie no trobat' + ex)
        NoSerie = True

    puntDesti = (0.0, 0.0)
    
    coordenades = []
    numord = 0
    numordcapt = 0
    numlec = 0
    numitems = 0
    mouAuto_premut = False
    puntActual = (0.0, 0.0)
    angleActual = 0.0
    puntVell = (0.0, 0.0)
    difvell = 0
    difsum = 0
    antenaVirtual = True
    angle_robot = False # True si l'angle es del robot
    tintermitent = 0.0
    interpunts = False
    gir_pas = True
    ninterpunts = 0
    Llistainterpunts = []

    def run(self):
        self.volta = True
        """Bucle principal"""
        if self.NoSerie is True:
            self.err.emit("No port serie de GPS")
            return
        try:
            while self.volta:   #bucle principal a ritme de coordenades
                if(time.time() - self.tintermitent > 0.5): # intermitent cada 0.5 segons
                    self.tintermitent = time.time()
                    self.intermitent.emit()

                try:
                    lon, lat, ang, qualitat, speed = self.posicio() # llegim posicio del GPS
                    if qualitat == 0:
                        self.enviacoor.emit('0.0,0.0,0.0,0,0.0\n')
                        self.coord.emit("Lon 0.0, Lat 0.0, d: 0.0, a: 0, Q: 0, V: 0")
                        # print("Sense coordenades")
                        continue
                    if ang > 0 and self.antenaVirtual is True:
                        lon, lat = calculate_new_coordinates(lon, lat, ang, 0.45) # calcula la posicio de l'antena virtual es sobre l'eix de les rodes
                    self.puntActual = (lon, lat)
                    if(self.numlec == 0):   #guardem el primer punt
                        self.puntVell = self.puntActual
                    self.numlec += 1
                    dist = haversine(self.puntVell, self.puntActual)
                    if ang >= 0:
                        if self.angle_robot is False:
                            self.angle_robot = True
                            self.angleOk.emit(1)
                        orig_ang = 'R' # llegit del robot
                        self.angleActual = ang
                    else:
                        if self.angle_robot is True:
                            self.angle_robot = False
                            self.angleOk.emit(0)
                        orig_ang = 'C' # calculat
                        self.angleActual = angle2punts(self.puntVell, self.puntActual)
                    if globals.reverse == True: # girem l'angle 180º
                        self.angleActual = (self.angleActual + 180) % 360
                    self.coord.emit("%d Lon %0.8f, Lat %0.8f, d: %0.3f, a: %0.3f-%s, Q: %d, V:%0.2f" 
                        % (self.numlec, lon, lat, dist, self.angleActual, orig_ang, qualitat, speed))
                    self.kmhGPS.emit(speed)
                    self.puntVell = self.puntActual
                    self.enviacoor.emit('%0.8f,%0.8f,%0.3f,%d,%0.1f\n' %(lon, lat, self.angleActual,qualitat,speed))

                    if globals.captura: # Capturem les coordenades
                        distCapt = haversine(self.puntAnteriorCaptura, self.puntActual)
                        if distCapt > 0.8: # 80 cm.
                            self.puntAnteriorCaptura = self.puntActual
                            self.numordcapt += 1
                            # nova_dada = (self.numord, parsed_data.lon, parsed_data.lat, dist, angle)
                            nova_dada = (self.numordcapt, lon, lat)
                            print(nova_dada)
                            #self.coordenades.append(self.puntNou)
                            self.coordenades.append(nova_dada)

                    if globals.testGir is True: # gira el robot
                        if self.gira_Robot() is True:
                            globals.testGir = False
                            self.somAdesti.emit() # atura el robot
                            #self.mouRobot.emit("0.0,0.0,0")
                            print("Gir acabat")

                    if globals.mouAuto is True and globals.pausaAuto is False:    #Movem automaticament a desti
                        # print("entrem a mouAuto")
                        if globals.debug is True:
                            self.angle_robot = True # estem en debug
                        if self.angle_robot is True:
                            if globals.movent is True:
                                if self.ves_a_punt() is True:
                                    # os.system("aplay -D hw:2,0 Sensors/so1.wav &")
                                    if self.numord < self.numitems:
                                        # si hi ha coordenades llegim la primera
                                        self.puntDesti = (globals.rutaactual[self.numord][1], globals.rutaactual[self.numord][2])
                                        print("Nou punt:", self.numord)
                                        #aqui hem de mirar quin tipus de punt es (gir o pas) si es pas executar ordres
                                        if globals.rutaactual[self.numord][3] == 'G': # si es un punt de gir
                                            globals.movent = False
                                            globals.girant = True
                                        else:
                                            globals.movent = True
                                            globals.girant = False
                                        self.numord += 1
                                    else:
                                        globals.mouAuto = False
                                        # self.mouRobot.emit("0.0,0.0,0") # atura el robot
                                        self.somAdesti.emit() # atura el robot
                                        globals.movent = False
                                        globals.girant = False
                                        print("Som a desti")
                            if globals.girant is True:
                                if self.gira_Robot() is True:
                                    globals.girant = False
                                    globals.movent = True
                                    print("Gir acabat")
                        else:
                            print("No tenim angle")
                            # self.mouRobot.emit("0.0,0.0,0")
                            self.somAdesti.emit() # atura el robot
                            globals.mouAuto = False
                            globals.movent = False
                            globals.girant = False
                            self.err.emit("No tenim angle")

                except Exception as ex:
                    print("Err: %s" % ex)
                    self.err.emit("Err: %s" % ex)
                    continue

        finally:
            # Ensure the background thread is always terminated when we are done
            print("Acaba bucle GPS")
            self.finished.emit()

    def atura(self):
        print("Surt del bucle GPS")
        globals.mouAuto = False
        self.volta = False

    def capturaPunt(self):
        '''Captura la posició actual i la inserta a la llista'''
        #print('Gw_capturaPunt')
        if self.gir_pas is True:
            tipus = 'G'
        else:
            tipus = 'P'
        self.numord += 1
        # nova_dada = (self.numord, parsed_data.lon, parsed_data.lat, dist, angle)
        nova_dada = (self.numord, self.puntActual[0], self.puntActual[1], tipus)
        # print(nova_dada)
        globals.rutaactual.append(nova_dada)
        self.insertaLlista.emit("%d, %0.8f, %0.8f, %s" %(self.numord,  self.puntActual[0], self.puntActual[1],tipus))

    def treuPunt(self):
        '''Treiem el punt de la llista'''
        globals.rutaactual.remove(globals.rutaactual[-1])
        globals.mwin.ui.listWidget.takeItem(globals.mwin.ui.listWidget.count()-1)
        self.numord -= 1
        # print(globals.rutaactual)


    def grava(self):
        '''Grava la llista de coordenades a un fitxer.'''
        features = []
        globals.captura = False
        #print("Gravem coordenades")
        fitxeragravar = time.strftime("coords/Coordenades_%d-%m_%H-%M.csv")
        self.fitxgrava.emit(fitxeragravar)
        with open(fitxeragravar, mode='w') as coords:
            csv_out = csv.writer(coords)
            csv_out.writerow(['Ord','Longitud', 'Latitud', 'Tipus'])
            for row in globals.rutaactual:
                csv_out.writerow(row)
                longitude = row[1]
                latitude = row[2]
                features.append(
                    Feature(
                        geometry = Point((longitude, latitude),precision = 8),
                        properties = {
                        'ord': row[0],
                        'tipus': row[3]
                        }
                    )
                )
        collection = FeatureCollection(features)

        fileout = fitxeragravar.replace('csv','geojson')
        with open(fileout, 'w') as f:
            dump(collection, f, indent = 4)

    def obreFitxer(self,fileName):
        print(fileName)

        # self.coordenades.clear()
        globals.rutaactual.clear()
        #self.neteja.emit() ???

        if fileName.find("geojson") == -1:
            self.fitxer.emit(fileName)
            with open(fileName, newline='') as csvfile:
                coordreader = csv.reader(csvfile) #, delimiter=' ', quotechar='|')
                for row in coordreader:
                    ord = row[0]
                    lon = row[1]
                    lat = row[2]
                    tipus = row[3]
                    if ord.isnumeric() is False: # Saltem la primera linia
                        continue
                    self.insertaLlista.emit("%s, %s, %s, %s" %(ord,lon,lat,tipus))
                    # (int(row(0)),float(b), float(c)) = row
                    #self.coordenades.append((int(a),float(b),float(c)))
                    globals.rutaactual.append((int(ord),float(lon),float(lat),tipus))
        else:
            features = gpd.read_file(fileName)
            print(features)
            for id, punt in features.iterrows():
                self.insertaLlista.emit("%s, %0.8f, %0.8f, %s" %(punt.ord, punt.geometry.x, punt.geometry.y, punt.tipus))
                #self.coordenades.append((punt.ord, punt.geometry.x, punt.geometry.y))
                globals.rutaactual.append((int(punt.ord), punt.geometry.x, punt.geometry.y, punt.tipus))
                print(id, punt.ord, punt.geometry.x, punt.geometry.y)

        print(globals.rutaactual)

    def mouAutomatic(self):
        '''Activa el moviment del robot
            Si hi ha elemnts a la llista va movent als punts
            si no llegeix el fitxer desti.txt i va cap a la coordenada llegida,
            Crea un fitxer de log'''
        # llegim coordenades de la llista rutaactual
        self.numitems = len(globals.rutaactual)
        self.fitxlog = time.strftime("logs/logAuto_%d-%m_%H-%M.log")
        self.flog = open(self.fitxlog,'w')
        iniline=("ord,lon,lat,temps,dist,Adesti,Actual,Adif,PID,Velocitat,Gir\n")
        self.flog.write(iniline)
        self.flog.close()
        self.initemps = time.time()
        features = []
        features.append(
            Feature(
                geometry = Point((self.puntActual[0], self.puntActual[1]),precision = 8),
                properties = {
                    'ord': 0,
                    'tipus': 'G'
                }
            )
        )
        collection = FeatureCollection(features)
        fileout = self.fitxlog.replace('.log','_log.geojson')
        with open(fileout, 'w') as f:
            dump(collection, f, indent = 4)

        if self.numitems > 0:
            # print("Items: ", self.numitems)
            self.numord = globals.rutaactual[0][0]
            self.puntDesti = (globals.rutaactual[0][1], globals.rutaactual[0][2])
            globals.mouAuto = True
            if globals.debug is True:
                globals.movent = True
                globals.girant = False
            else:
                globals.movent = False
                globals.girant = True
            print(self.puntDesti)

        else:
            self.err.emit("Llista de coordenades buida")
            globals.mouAuto = False
            globals.deltagir = 0.0
            self.mouRobot.emit("0.0,0.0,0") # gir a 0
        #print('Mou pulsat')

    def giraAuto(self):
        '''Activa el gir del robot
            per proves'''
        self.fitxlog = time.strftime("logs/logGir_%d-%m_%H-%M.log")
        self.flog = open(self.fitxlog,'w')
        iniline=("ord,lon,lat,temps,Adesti,Actual,Adif,PID,Gir\n")
        self.flog.write(iniline)
        self.flog.close()
        self.initemps = time.time()
        if len(globals.rutaactual) > 0:
            self.numord = globals.rutaactual[0][0]
            self.puntDesti = (globals.rutaactual[0][1], globals.rutaactual[0][2])
            globals.testGir = True
        else:
            self.err.emit("Llista de coordenades buida")
            globals.testGir = False

    def gira_Robot(self):
        angleDesti = angle2punts(self.puntActual, self.puntDesti)
        dif = angleDesti - self.angleActual
        if dif < 0:
            if dif <= -180:
                dif += 360
        else:
            if dif >= 180:
                dif -= 360
        if abs(dif) < minAngle:
            self.mouRobot.emit("0.0,0.0,1")
            # self.giraRobot.emit("0.0")
            return True
        else:
            pid = dif * Kpa + (dif - self.difvell) * Kda + self.difsum * Kia
            self.difvell = dif
            self.difsum += dif
            if pid > 0:
                gir = interp(pid,[0,180],[0,1])
            else:
                gir = interp(pid,[-180,0],[-1,0])
            # if dif < 0:
            #     if dif > -10:
            #         gir = -0.3
            #     else:
            #         gir = -1.0
            # else:
            #     if dif < 10:
            #         gir = 0.3
            #     else:
            #         gir = 1

            #self.giraRobot.emit("%.3f" %(gir))
            self.mouRobot.emit("0.0,%.3f,1"  %(gir))
            self.fes_log(angleDesti, dif, pid, gir)
            return False
         
    def ves_a_punt(self):
        '''Mou el robot al seguent punt de la llista 
        si la distancia entre punts es mes gran que interdist interpola punts intermitjos'''

        if self.interpunts is False:
            angleDesti = angle2punts(self.puntActual, self.puntDesti)
            distDesti = haversine(self.puntActual, self.puntDesti)
            print("Distancia: %.3f" % distDesti)
            if distDesti > interdist:
                self.interpunts = True
                self.Llistainterpunts = interpola_geo_punts(self.puntActual, self.puntDesti, interdist)
                # self.Llistainterpunts.remove(self.Llistainterpunts[0]) # treiem el primer punt
                self.ninterpunts = len(self.Llistainterpunts)
                print(self.Llistainterpunts)
            else:
                if distDesti < distmindesti: # A 30cm. del desti s'atura o agafa seguent coordenada
                    dif = angleDesti - self.angleActual
                    pid = 0.0
                    velo = 0.0
                    gir = 0.0
                    self.difsum = 0
                    self.fes_log(angleDesti, dif, pid, gir)
                    print("som a punt %d" % self.numord)
                    return True
        else:
            if self.ninterpunts > 0:
                self.puntinterDesti = self.Llistainterpunts[0]
                self.ninterpunts = len(self.Llistainterpunts)
                angleDesti = angle2punts(self.puntActual, self.puntinterDesti)
                distDesti = haversine(self.puntActual, self.puntinterDesti)
                print("Distancia inter : %.3f npinter: %d" % (distDesti, self.ninterpunts))
                if distDesti < distmindesti:
                    print(self.puntinterDesti)
                    self.difsum = 0
                    self.Llistainterpunts.remove(self.Llistainterpunts[0])
                    self.ninterpunts = len(self.Llistainterpunts)
                    if self.ninterpunts == 0:
                        self.interpunts = False
                        return True
                    else:
                        return False

        dif = angleDesti - self.angleActual
        if dif < 0:
            if dif <= -180:
                dif += 360
        else:
            if dif >= 180:
                dif -= 360
        pid = dif * Kp + (dif - self.difvell) * Kd + self.difsum * Ki
        self.difvell = dif
        self.difsum += dif
        if pid > 0:
            gir = interp(pid,[0,180],[0,1])
        else:
            gir = interp(pid,[-180,0],[-1,0])
        #print("gir %.3f" %(gir))
        if globals.velofix is True:                          
            velo = globals.velofix
        else:
            velo = globals.deltavelo
        globals.deltagir = gir
        self.fes_log(angleDesti, dif, pid, gir, velo)
        self.mouRobot.emit("%.3f,%.3f,%d" %(velo, gir, False))
        # print("Envia %.3f,%.3f" %(velo, gir))
        return False
    
    def posicio(self):
        ''' torna la posicio actual, lon, lat, angle, qualitat i velocitat'''
        gga = 0
        hdt = 0
        speed = 0.0
        while gga ==0 or hdt == 0:
            nmea_bytes = self.GPSstream.readline()        
            nmea_string = str(nmea_bytes.decode())  
            nmea_string = nmea_string.rstrip()
            # print(nmea_string)
            # speed = 0.0
            if (nmea_string.startswith('$GPGGA')):
                gga = 1
                nmea_array = [element.strip() for element in nmea_string.split(',')]
                Quality_Indicator = int(nmea_array[6])
                if Quality_Indicator == 0:
                    Latitude = 0.0
                    Longitude = 0.0
                    # print("GGA No Fixat!") # NMEA GGA Quality indicator = 0 means no fix available
                else:
                    # Parse NMEA GGA message
                    Latitude = graus_decimal(float(nmea_array[2]))
                    Longitude = graus_decimal(float(nmea_array[4]))
                    Height = float(nmea_array[9])
                # time.sleep(0.1)
            elif (nmea_string.startswith('$GPVTG')):
                #print(nmea_string)
                nmea_array = [element.strip() for element in nmea_string.split(',')]
                #angleNMEA = float(nmea_array[1])
                if nmea_array[7] == '':
                    speed = 0.0
                else:
                    speed = float(nmea_array[7])
                # Print coordinates
                #print(' AngleNMEA: ' + str(angleNMEA))
                #print(' Velocitat: ' + str(speed))
                # time.sleep(0.1)
            elif (nmea_string.startswith('$GPHDT')):
                hdt = 1
                nmea_array = [element.strip() for element in nmea_string.split(',')]
                angle = nmea_array[1]
                if angle == '':
                    Angle = -1.0
                else:
                    Angle = float(angle)
                    # print('Angle: ' + angle)
        
        return Longitude, Latitude, Angle, Quality_Indicator, speed

    def fes_log(self, angleDesti, dif, pid, gir, velo=0.0):
        # "ord,lon,lat,temps,Adesti,Actual,Adif,PID,Gir\n"
        linelog = ('%d,%.8f,%.8f,%.2f,%.3f,%.3f,%.3f,%.3f,%.3f\n'
                %(self.numord, self.puntActual[0], self.puntActual[1], time.time()-self.initemps, angleDesti, self.angleActual, dif, pid, gir))
        self.flog = open(self.fitxlog,'a')
        self.flog.write(linelog)
        self.flog.close()
        features = []
        features.append(
            Feature(
                geometry = Point((self.puntActual[0], self.puntActual[1]),precision = 8),
                properties = {
                    'ord': self.numord,
                    'temps': time.time()-self.initemps,
                    'angleDesti': angleDesti,
                    'angleActual': self.angleActual,
                    'dif': dif,
                    'pid': pid,
                    'gir': gir,
                    'velo': velo
                }
            )
        )
        fileout = self.fitxlog.replace('.log','_log.geojson')
        add_features(fileout, features)

def calculate_new_coordinates(lon, lat, angle, distance):
    # Convertimos los grados a radianes
    lat = math.radians(lat)
    lon = math.radians(lon)
    angle = math.radians(angle)

    # Radio de la Tierra en metros
    earth_radius = 6371000

    # Calculamos las nuevas coordenadas
    new_lat = math.asin(math.sin(lat) * math.cos(distance / earth_radius) + math.cos(lat) * math.sin(distance / earth_radius) * math.cos(angle))
    new_lon = lon + math.atan2(math.sin(angle) * math.sin(distance / earth_radius) * math.cos(lat), math.cos(distance / earth_radius) - math.sin(lat) * math.sin(new_lat))

    # Convertimos las coordenadas de vuelta a grados
    new_lat = math.degrees(new_lat)
    new_lon = math.degrees(new_lon)

    return new_lon, new_lat

def calcula_nova_coordenada(Point, angle, distance):
    # Convertimos los grados a radianes
    lat = math.radians(Point[1])
    lon = math.radians(Point[0])
    angle = math.radians(angle)

    # Radio de la Tierra en metros
    earth_radius = 6371000

    # Calculamos las nuevas coordenadas
    new_lat = math.asin(math.sin(lat) * math.cos(distance / earth_radius) + math.cos(lat) * math.sin(distance / earth_radius) * math.cos(angle))
    new_lon = lon + math.atan2(math.sin(angle) * math.sin(distance / earth_radius) * math.cos(lat), math.cos(distance / earth_radius) - math.sin(lat) * math.sin(new_lat))

    # Convertimos las coordenadas de vuelta a grados
    new_lat = math.degrees(new_lat)
    new_lon = math.degrees(new_lon)

    return (new_lon, new_lat)


def interpola_geo_punts(point1, point2, dist):
    n = int(haversine(point1, point2)/dist)
    angle = angle2punts(point1,point2) 
    interpolated_points = []
    distance = haversine(point1, point2)/n
    current_point = point1
    for i in range(n):
        punt_nou  = calcula_nova_coordenada(current_point, angle, distance)
        interpolated_points.append(punt_nou)
        current_point = punt_nou

    return interpolated_points


def add_features(geojson_file, new_features):
    with open(geojson_file, 'r+') as file:
        # Load existing data
        data = json.load(file)
        # Append new features to the 'features' field
        data['features'].extend(new_features)
        # Move the pointer to the beginning of the file
        file.seek(0)
        # Write the updated data back to the file
        json.dump(data, file, indent=4)
        # Truncate the file to the current position to remove leftover part
        file.truncate()
