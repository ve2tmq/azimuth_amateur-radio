#!/usr/bin/env python3

import math, numpy, statistics
from scipy import stats
import serial
import io
import datetime
import sys
import threading
import http.server
from functools import partial
import sys
from time import sleep
import configparser
from signal import signal, SIGINT
from gpiozero import Button, LED
from ft8x7 import FT8x7
from smbus import SMBus
from buttonLED import ButtonLED
#from i2clibraries import i2c_hmc5883l

EarthRadius = 6371 # Km

def handler(signal_received, frame):
    # Handle any cleanup here
    print('SIGINT or CTRL-C detected. Exiting gracefully')
    exit(0)

class httpServer:
    def __init__(self,  port = 8000, path = None):
        self.port = port
        self.path = path
        self.handler_class = partial(http.server.SimpleHTTPRequestHandler, directory=path)
        
    def run(self):        
        with http.server.HTTPServer(("", self.port), self.handler_class) as httpd:
            print("serving at port", self.port)
            httpd.serve_forever()



class plot:
    """
    Calculate plot from Point, bearing and distance.
    :Parameters:
     - QTH: The tuple representing the latitude/longitude for current
        point. Latitude and longitude must be in decimal degrees.
     - bearing: Degrees from QTH read from doppler.
     - azimuth: My azimuth, to calculate real bearing.
     - distance: Distance (km) from QTH.
    :Return:
     Coordinates
    """

    latitude = None
    longitude = None
    bearing = None
    distance = None
    
    def __init__(self, QTH, bearing, azimuth, distance):
        if (type(QTH) != tuple):
            raise TypeError("Only tuples are supported as arguments")

        # Calculate offset between bearing my mobile bearing and plot bearing.
        self.bearing = azimuth + bearing
        if(self.bearing > 360):
            self.bearing = self.bearing % 360

        self.bearing = math.radians(self.bearing)
        self.latitude = math.radians(QTH[0])
        self.longitude = math.radians(QTH[1])
        self.distance = distance
    
    def get_plot(self):
        lat = math.asin( math.sin(self.latitude) * math.cos(self.distance/EarthRadius) + math.cos(self.latitude) * math.sin(self.distance/EarthRadius) * math.cos(self.bearing))
        lon = self.longitude + math.atan2(math.sin(self.bearing) * math.sin(self.distance/EarthRadius) * math.cos(self.latitude), math.cos(self.distance/EarthRadius)-math.sin(self.latitude) * math.sin(lat))
        return float(format(math.degrees(lat), ".6f")), float(format(math.degrees(lon), ".6f"))
        


class read_serial:
    ser = None
    sio = None
    
    def __init__(self, port = "/dev/ttyUSB0", baudrate = 4800):
        self.ser = serial.Serial(port, baudrate, timeout = 0.5)
        self.sio = io.TextIOWrapper(io.BufferedRWPair(self.ser, self.ser))
    
    def readSerial(self):
        self.sio.flush()
        return self.sio.readline()

'''
Thread Doppler
'''
class Doppler(read_serial):
    dFlag = False
    
    """
    Thread
    Read doppler from serial port
    Bearing: int (degree)
    Quality: int [0-9]
    """
    def readDoppler(self, s = None):
        if not(s):
            s = self.readSerial()
        try:
            if s[0] == "%":
                data = s.split("/")
                bearing = int(data[0][1:])
                quality = int(data[1])
                return (bearing, quality)
        except:
            return None

    def run(self):
        print("Doppler started")
        while True:
            bearing = self.readDoppler()
            if(bearing):
                self.bearing = bearing
                self.dFlag = True
                print(bearing)



class GPS(read_serial):
    azimuth = 0
    speed = 0
    QTH = (0, 0)

    def __init__(self, GPSDev, GPSBaud, degrees = 0, minutes = 0):
        super().__init__(GPSDev, GPSBaud)
        #self.compass = i2c_hmc5883l.i2c_hmc5883l(1)
        #self.compass.setContinuousMode()
        #self.compass.setDeclination(degrees, minutes)

    """
    Approximate calculation distance (in meter)
    (expanding the trigonometric functions around the midpoint
    https://github.com/gojuno/geo-py/blob/master/geo/sphere.py
    """
    def get_distance(self, QTH0, QTH1):
        lat1 = math.radians(QTH0[0])
        lat2 = math.radians(QTH1[0])
        lon1 = math.radians(QTH0[1])
        lon2 = math.radians(QTH1[1])
        
        cos_lat = math.cos((lat1 + lat2) / 2.0)
        dx = (lat2 - lat1)
        dy = (cos_lat * (lon2 - lon1))

        return EarthRadius * math.sqrt(dx**2 + dy**2)/1000

    """
    Calculates the azimuth between two points.
    https://www.movable-type.co.uk/scripts/latlong.html
    https://gist.github.com/jeromer/2005586

    Magnetic Compass HMC_5883L
    https://tutorials-raspberrypi.com/build-your-own-raspberry-pi-compass-hmc5883l/
    https://www.electronicwings.com/sensors-modules/hmc5883l-magnetometer-module

    :Parameters:
      - `QTH: The tuple representing the latitude/longitude for the
        first point. Latitude and longitude must be in decimal degrees
    :Returns:
      The heading azimuth in degrees
    :Returns Type:
      int
    """
    def get_azimuth(self, QTH0, QTH1):
        if self.speed >= 1:
            lat1 = math.radians(QTH0[0])
            lat2 = math.radians(QTH1[0])

            diffLong = math.radians(QTH1[1] - QTH0[1])

            x = math.sin(diffLong) * math.cos(lat2)
            y = math.cos(lat1) * math.sin(lat2) - (math.sin(lat1) * math.cos(lat2) * math.cos(diffLong))

            heading = math.atan2(x, y)

            # Now we have the initial bearing but math.atan2 return values
            # from -180° to + 180° which is not what we want for a compass bearing
            # The solution is to normalize the initial bearing as shown below
            heading = int( ( math.degrees(heading) + 360) % 360 )

        else:
            heading = self.azimuth

        return heading
    
    def getspeed(self, GPVTG):
        return float(GPVTG[7])


    """
    Thread
    Read GPS from serial port
    coordinate: tuple (lat, lon)
    """
    def readGPS(self, GPGGA):
        time = GPGGA[1]
        
        latDMm = float(GPGGA[2])/100
        DD = int(latDMm)
        Mm = ((latDMm - DD) * 100)/60
        lat = float(format(DD + Mm, '.6f'))
        if GPGGA[3] == 'S':
            lat = -lat
        
        lonDMm = float(GPGGA[4])/100
        DD = int(lonDMm)
        Mm = ((lonDMm - DD) * 100)/60
        lon = float(format(DD + Mm, '.6f'))
        if GPGGA[5] == 'W':
            lon = -lon
        
        return (lat, lon)
        
    def run(self):
        QTH = None
        print("GPS started")
    
        while True:
            try:
                s = self.readSerial()
                data = s.split(",")
                
                if data[0] == "$GPVTG":
                    self.speed = self.getspeed(data)
    
                elif data[0] == "$GPGGA":
                    self.QTH = self.readGPS(data)
                    
                    if(QTH):
                        self.azimuth = self.get_azimuth(QTH, self.QTH)
                    QTH = self.QTH

            except:
                continue


'''
Thread Doppler, use GPS in Doppler
'''
class DopplerGPS(Doppler, GPS):
    def run(self):
        print("Doppler and GPS started")
        QTH1 = None

        while True:
            try:
                s = self.readSerial()

                if len(s) > 0 and s[0] == '$':
                    data = s.split(",")
                    if data[0] == "$GPGGA":
                        QTH = self.readGPS(data)
                        self.QTH = QTH
                    
                        if QTH1:
                            self.azimuth = self.get_azimuth(QTH, QTH1)
                        QTH1 = QTH
                    
                        print(QTH, self.azimuth)
                
                elif s[0] == "%":
                    bearing = self.readDoppler(s)
                    if(bearing):
                        self.bearing = bearing
                        self.dFlag = True
                        print(bearing)

            except:
                continue


'''
I2C 4-channels 100 kohms potentiometer.
Virtual rotate antennas.
Read S-Meter after each antenna position.
'''
class ant4:
    address = 0x2f
    #antA: P0, TCON0
    #antB: P1, TCON0
    #antC: P3, TCON1
    #antD: P2, TCON1
    #ant = (Register, TCON-on, degrees bearing)
    antA = (0x00, 0xf, 0, 'antA') 
    antB = (0x10, 0xf0, 90, 'antB')
    antC = (0x70, 0xf0, 180, 'antC')
    antD = (0x60, 0xf, 270, 'antD')
    readTCON0 = 0x4d
    writeTCON0 = 0x41
    readTCON1 = 0xad
    writeTCON1 = 0xa1
    Quad = [(antA, antB), (antB, antC), (antC, antD), (antD, antA)]
    step = 1
    cycle = 0
    ready = False

    def __init__(self, rigcat):
        self.bus = SMBus(1)
        self.dimm(self.antA, 0)
        self.dimm(self.antB, 0)
        self.dimm(self.antC, 0)
        self.dimm(self.antD, 0)
        self.rigcat = rigcat
        self.Array = list()

    '''
    TCON: get status of terminal (Off/On)
    '''
    def readTCON(self, ant):
        if ant == self.antA or ant == self.antB:
            data = self.bus.read_word_data(self.address, self.readTCON0) >> 8
        elif ant == self.antC or ant == self.antD:
            data = self.bus.read_word_data(self.address, self.readTCON1) >> 8

        return data

    def turnOFF(self, ant):
        data = self.readTCON(ant)
        if ant == self.antA or ant == self.antB:
            self.bus.write_byte_data(self.address, self.writeTCON0, data & ~ant[1])
        elif ant == self.antC or ant == self.antD:
            self.bus.write_byte_data(self.address, self.writeTCON1, data & ~ant[1])

    def turnON(self, ant):
        data = self.readTCON(ant)
        if ant == self.antA or ant == self.antB:
            self.bus.write_byte_data(self.address, self.writeTCON0, data | ant[1])
        elif ant == self.antC or ant == self.antD:
            self.bus.write_byte_data(self.address, self.writeTCON1, data | ant[1])

    def dimm(self, ant, i):
        self.bus.write_byte_data(self.address, ant[0], i)

    def rotate(self, antA, antB, i):
        self.dimm(antA, 254 - i)
        self.dimm(antB, i)
        #print((i, 254 - i))
        return ( (i / 254 * 90) + antA[2] ) % 360

    def test(self, ant, i=255):
        for a in [self.antA, self.antB, self.antC, self.antD]:
            if ant is a:
                self.turnON(a)
                self.dimm(a, i)
            else:
                self.turnOFF(a)

    def get_bearing(self):
        Array = self.Array.copy()

        # Find max S-Meter value
        MAX = max(Array, key=lambda x:x[1])

        # Create sub array of deg for MAX values.
        degArray = list()
        for i in Array:
            if i == MAX:
                degArray.append(i[0])

        r = abs(stats.spearmanr(Array).correlation)

        # Calculate bearing
        return (statistics.median(degArray), MAX[1], r)

    def reset(self):
        self.cycle = 0
        self.ready = False

    def run(self):
        print("4ant started")
        Array = list()
        self.ready = False
        while True:
            step = int(256 / 2 ** (self.cycle) + self.step)
            self.rigcat.read_rx_status()
            if self.rigcat._squelch is False:
                Array.clear()
                for ant in self.Quad:
                    try:
                        for i in range(0, 255, step):
                            bearing = self.rotate(ant[0], ant[1], i)
                            sleep(0.1)
                            self.rigcat.read_rx_status()
                            Array.append( (int(bearing), self.rigcat._s_meter) )
                            if self.rigcat._squelch is True:
                                break
                    
                    except:
                        continue

                    if self.rigcat._squelch is True:
                        break
                
                if self.rigcat._squelch is True:
                    self.Array.clear()
                    self.reset()
                else:
                    self.ready = False
                    sleep(0.25)
                    self.Array = Array.copy()
                    self.ready = True
                    self.cycle = self.cycle +1 if self.cycle < 8 else 8

class kml:
    File = None
    record = list()
    
    def __init__(self, File):
        self.File = File
        with open(File, 'w') as F:
            F.write('<kml></kml>\n')
    
    def writePos(self, QTH, Bearing, Quality):
        timestamp = str(datetime.datetime.now())
        latitude = str(QTH[0])
        longitude = str(QTH[1])
        BearingLat = str(Bearing[0])
        BearingLon = str(Bearing[1])
                
        if Quality == 15:
            color = '000000'
        elif Quality >= 9 and Quality < 15:
            color = '0000ff'
        elif Quality == 8:
            color = '00007f'
        elif Quality == 7:
            color = '0055ff'
        elif Quality == 6:
            color = '002a7f'
        elif Quality == 5:
            color = '00ffff'
        elif Quality == 4:
            color = '007f7f'
        elif Quality == 3:
            color = '00ff00'
        elif Quality == 2:
            color = '007f00'
        elif Quality == 1:
            color = 'ff0000'
        else:
            color = '7f0000'
        
        self.record.append((timestamp, latitude, longitude, BearingLat, BearingLon, color))
        
        with open(self.File, 'w') as F:
            F.write('<Document>\n')
            for R in self.record:
                F.write('<Placemark>\n')
                F.write('<name>' + R[0] + '</name>\n')
                F.write('<Style>\n')
                F.write('<IconStyle>\n')
                F.write('<Icon>\n')
                F.write('<href>http://earth.google.com/images/kml-icons/track-directional/track-0.png</href>\n')
                F.write('</Icon>\n')
                F.write('</IconStyle>\n')
                F.write('</Style>\n')
                F.write('<Point>\n')
                F.write('<coordinates>' + R[2] + ',' + R[1] + ',0</coordinates>\n')
                F.write('</Point>\n')
                F.write('</Placemark>\n')
                F.write('<Placemark>\n')
                F.write('<name>' + R[0] + '</name>\n')
                F.write('<Style>')
                F.write('<LineStyle>')
                F.write('<color>ff' + R[5] + '</color>\n')
                F.write('<width>2</width>\n')
                F.write('</LineStyle>\n')
                F.write('</Style>\n')
                F.write('<LineString>\n')
                F.write('<coordinates>' + R[2] + ',' + R[1] + ',0 ' + R[4] + ',' + R[3] + ',0</coordinates>\n')
                F.write('</LineString>\n')
                F.write('</Placemark>\n')
            F.write('</Document>\n')
            F.close()
        return

class ButtonLED(ButtonLED):
    button_pressed = False

    def run(self):
        while True:
            if self.button.is_pressed:
                self.led.off()
                self.button_pressed = True

    def reset(self):
        self.led.on()
        self.button_pressed = False

def main():
    signal(SIGINT, handler)
    config = configparser.ConfigParser()
    if len(sys.argv) > 1:
        config.read(argv[1])
    else:
        config.read('azimuth.conf')
        
    if 'gps' in config.sections():
        my_GPS = GPS(config['gps']['dev'], int(config['gps']['baud']), -14, 0)
        TGPS = threading.Thread(target=my_GPS.run)
        TGPS.start()

    if 'httpd' in config.sections():
        httpd = httpServer(int(config['httpd']['port']), config['httpd']['root'])
        Thttpd = threading.Thread(target = httpd.run)
        Thttpd.start()
        K = kml(config['httpd']['root'] + "azimuth.kml")
    
    if config['common']['method'] == 'doppler':
        if 'gps' in config.sections():
            my_Doppler = Doppler(config['doppler']['dev'], config['doppler']['baud'])
            TDoppler = threading.Thread(target=my_Doppler.run)
            TDoppler.start()
        else:
            my_Doppler = DopplerGPS(config['doppler']['dev'], config['doppler']['baud'])
            TDoppler = threading.Thread(target=my_Doppler.run)
            my_GPS = my_Doppler
            TDoppler.start()

        while True:        
            if my_Doppler.dFlag:
                p = plot(my_GPS.QTH, my_Doppler.bearing[0], my_GPS.azimuth, int(config['common']['radius']))
                K.writePos(my_GPS.QTH, p.get_plot(), my_Doppler.bearing[1])
                print("-->", my_GPS.QTH, str(int(math.degrees(p.bearing))) + "\u00b0", "Q" + str(my_Doppler.bearing[1]), "<--")
                my_Doppler.dFlag = False
            sleep(1)

    else:
        my_ButtonLED = ButtonLED(18, 17)

        if 'rigcat' in config.sections():
            try:
                rigcat = FT8x7(config['rigcat']['dev'], int(config['rigcat']['baud']))
            except:
                pass

        while True and config['common']['method'] == '1ant':
            my_ButtonLED.reset()
            print(my_GPS.QTH, str(my_GPS.azimuth) + "\u00b0", str(my_GPS.speed) + " km/h")
            sleep(0.25)
            if my_ButtonLED.button_pressed:
                try:
                    rigcat.read_rx_status()
                    s_meter = rigcat._s_meter
                except:
                    s_meter = 0xf

                p = plot(my_GPS.QTH, 0, my_GPS.azimuth, int(config['common']['radius']))
                K.writePos(my_GPS.QTH, p.get_plot(), s_meter)
                print("-->", my_GPS.QTH, str(int(math.degrees(p.bearing))) + "\u00b0", "S" + str(s_meter), "<--")
                sleep(0.25)
        
        if config['common']['method'] == '4ant':
            my_ButtonLED.reset()
            my_4ant = ant4(rigcat)
            T4ant = threading.Thread(target = my_4ant.run)
            T4ant.start()
            while True:
                my_4ant.step = int(my_GPS.speed * 8)
                print(my_GPS.QTH, str(my_GPS.azimuth) + "\u00b0", str(my_GPS.speed) + " km/h")
                sleep(0.25)

                if my_4ant.ready:
                    my_ButtonLED.led.on()
                    bearing = my_4ant.get_bearing()
                    print(bearing)
                else:
                    my_ButtonLED.led.off()

                if my_ButtonLED.button_pressed and my_4ant.ready:
                    my_4ant.ready = False
                    bearing = my_4ant.get_bearing()
                    p = plot(my_GPS.QTH, bearing[0], my_GPS.azimuth, int(config['common']['radius']))
                    K.writePos(my_GPS.QTH, p.get_plot(), bearing[1])
                    print("-->", my_GPS.QTH, str(int(math.degrees(p.bearing))) + "\u00b0", "S" + str(bearing[1]), "<--")
                    sleep(0.25)
                    my_4ant.reset()
                    my_ButtonLED.reset()
    
        if config['common']['method'] == 'test4ant':
            my_4ant = ant4(rigcat)
            while True:
                for a in [my_4ant.antA, my_4ant.antB, my_4ant.antC, my_4ant.antD]:
                    print(a[3])
                    my_ButtonLED.reset()
                    my_4ant.test(a)
                    while not my_ButtonLED.button_pressed:
                        sleep(1)

if __name__ == '__main__':
    main()
