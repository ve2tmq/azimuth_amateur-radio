#!/usr/bin/env python3

import math, numpy

from scipy.stats import circmean
from read_serial import ReadSerial
import datetime
import logging
import threading
import http.server
from functools import partial
import sys
from time import sleep
import configparser
from signal import signal, SIGINT
import Hamlib
from smbus import SMBus
from buttonLED import ButtonLED
from gps import GPS, EARTH_RADIUS
import curses




def handler(signal_received, frame):
    # Handle any cleanup here
    curses.endwin()
    logging.info('SIGINT or CTRL-C detected. Exiting gracefully')
    exit(0)


class httpServer:
    def __init__(self, port=8000, path=None):
        self.port = port
        self.path = path
        self.handler_class = partial(http.server.SimpleHTTPRequestHandler, directory=path)

    def run(self):
        with http.server.HTTPServer(("", self.port), self.handler_class) as httpd:
            logging.info("serving at port", self.port)
            httpd.serve_forever()


class plot:
    """
    Calculate plot from Point, bearing and distance.
    :Parameters:
     - QTH: The tuple representing the latitude/longitude for current
        point. Latitude and longitude must be in decimal degrees.
     - bearing: Degrees from QTH read from doppler.
     - heaging: My heading, to calculate real azimuth.
     - distance: Distance (km) from QTH.
    :Return:
     Coordinates
    """

    latitude = None
    longitude = None
    azimuth = None
    distance = None

    def __init__(self, QTH, bearing, heading, distance):
        if (type(QTH) != tuple):
            raise TypeError("Only tuples are supported as arguments")

        # Calculate offset between bearing and my heading.
        self.azimuth = (heading + bearing) % 360
        self.latitude = math.radians(QTH[0])
        self.longitude = math.radians(QTH[1])
        self.distance = distance

    def get_plot(self):
        lat = math.asin(
            math.sin(self.latitude) * math.cos(self.distance / EARTH_RADIUS) + math.cos(self.latitude) * math.sin(
                self.distance / EARTH_RADIUS) * math.cos(self.azimuth))
        lon = self.longitude + math.atan2(
            math.sin(self.azimuth) * math.sin(self.distance / EARTH_RADIUS) * math.cos(self.latitude),
            math.cos(self.distance / EARTH_RADIUS) - math.sin(self.latitude) * math.sin(lat))
        return float(format(math.degrees(lat), ".6f")), float(format(math.degrees(lon), ".6f"))





class Doppler(ReadSerial):
    """
    Thread Doppler
        Thread
    Read doppler from serial port
    Bearing: int (degree)
    Quality: int [0-9]
    """
    dFlag = False


    def readDoppler(self, s=None):
        if not (s):
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
        logging.info("Doppler started")
        while True:
            bearing = self.readDoppler()
            if (bearing):
                self.bearing = bearing
                self.dFlag = True




class DopplerGPS(Doppler, GPS):
    """
    Thread Doppler, use GPS in Doppler
    """
    def run(self):
        logging.info("Doppler and GPS started")
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
                            self.heading = self.get_heading(QTH, QTH1)
                        QTH1 = QTH


                elif s[0] == "%":
                    bearing = self.readDoppler(s)
                    if (bearing):
                        self.bearing = bearing
                        self.dFlag = True

            except:
                continue



class ant4:
    """
    I2C 4-channels 100 kohms potentiometer.
    Virtual rotate antennas.
    Read S-Meter after each antenna position.
    """
    __address = 0x2f
    __div = 1  # 1, 2 or 4 -- This is division of each quads.
    __ohm_max = 255
    # antA: P0, TCON0
    # antB: P1, TCON0
    # antC: P3, TCON1
    # antD: P2, TCON1
    # ant = (Register, TCON-on, [degrees bearing])
    __antA = (0x00, 0xf)
    __antB = (0x10, 0xf0)
    __antC = (0x70, 0xf0)
    __antD = (0x60, 0xf)
    __readTCON0 = 0x4d
    __writeTCON0 = 0x41
    __readTCON1 = 0xad
    __writeTCON1 = 0xa1
    _deg = 0
    Array = list()

    def __init__(self, rigcat=None, div=1, bearingA=0, bearingB=90, bearingC=180, bearingD=270):
        self.bus = SMBus(1)
        self.__antA += (bearingA,)
        self.__antB += (bearingB,)
        self.__antC += (bearingC,)
        self.__antD += (bearingD,)
        self.__Quad = [(self.__antA, self.__antB), (self.__antB, self.__antC), (self.__antC, self.__antD),
                       (self.__antD, self.__antA)]
        self.dimm(self.__antA, 0)
        self.dimm(self.__antB, 0)
        self.dimm(self.__antC, 0)
        self.dimm(self.__antD, 0)
        self.rigcat = rigcat
        self.__div = div
        self.DArray = dict()

    '''
    TCON: get status of terminal (Off/On)
    '''

    def __readTCON(self, ant):
        if ant == self.__antA or ant == self.__antB:
            data = self.bus.read_word_data(self.__address, self.__readTCON0) >> 8
        elif ant == self.__antC or ant == self.__antD:
            data = self.bus.read_word_data(self.__address, self.__readTCON1) >> 8

        return data

    def turnOFF(self, ant):
        data = self.__readTCON(ant)
        if ant == self.__antA or ant == self.__antB:
            self.bus.write_byte_data(self.__address, self.__writeTCON0, data & ~ant[1])
        elif ant == self.__antC or ant == self.__antD:
            self.bus.write_byte_data(self.__address, self.__writeTCON1, data & ~ant[1])

    def turnON(self, ant):
        data = self.__readTCON(ant)
        if ant == self.__antA or ant == self.__antB:
            self.bus.write_byte_data(self.__address, self.__writeTCON0, data | ant[1])
        elif ant == self.__antC or ant == self.__antD:
            self.bus.write_byte_data(self.__address, self.__writeTCON1, data | ant[1])

    def dimm(self, ant, i):
        if i == 0:
            self.turnOFF(ant)
        else:
            self.turnON(ant)
            self.bus.write_byte_data(self.__address, ant[0], i)

    def rotate(self, antA, antB, i):
        self.dimm(antA, self.__ohm_max - i)
        self.dimm(antB, i)
        return round(((i / self.__ohm_max * 90) + antA[2]) % 360)

    def get_bearing(self):
        DArray = self.DArray.copy()
        if 0 in DArray:
            DArray[360] = DArray[0]

        # Find max S-Meter value
        MAX = max(DArray.values(), key=lambda x: x)

        # Create sub array of deg for MAX values.
        max_array = list()
        for i in DArray:
            if DArray[i] == MAX:
                max_array.append(i)

        # Calculate bearing
        bearing = round(numpy.rad2deg(circmean(numpy.deg2rad(numpy.array(max_array))))) % 360
        return (bearing, MAX)

    def run(self):
        logging.info("4ant started")
        DArray = dict()
        while True:
            DArray.clear()
            for ant in self.__Quad:
                for i in range(0, self.__div):
                    self._deg = self.rotate(ant[0], ant[1], i * round(self.__ohm_max / self.__div))
                    sleep(2)
                    self._s_meter = int((self.rigcat.get_level_i(Hamlib.RIG_LEVEL_STRENGTH) + 54) / 6)
                    DArray[self._deg] = self._s_meter
                self.rotate(ant[0], ant[1], self.__ohm_max)
            self.DArray = DArray.copy()


class kml:
    __File = None
    __record = list()

    def __init__(self, File):
        self.__File = File
        with open(File, 'w') as F:
            F.write('<kml></kml>\n')

    def writePos(self, QTH, Azimuth, Quality):
        timestamp = str(datetime.datetime.now())
        latitude = str(QTH[0])
        longitude = str(QTH[1])
        AzimuthLat = str(Azimuth[0])
        AzimuthLon = str(Azimuth[1])

        if Quality >= 9:
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
            color = '000000'

        self.__record.append((timestamp, latitude, longitude, AzimuthLat, AzimuthLon, color))

        with open(self.__File, 'w') as F:
            F.write('<Document>\n')
            for R in self.__record:
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


class MyButtonLED(ButtonLED):
    button_pressed = False

    def run(self):
        while True:
            if self.button.is_pressed:
                self.led.off()
                while self.button.is_pressed:
                    None
                self.button_pressed = True

    def reset(self):
        self.led.on()
        self.button_pressed = False


def main():
    signal(SIGINT, handler)
    config = configparser.ConfigParser()
    if len(sys.argv) > 1:
        config.read(sys.argv[1])
    else:
        config.read('azimuth.conf')

    if 'gps' in config.sections():
        my_GPS = GPS(config['gps']['dev'], int(config['gps']['baud']), -14, 0)
        TGPS = threading.Thread(target=my_GPS.run)
        TGPS.start()

    if 'httpd' in config.sections():
        httpd = httpServer(int(config['httpd']['port']), config['httpd']['root'])
        Thttpd = threading.Thread(target=httpd.run)
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

        screen = curses.initscr()
        while TDoppler.is_alive():
            if my_Doppler.dFlag:
                p = plot(my_GPS.QTH, my_Doppler.bearing[0], my_GPS.heading, int(config['common']['radius']))
                K.writePos(my_GPS.QTH, p.get_plot(), my_Doppler.bearing[1])
                msg = "--> " + str(my_GPS.QTH) + str(p.azimuth) + "\u00b0 Q" + str(my_Doppler.bearing[1]) + " <--"
                screen.addstr(1, 1, msg)
                screen.refresh()
                my_Doppler.dFlag = False
            curses.napms(1000)

    else:
        my_ButtonLED = MyButtonLED(18, 17)

        if 'rigcat' in config.sections():
            Hamlib.rig_set_debug(Hamlib.RIG_DEBUG_NONE)
            rigcat = Hamlib.Rig(Hamlib.__dict__[config['rigcat']['model']])
            rigcat.state.rigport.pathname = config['rigcat']['dev']
            rigcat.state.rigport.parm.serial.rate = int(config['rigcat']['baud'])
            rigcat.open()

        screen = curses.initscr()
        while True and config['common']['method'] == '1ant':
            my_ButtonLED.reset()
            s_meter = int((rigcat.get_level_i(Hamlib.RIG_LEVEL_STRENGTH) + 54) / 6)
            status = str(my_GPS.QTH) + " " + str(my_GPS.heading) + "\u00b0 " + str(my_GPS.speed) + " km/h S" + str(
                s_meter) + "   "
            screen.addstr(2, 1, status)
            screen.refresh()
            curses.napms(250)
            if my_ButtonLED.button_pressed:
                p = plot(my_GPS.QTH, 0, my_GPS.heading, int(config['common']['radius']))
                K.writePos(my_GPS.QTH, p.get_plot(), s_meter)
                msg = "--> " + str(my_GPS.QTH) + str(p.azimuth) + "\u00b0 S" + str(s_meter) + " <--   "
                screen.addstr(1, 1, msg)
                screen.refresh()
                curses.napms(250)

        if config['common']['method'] == '4ant':
            my_ButtonLED.reset()
            my_4ant = ant4(rigcat, int(config['4ant']['div']), int(config['4ant']['antA']), int(config['4ant']['antB']),
                           int(config['4ant']['antC']), int(config['4ant']['antD']))
            T4ant = threading.Thread(target=my_4ant.run)
            T4ant.start()
            screen = curses.initscr()
            while T4ant.is_alive():
                s_meter = int((rigcat.get_level_i(Hamlib.RIG_LEVEL_STRENGTH) + 54) / 6)
                status = "QTH: " + str(my_GPS.QTH) + " heading: " + str(my_GPS.heading) + "\u00b0 speed: " + str(
                    my_GPS.speed) + " km/h ant deg: " + str(my_4ant._deg) + "\u00b0 S" + str(s_meter) + "   "
                if len(my_4ant.DArray):
                    stats_array = str(my_4ant.DArray) + "          "
                    bearing_quality = my_4ant.get_bearing()
                    p = plot(my_GPS.QTH, bearing_quality[0], my_GPS.heading, int(config['common']['radius']))
                    msg = "--> " + str(p.azimuth) + "\u00b0 S" + str(bearing_quality[1]) + " <--   "
                    screen.addstr(1, 1, msg)
                    screen.addstr(3, 1, stats_array)
                    screen.addstr(4, 1, str(bearing_quality) + "   ")
                screen.addstr(2, 1, status)
                screen.refresh()
                curses.napms(250)

                if my_ButtonLED.button_pressed:
                    K.writePos(my_GPS.QTH, p.get_plot(), bearing_quality[1])
                    my_ButtonLED.reset()
                    sleep(3)


if __name__ == '__main__':
    main()
