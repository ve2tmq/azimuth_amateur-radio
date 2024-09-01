import logging
import math
from read_serial import ReadSerial

EARTH_RADIUS = 6371  # Km


class GPS(ReadSerial):
    heading = 0
    speed = 0
    QTH = (0, 0)

    def __init__(self, GPSDev, GPSBaud, degrees=0, minutes=0):
        super().__init__(GPSDev, GPSBaud)

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

        return EARTH_RADIUS * math.sqrt(dx ** 2 + dy ** 2) / 1000

    """
    Calculates the heading within two points.
    https://www.movable-type.co.uk/scripts/latlong.html
    https://gist.github.com/jeromer/2005586

    :Parameters:
      - `QTH: The tuple representing the latitude/longitude for the
        first point. Latitude and longitude must be in decimal degrees
    :Returns:
      The heading in degrees
    :Returns Type:
      int
    """

    def get_heading(self, QTH0, QTH1):
        if self.speed >= 1:
            lat1 = math.radians(QTH0[0])
            lat2 = math.radians(QTH1[0])

            diffLong = math.radians(QTH1[1] - QTH0[1])

            x = math.sin(diffLong) * math.cos(lat2)
            y = math.cos(lat1) * math.sin(lat2) - (math.sin(lat1) * math.cos(lat2) * math.cos(diffLong))

            # math.atan2 return values from -180° to + 180°, so we calculate (heading + 360) % 360.
            heading = int((math.degrees(math.atan2(x, y)) + 360) % 360)
            return heading

        return self.heading

    """ Get heaging with GPS value.
    """

    def get_heading(self, GPVTG):
        if len(GPVTG[1]):
            return float(GPVTG[1])

        return self.heading

    def get_speed(self, GPVTG):
        return float(GPVTG[7])

    """
    Thread
    Read GPS from serial port
    coordinate: tuple (lat, lon)
    """

    def readGPS(self, GPGGA):
        time = GPGGA[1]

        latDMm = float(GPGGA[2]) / 100
        DD = int(latDMm)
        Mm = ((latDMm - DD) * 100) / 60
        lat = float(format(DD + Mm, '.6f'))
        if GPGGA[3] == 'S':
            lat = -lat

        lonDMm = float(GPGGA[4]) / 100
        DD = int(lonDMm)
        Mm = ((lonDMm - DD) * 100) / 60
        lon = float(format(DD + Mm, '.6f'))
        if GPGGA[5] == 'W':
            lon = -lon

        return (lat, lon)

    def run(self):
        QTH = None
        logging.info("GPS started")

        while True:
            try:
                s = self.readSerial()
                data = s.split(",")

                if data[0] == "$GPVTG":
                    self.speed = self.get_speed(data)
                    self.heading = round(self.get_heading(data))

                elif data[0] == "$GPGGA":
                    self.QTH = self.readGPS(data)

            except:
                continue
