import math
EARTH_RADIUS = 6371  # Km


def distance(self, QTH0, QTH1):
    """
    Approximate calculation distance (in meter)
    (expanding the trigonometric functions around the midpoint
    https://github.com/gojuno/geo-py/blob/master/geo/sphere.py
    """
    lat1 = math.radians(QTH0[0])
    lat2 = math.radians(QTH1[0])
    lon1 = math.radians(QTH0[1])
    lon2 = math.radians(QTH1[1])

    cos_lat = math.cos((lat1 + lat2) / 2.0)
    dx = (lat2 - lat1)
    dy = (cos_lat * (lon2 - lon1))

    return EARTH_RADIUS * math.sqrt(dx ** 2 + dy ** 2) * 1000



class plot:
    """
    Calculate plot from Point, bearing and distance.
    :Parameters:
     - QTH: The tuple representing the latitude/longitude for current
        point. Latitude and longitude must be in decimal degrees.
     - bearing: Degrees from QTH read from doppler.
     - distance: Distance (km) from QTH.
     - heading: My heading, to calculate real azimuth.
    :Return:
     Coordinates
    """

    latitude = None
    longitude = None
    azimuth = None
    distance = None

    def __init__(self, QTH, bearing, distance, heading = 0):
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
