import math
EARTH_RADIUS = 6371  # Km


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
