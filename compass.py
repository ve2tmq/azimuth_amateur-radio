'''
    Find Heading by using HMC5883L interface with Raspberry Pi using Python
	http://www.electronicwings.com
'''
import smbus		# import SMBus module of I2C
import math

# some MPU6050 Registers and their Address
Register_A     = 0              # Address of Configuration register A
Register_B     = 0x01           # Address of configuration register B
Register_mode  = 0x02           # Address of mode register

X_axis_H    = 0x03              # Address of X-axis MSB data register
Z_axis_H    = 0x05              # Address of Z-axis MSB data register
Y_axis_H    = 0x07              # Address of Y-axis MSB data register


class Compass:
    def __init__(self, declination: float = 0.0):
        self.bus = smbus.SMBus(1)  # or bus = smbus.SMBus(0) for older version boards
        self._device_address = 0x1e  # HMC5883L magnetometer device address
        self._declination = declination

        # Write to Configuration Register A
        self.bus.write_byte_data(self._device_address, Register_A, 0x70)

        # Write to Configuration Register B for gain
        self.bus.write_byte_data(self._device_address, Register_B, 0xa0)

        # Write to mode Register for selecting mode
        self.bus.write_byte_data(self._device_address, Register_mode, 0)


    def read_raw_data(self, addr):
        # Read raw 16-bit value
        high = self.bus.read_byte_data(self._device_address, addr)
        low = self.bus.read_byte_data(self._device_address, addr+1)

        # concatenate higher and lower value
        value = ((high << 8) | low)

        # to get signed value from module
        if value > 32768:
            value = value - 65536
        return value

    def get_heading(self):
        # Read Accelerometer raw value
        x = self.read_raw_data(X_axis_H)
        y = self.read_raw_data(Y_axis_H)

        # math.atan2 return values from -180° to + 180°, so we calculate (heading + 360) % 360.
        return int(((math.degrees(math.atan2(x, y)) + 360) % 360) - self._declination)

        """
        heading = math.atan2(y, x) + self._declination
        
        # Due to declination check for >360 degree
        if heading > 2*math.pi:
            heading = heading - 2*math.pi

        # check for sign
        if heading < 0:
            heading = heading + 2*math.pi

        # convert into angle
        return int(heading * 180/math.pi)
        """

