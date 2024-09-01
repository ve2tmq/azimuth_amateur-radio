'''
    Find Heading by using HMC5883L interface with Raspberry Pi using Python
	http://www.electronicwings.com
'''
import smbus		# import SMBus module of I2C
import numpy

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
        self.declination = declination

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
        z = self.read_raw_data(Z_axis_H)

        print((x, y, z))
        heading = numpy.arctan2(y, x)
        heading += self.declination

        if heading < 0:
            heading += 2 * numpy.pi

        if heading > 2 * numpy.pi:
            heading -= 2 * numpy.pi

        return int(numpy.rad2deg(heading))
