import io
import serial
class ReadSerial:
    __ser = None
    __sio = None

    def __init__(self, port="/dev/ttyUSB0", baudrate=4800):
        self.__ser = serial.Serial(port, baudrate, timeout=0.5)
        self.__sio = io.TextIOWrapper(io.BufferedRWPair(self.__ser, self.__ser))

    def readSerial(self):
        self.__sio.flush()
        return self.__sio.readline()