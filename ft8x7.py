#!/usr/bin/python3
'''
Created on 06 May 2016
Updated on 31 Dec 2016

@author: Dmitry Melnichansky 4X5DM ex 4Z7DTF
         https://github.com/4x5dm
         http://www.qrz.com/db/4X5DM

@note: FT8x7 class which communicates with FT-8x7 using serial library,
       queries and sets transceiver's frequency and state and generates
       transceiver state strings for printing.

'''

import serial
import sys

class FT8x7(object):
    
    # Constants
    # Serial port settings
    SERIAL_SPEED = 9600
    SERIAL_STOPBITS = serial.STOPBITS_TWO
    SERIAL_TIMEOUT = 1.0
    # Transceiver modes and commands
    MODES = ["LSB", "USB", "CW", "CWR", "AM", None, "WFM", None, "FM", None, "DIG", None, "PKT"]
    CMD_READ_FREQ = [0x00, 0x00, 0x00, 0x00, 0x03]
    CMD_READ_RX_STATUS = [0x00, 0x00, 0x00, 0x00, 0xE7]
    
    def __init__(self, serial_port, serial_speed=SERIAL_SPEED, serial_stopbits=SERIAL_STOPBITS):
        self._serial = serial.Serial(serial_port, serial_speed, stopbits=serial_stopbits, timeout=FT8x7.SERIAL_TIMEOUT)
        self._frequency = ""
        self._mode = ""
        self._squelch = True
        self._s_meter = ""
        
    def read_frequency(self):
        '''Queries transceiver RX frequency and mode.
        The response is 5 bytes: first four store frequency and
        the fifth stores mode (AM, FM, SSB etc.)
        '''
        cmd = FT8x7.CMD_READ_FREQ
        self._serial.write(cmd)
        resp = self._serial.read(5)
        resp_bytes = (ord(chr(resp[0])), ord(chr(resp[1])), ord(chr(resp[2])), ord(chr(resp[3])))
        self._frequency = "%02x%02x%02x%02x" % resp_bytes
        self._mode = FT8x7.MODES[ord(chr(resp[4]))]
        
    def read_rx_status(self):
        '''Queries transceiver RX status.
        The response is 1 byte:
        bit 7: Squelch status: 0 - off (signal present), 1 - on (no signal)
        bit 6: CTCSS/DCS Code: 0 - code is matched, 1 - code is unmatched
        bit 5: Discriminator centering: 0 - discriminator centered, 1 - uncentered
        bit 4: Dummy data
        bit 3-0: S Meter data
        '''
        cmd = FT8x7.CMD_READ_RX_STATUS
        self._serial.write(cmd)
        resp = self._serial.read(1)
        resp_byte = ord(chr(resp[0]))
        self._squelch = True if (resp_byte & 0B10000000) else False
        self._s_meter = resp_byte & 0x0F
        
    def get_s_meter_string(self, s_meter):
        '''Generates S-Meter string for printing. The string includes
        S value with decibels over 9 is printed and a simple 15 symbols scale.
        Examples:
        S0:      S0+00 ...............
        S3:      S3+00 |||............
        S9:      S9+00 |||||||||......
        S9+20dB: S9+00 |||||||||||....
        '''
        res = "S9" if s_meter >= 9 else "S" + str(s_meter)
        above_nine = s_meter - 9
        if above_nine > 0:
            res += "+%s" % (10 * above_nine)
        else:
            res += "+00"
        res += " "
        res += "|" * s_meter
        res += "." * (15 - s_meter)
        return res

    def get_trx_state_string(self):
        '''Returns transceiver state data for printing.
        '''
        s_meter_str = self.get_s_meter_string(self._s_meter)
        sql_str = 'SQL: ON' if self._squelch else 'SQL: OFF'
        res = "%sHz %s %s\r\n%s" % (self._frequency, self._mode, sql_str, s_meter_str)
        return res
    
    def __str__(self):
        '''Overrides __str__() method for using FT8x7 class with print command.
        '''
        return self.get_trx_state_string()
    
if __name__ == '__main__':
    ft8x7 = FT8x7(sys.argv[1])
    ft8x7.read_frequency()
    ft8x7.read_rx_status()
    print(ft8x7)

