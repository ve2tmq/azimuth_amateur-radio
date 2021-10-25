#!/usr/bin/env python3
from gpiozero import LED, Button
import threading
import sys


class ButtonLED:
    def __init__(self, led, button):
        self.led = LED(led)
        self.button = Button(button)
        self.TButtonLED = threading.Thread(target = self.run)
        self.TButtonLED.start()

    def run(self):
        while True:
            if self.button.is_pressed:
                self.led.on()
            else:
                self.led.off()

    def reset(self):
        self.led.off()

def clearLED(argv):
    led = LED(argv[1])
    led.off()

if __name__ == '__main__':
    clearLED(sys.argv)
