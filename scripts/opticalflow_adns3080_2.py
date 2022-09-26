# -*- coding: utf-8 -*-
# https://github.com/carlosgj/ADNS3080/blob/master/ADNS3080.py - 코드
# https://forums.raspberrypi.com/viewtopic.php?t=154317 - GPIO 핀에서 SPI 통신을 활성화 해야함.
# https://jetsonhacks.com/2020/05/04/spi-on-jetson-using-jetson-io/ - GPIO Pin 활성화 하는 방법
# https://forums.developer.nvidia.com/t/jetson-nano-spi-and-pwm-not-working/188866 - sudo modprobe spidev

import spidev 
# import RPi.GPIO as GPIO
import Jetson.GPIO as GPIO
import time 
import adns3080_srom as srom
import struct

class ADNS3080():
    def readReg(self, address):
        result = self.spi.xfer2([address, 0], 50000)
        return result[1]
    
    def writeReg(self, address, data):
        address = address | 0b10000000
        self.spi.xfer2([address, data], 50000)

    def __init__(self, rstpin=22, bus=0, device=0):
        self.rstpin = rstpin
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(rstpin, GPIO.OUT)
        GPIO.output(rstpin, GPIO.LOW)
        self.spi = spidev.SpiDev()
        #self.spi.max_speed_hz = 1953000
        self.spi.open(bus, device)
        self.spi.mode = 3
        
    def resetDevice(self):
        GPIO.output(self.rstpin, GPIO.HIGH)
        time.sleep(0.01)
        GPIO.output(self.rstpin, GPIO.LOW)
        time.sleep(0.01)

    def downloadSROM(self, SROM):
        self.resetDevice()
        time.sleep(0.1)
        self.writeReg(0x20, 0x44)
        time.sleep(0.1)
        self.writeReg(0x23, 0x07)
        time.sleep(0.1)
        self.writeReg(0x24, 0x88)
        time.sleep(0.1)
        self.writeReg(0x14, 0x18)
        time.sleep(0.1)
        self.spi.xfer([0x60]+SROM, 50000)
        time.sleep(1)
        print("SROM ID:", self.readReg(0x1f))
        # print "SROM ID:", self.readReg(0x1f)

    def wait(self):
        #catches an inevitable "Bad file descriptor"
        try:
            time.sleep(2)
        except:
            pass

    def initializeSensor(self):
        #Toggle reset
        self.resetDevice()
        time.sleep(0.1)
        id = self.readReg(0)
        id = self.readReg(0)
        if id != 23:
            print("ERROR: Expected device ID 0x17, got 0x%x."%id)
            return
        print("SROM ID:", self.readReg(0x1f))

    def runSelfTest(self):
        self.resetDevice()
        currentRegVal = self.readReg(0x0a)
        self.writeReg(0x0a, currentRegVal | 0b00100000)
        time.sleep(1)
        result = (self.readReg(0x0d)<<8) | self.readReg(0x0c)
        if result == 0x1bbf:
            return True
        else:
            print("ERROR: Self-test failed")
            return False

    def frameCapture(self):
        self.resetDevice()
        self.writeReg(0x13, 0x83)
        time.sleep(0.1)
        outputData = self.spi.xfer([0x40]+[0]*900, 1000000, 100)
        outputData = outputData[1:]
        for i, byte in enumerate(outputData):
            if (byte & 0b10000000) == 0:
                print("WARNING: MSB of byte %d not set."%i)
            if (byte & 0b01000000) == 0:
                if i == 0:
                    print("WARNING: Bit 6 of first byte of frame data not set.")
            else:
                if i > 0:
                    print("WARNING: Bit 6 of byte %d of frame data is set."%i)
            outputData[i] = byte & 0b00111111
        return outputData
        
    def getMotion(self):
        #TODO: may want to redo this with motion_burst        
        motionByte = self.readReg(2)
        x = self.readReg(3)
        y = self.readReg(4)
        x = struct.unpack('b', struct.pack('B', x))[0]
        y = struct.unpack('b', struct.pack('B', y))[0]
        resolution = ((motionByte & 1) != 0)
        overflow = ((motionByte & 0b10000) != 0)
        motion = ((motionByte & 0b10000000) != 0)
        return (motion, overflow, resolution, x, y)

if __name__ == "__main__":
    this = ADNS3080()
    this.wait()
    this.initializeSensor()
    #this.downloadSROM(srom.srom)
    #print this.frameCapture()
    #this.runSelfTest()
    while(1):
        # print(this.readReg(0), this.readReg(1), this.readReg(5), this.getMotion())
        # print(" readReg(0): ", this.readReg(0), "\\1 : " ,this.readReg(1), "\\5 : ",this.readReg(5), "\\get motion : ",this.getMotion())
        # print(len(this.frameCapture()))
        print(this.frameCapture())
        time.sleep(0.1)