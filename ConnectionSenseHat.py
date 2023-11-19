#!/usr/bin/python
import threading
from sense_hat import SenseHat

class ConnectionSenseHat():
    sense = None
    th = None
    accX = 0.0
    accY = 0.0
    accZ = 0.0
    gyroX = 0.0
    gyroY = 0.0
    gyroZ = 0.0
    angleX = 0.0
    angleY = 0.0
    angleZ = 0.0

    def __init__(self):
        self.sense = SenseHat()
        self.start()

    def read(self, threadName):
        orientation = None
        while True:
            orientation = self.sense.get_orientation_degrees()
            self.angleY = orientation['pitch']


    def start(self):
        try:
            self.isOpen = True
            self.th = threading.Thread(target=self.read, args=("Data-Received-Thread"))
            self.th.start()
        except Exception:
            print("")

    def getAngle(self):
        return self.angleX,self.angleY,self.angleZ

    
