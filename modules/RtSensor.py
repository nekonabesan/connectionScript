#!/usr/bin/python

import serial

class RtSensor():
  port = None
  rate = None
  time_out_sec = None
  ser = None

  def __init__(self, port = '/dev/ttyACM0', rate = 115200, time_out_sec = 0.5):
    self.port = port
    self.rate = rate
    self.time_out_sec = time_out_sec
    self.ser = serial.Serial(self.port, self.rate, timeout = self.time_out_sec)

  def getSensorValues(self):
    data = self.ser.readline()
    return data.decode().replace('\n','').split(',')

