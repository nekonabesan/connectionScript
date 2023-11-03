#!/usr/bin/python

import time
import math
import requests
import simplejson as json

from modules.PCA9685 import PCA9685
from RPi import GPIO

from threading import Event
from gpiozero import RotaryEncoder, Button
from gpiozero.pins.pigpio import PiGPIOFactory


URL = 'http://192.168.0.56:8000'
PATH = '/controller/observer/send/'

session_id = 0
counter = 0
mode = -1
headers = {"Content-Type" : "application/json"}

# ロータリーエンコーダのピン設定
PIN_ROTAR_A1 = 16    # 
PIN_ROTAR_A2 = 20

#GPIO.setmode(GPIO.BCM)
#GPIO.setup(PIN_ROTAR_A1, GPIO.IN, pull_up_down=GPIO.PUD_UP)
#GPIO.setup(PIN_ROTAR_A2, GPIO.IN, pull_up_down=GPIO.PUD_UP)


Dir = [
    'forward',
    'backward',
]
pwm = PCA9685(0x40, debug=False)
pwm.setPWMFreq(50)

class MotorDriver():
    def __init__(self):
        self.PWMA = 0
        self.AIN1 = 1
        self.AIN2 = 2
        self.PWMB = 5
        self.BIN1 = 3
        self.BIN2 = 4

    def MotorRun(self, motor, index, speed):
        if speed > 100:
            return
        if(motor == 0):
            pwm.setDutycycle(self.PWMA, speed)
            if(index == Dir[0]):
                #print ("1")
                pwm.setLevel(self.AIN1, 0)
                pwm.setLevel(self.AIN2, 1)
            else:
                #print ("2")
                pwm.setLevel(self.AIN1, 1)
                pwm.setLevel(self.AIN2, 0)
        else:
            pwm.setDutycycle(self.PWMB, speed)
            if(index == Dir[0]):
                #print ("3")
                pwm.setLevel(self.BIN1, 0)
                pwm.setLevel(self.BIN2, 1)
            else:
                #print ("4")
                pwm.setLevel(self.BIN1, 1)
                pwm.setLevel(self.BIN2, 0)

    def MotorStop(self, motor):
        if (motor == 0):
            pwm.setDutycycle(self.PWMA, 0)
        else:
            pwm.setDutycycle(self.PWMB, 0)


#print("this is a motor driver test code")
Motor = MotorDriver()
# 25000000.0
pwm.setPWMFreq(25000000.0)
start = time.time()
beforeCount = 0
rt_counter = 0  #エンコーダー積算
times = 0    #エンコーダークリック回数
AstateOld = 1
BstateOld = 1

# -------------------------------------------------------------------------------- #
# PIN Factory エラー発生時の対応
# (参考)https://www.nicovideo.jp/watch/sm39096166
# sudo systemctl enable pigpiod
# sudo systemctl start pigpiod
# -------------------------------------------------------------------------------- #
factory = PiGPIOFactory()
rotor = RotaryEncoder(
        PIN_ROTAR_A1, PIN_ROTAR_A2, wrap=True, max_steps=180, pin_factory=factory
)
rotor.steps = 0

Motor.MotorRun(0, 'forward', 10)
Motor.MotorRun(1, 'forward', 10)
while True:

    rt_counter = rotor.steps
    print("counter  : " + str(rt_counter) + "\t delta : " + str(time.time() - start))

    beforeCount = rt_counter
    
    start = time.time()
    time.sleep(0.05)