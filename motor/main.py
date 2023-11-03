#!/usr/bin/python

import time
from PCA9685 import PCA9685
from RPi import GPIO

from threading import Event
from gpiozero import RotaryEncoder, Button
from gpiozero.pins.pigpio import PiGPIOFactory


# ロータリーエンコーダのピン設定
PIN_ROTAR_A = 16
PIN_ROTAR_B = 20

GPIO.setmode(GPIO.BCM)
GPIO.setup(PIN_ROTAR_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(PIN_ROTAR_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)

counter = 0  #エンコーダー積算
times = 0    #エンコーダークリック回数
AstateOld = 1
BstateOld = 1


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

print("this is a motor driver test code")
Motor = MotorDriver()

#print("forward 2 s")
#Motor.MotorRun(0, 'forward', 20)
#Motor.MotorRun(1, 'forward', 20)
#time.sleep(12)

#print("backward 2 s")
#Motor.MotorRun(0, 'backward', 20)
#Motor.MotorRun(1, 'backward', 20)
#time.sleep(12)

#print("stop")
#Motor.MotorStop(0)
#Motor.MotorStop(1)
# 
# 25000000.0
pwm.setPWMFreq(25000000.0)



while True:
    Astate = GPIO.input(PIN_ROTAR_A)
    Bstate = GPIO.input(PIN_ROTAR_B)

    Motor.MotorRun(0, 'forward', 10)
    Motor.MotorRun(1, 'forward', 10)

    #print(Bstate)

    if Astate == 1 and AstateOld == 0:
        #print("AstateOld = " + str(AstateOld))
        #print("BstateOld = " + str(BstateOld))
        BstateOld = Bstate

        if AstateOld == 0 and BstateOld == 0:
            counter += 1
        elif AstateOld == 0 and BstateOld == 1:
            counter -= 1
        times += 1

        print("counter  = " + str(counter))
        print("times    = " + str(times))

    AstateOld = Astate   #今回のステートを前回のデータとして記録
    BstateOld = Bstate   #

print("stop")
Motor.MotorStop(0)
Motor.MotorStop(1)