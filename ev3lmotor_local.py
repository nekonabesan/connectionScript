#!/usr/bin/python

import time
import math
import numpy as np
from decimal import *
from threading import Event
from modules.PCA9685 import PCA9685
from gpiozero import RotaryEncoder, Button
from gpiozero.pins.pigpio import PiGPIOFactory
from WitMotionSensorConnection import JY901S

SPEED = 0

# ロータリーエンコーダのピン設定
PIN_ROTAR_A1 = 16
PIN_ROTAR_A2 = 20
PIN_ROTAR_D1 = 23
PIN_ROTAR_D2 = 24

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
                pwm.setLevel(self.AIN1, 0)
                pwm.setLevel(self.AIN2, 1)
            else:
                pwm.setLevel(self.AIN1, 1)
                pwm.setLevel(self.AIN2, 0)
        else:
            pwm.setDutycycle(self.PWMB, speed)
            if(index == Dir[0]):
                pwm.setLevel(self.BIN1, 0)
                pwm.setLevel(self.BIN2, 1)
            else:
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

# ロータリーエンコーダ初期化
factory = PiGPIOFactory()
rotorA = RotaryEncoder(
        PIN_ROTAR_A1, PIN_ROTAR_A2, wrap=True, max_steps=180, pin_factory=factory
)
rotorA.steps = 0

rotorD = RotaryEncoder(
        PIN_ROTAR_D1, PIN_ROTAR_D2, wrap=True, max_steps=180, pin_factory=factory
)
rotorD.steps = 0

beforeCountA = 0
beforeCountD = 0
rt_a_counter = 0  #エンコーダー積算
rt_d_counter = 0  #エンコーダー積算

# 角度センサ初期化
obj = JY901S()
device = obj.getDevice()
device.serialConfig.portName = "/dev/ttyUSB0"   # Set serial port
device.serialConfig.baud = 9600                     # Set baud rate
device.openDevice()                                 # Open serial port
# Read configuration information
obj.readConfig(device)
# Data update event
device.dataProcessor.onVarChanged.append(obj.onUpdate)

# 積算開始
start = time.time()

# 制御パラメータ設定
am = Decimal(str(17.015716113746162))
bm = Decimal(str(10.011633801736519))
alpha = Decimal(str(899.9999999999998))
beta = Decimal(str(6.0))
gumma = Decimal(str(-51.98628434185563))
delta = Decimal(str(30.587466202679973))
k1 = Decimal(str(94.88572135))
k2 = Decimal(str(10.48930069))
k3 = Decimal(str(0.31622777))
k4 = Decimal(str(3.5038014))

# Start recording data
obj.startRecord()

direction = 0
Motor.MotorRun(0, Dir[direction], SPEED)
Motor.MotorRun(1, Dir[direction], SPEED)

getcontext().prec = 28
while True:
    #accel_x,angular_velocity_x,angle_x,accel_y,angular_velocity_y,angle_y, accel_z,angular_velocity_z,angle_z = obj.getDataAxisAll()
    accel_y,angular_velocity_y,angle_y = obj.getYDataAxisY()
    # Decimalにキャスト
    accel_y = Decimal(str(accel_y))
    angular_velocity_y = Decimal(str(angular_velocity_y))
    angle_y = Decimal(str(angle_y))
    rt_a_counter = rotorA.steps
    rt_d_counter = rotorD.steps

    diff_a_countor = abs(abs(beforeCountA) - abs(rt_a_counter))
    diff_d_countor = abs(abs(beforeCountD) - abs(rt_d_counter))
    #print(Decimal(str(time.time() - start)))

    # U(t)を導出
    u = round(abs(- (angle_y * k1) + (angular_velocity_y * k2) - (0 * k3) - (accel_y * k4)))
    
    if u < 0:
        direction = 1
    else:
        direction = 0

    
    
    
    

    print("accel_y  : " + str(accel_y) 
                + "\t angular_velocity_y  : " + str(angular_velocity_y) 
                + "\t angle_y  : " + str(angle_y)
                + "\t u(t)  : " + str(u)
                + "\t delta : " + str(time.time() - start))

    # deleta theata計測用
    beforeCountA = rt_a_counter
    beforeCountD = rt_d_counter
    #pwm_value = 0
    # モータへ電圧を印加
    Motor.MotorRun(0, Dir[direction], u)
    Motor.MotorRun(1, Dir[direction], u)
    start = time.time()
    