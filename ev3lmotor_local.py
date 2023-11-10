#!/usr/bin/python

import os
import time
import math
import numpy as np
from decimal import *
from threading import Event
from modules.PCA9685 import PCA9685
from gpiozero import RotaryEncoder, Button
from gpiozero.pins.pigpio import PiGPIOFactory
from WitMotionSensorConnection import JY901S

U_MAX = 100
U_MIN = 0
SPEED = 0
OFFSET = -88

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
rt_a_counter = 0
rt_d_counter = 0
obj = None
device = None

# 角度センサ初期化
devicePath = "/dev/ttyUSB0"
if os.path.exists("/dev/ttyUSB1"):
    devicePath = "/dev/ttyUSB1"
obj = JY901S()
device = obj.getDevice()
device.serialConfig.portName = devicePath   # Set serial port
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
alpha = Decimal(str(100.0))
beta = Decimal(str(4.0))
gumma = Decimal(str(8.823137953779899))
delta = Decimal(str(-5.520524167632318))
k1 = Decimal(str(211.99102759))
k2 = Decimal(str(32.90440824))
k3 = Decimal(str(0.28867513))
k4 = Decimal(str(10.72842288))

direction = 0
Motor.MotorRun(0, Dir[direction], SPEED)
Motor.MotorRun(1, Dir[direction], SPEED)

getcontext().prec = 28
while True:
    #accel_x,angular_velocity_x,angle_x,accel_y,angular_velocity_y,angle_y, accel_z,angular_velocity_z,angle_z = obj.getDataAxisAll()
    accel_y,angular_velocity_y,angle_y = obj.getYDataAxisY()
    # Decimalにキャスト
    accel_y = Decimal(str(math.floor(accel_y)))
    angular_velocity_y = Decimal(str(math.floor(angular_velocity_y)))
    angle_y = Decimal(str(math.floor(angle_y)))
    rt_a_counter = rotorA.steps
    rt_d_counter = rotorD.steps
    motor_a_angle = angle_y - rt_a_counter
    motor_d_angle = angle_y - rt_d_counter
    delta_theta_a = Decimal(abs(abs(rt_a_counter) - abs(beforeCountA)))
    delta_theta_d = Decimal(abs(abs(rt_d_counter) - abs(beforeCountD)))
    delta_time = Decimal(time.time() - start)
    start = time.time()

    velocity_a = Decimal(delta_theta_a/delta_time)
    if delta_theta_a == 0:
        Motor.MotorRun(0, Dir[0], 0)
        velocity_a = Decimal(0.0)
        
    velocity_d = Decimal(delta_theta_d/delta_time)
    if delta_theta_d == 0:
        Motor.MotorRun(1, Dir[0], 0)
        velocity_d = Decimal(0.0)    

    diff_a_countor = abs(abs(beforeCountA) - abs(rt_a_counter))
    diff_d_countor = abs(abs(beforeCountD) - abs(rt_d_counter))

    if angle_y < 0:
        direction = 1
        velocity_a = velocity_a * (-1)
        velocity_d = velocity_d * (-1)
    else:
        direction = 0

    # U(t)を導出
    ua = round(abs((angle_y * k1) + (angular_velocity_y * k2) + (motor_a_angle * k3) + (velocity_a * k4)))
    ud = round(abs((angle_y * k1) + (angular_velocity_y * k2) + (motor_d_angle * k3) + (velocity_d * k4)))
    
    if ua != U_MIN:
        ua = 20 * math.log10(ua)
    if ud != U_MIN:
        ud = 20 * math.log10(ud)

    if ua > U_MAX:
        ua = U_MAX
    if ud > U_MAX:
        ud = U_MAX

    if delta_theta_a == 0:
        ua = 0
    if delta_theta_d == 0:
        ud = 0   

    print("accel_y  : " + str(accel_y) 
                + "\t angular_velocity_y  : " + str(angular_velocity_y) 
                + "\t angle_y  : " + str(angle_y)
                + "\t motor velocity : " + str(velocity_a)
                + "\t motor angle : " + str(angle_y - rt_a_counter)
                + "\t motor delta_theta : " + str(delta_theta_a)
                + "\t u(t)  : " + str(ua)
                + "\t delta : " + str(delta_time))

    # deleta theata計測用
    beforeCountA = rt_a_counter
    beforeCountD = rt_d_counter

    # モータへ電圧を印加
    Motor.MotorRun(0, Dir[direction], ua)
    Motor.MotorRun(1, Dir[direction], ua)
    #time.sleep(0.005)
    