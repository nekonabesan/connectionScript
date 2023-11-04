#!/usr/bin/python

import time
from threading import Event
from WitMotionSensor import *
from modules.PCA9685 import PCA9685
from gpiozero import RotaryEncoder, Button
from gpiozero.pins.pigpio import PiGPIOFactory

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
# 積算開始
start = time.time()
# 角度センサ初期化
obj = BWT901("/dev/ttyUSB1")
angle_x = 0
angle_y = 0
angle_z = 0

# 
am = 17.015716113746162
bm = 10.011633801736519
alpha = 899.9999999999998
beta = 6.0
gumma = -51.98628434185563
delta = 30.587466202679973
K1 = -108.4851416
K2 = -5.1033899
K3 = 1
K4 = 3.8032562

Motor.MotorRun(0, 'forward', SPEED)
Motor.MotorRun(1, 'forward', SPEED)
while True:
    #accel_x,angular_velocity_x,angle_x,accel_y,angular_velocity_y,angle_y, accel_z,angular_velocity_z,angle_z = obj.getAngle()
    accel_y,angular_velocity_y,angle_y  = obj.getAngleY()
    '''
    accel_y=0
    angular_velocity_y=0
    angle_y=0
    '''
    rt_a_counter = rotorA.steps
    rt_d_counter = rotorD.steps
    print("accel_y  : " + str(accel_y) 
                + "\t angular_velocity_y  : " + str(angular_velocity_y) 
                + "\t angle_y  : " + str(angle_y) 
                + "\t delta : " + str(time.time() - start))
    # deleta theata計測用
    #beforeACount = rt_a_counter
    #beforeDCount = rt_d_counter
    pwm_value = 0
    
    Motor.MotorRun(0, 'forward', pwm_value)
    Motor.MotorRun(1, 'forward', pwm_value)
    start = time.time()
    