#!/usr/bin/python

import os
import time
import numpy as np

import serial
import binascii

from decimal import *
from threading import Event
from modules.MotorDriver import *
from modules.ConnectionSenseHat import *
from modules.PCA9685 import PCA9685
from multiprocessing import Value, Array, Process
from gpiozero import RotaryEncoder, Button
from gpiozero.pins.pigpio import PiGPIOFactory
from witmotion import IMU


U_MAX = 100
U_MIN = 0
SPEED = 0
OFFSET = 0
FWD = 0
BCK = 1

# witmotion
#imu = IMU()

# ロータリーエンコーダのピン設定
PIN_ROTAR_A1 = 16
PIN_ROTAR_A2 = 20
PIN_ROTAR_D1 = 19
PIN_ROTAR_D2 = 26

# ロータリーエンコーダ初期化
rotation_a = FWD
rotation_d = FWD
def rotated_clockwise_a():
    global rotation_a
    rotation_a = BCK

def rotated_clockwise_d():
    global rotation_d
    rotation_d = BCK

def rotated_counter_clockwise_a():
    global rotation_a
    rotation_a = FWD

def rotated_counter_clockwise_d():
    global rotation_d
    rotation_d = FWD
 
getcontext().prec = 28
factory = PiGPIOFactory()
rotary_encoder_a = RotaryEncoder(
    PIN_ROTAR_A1, PIN_ROTAR_A2, wrap=True, max_steps=180, pin_factory=factory
)
rotary_encoder_a.steps = 0
rotary_encoder_a.when_rotated_clockwise = rotated_clockwise_a
rotary_encoder_a.when_rotated_counter_clockwise = rotated_counter_clockwise_a

rotary_encoder_d = RotaryEncoder(
    PIN_ROTAR_D1, PIN_ROTAR_D2, wrap=True, max_steps=180, pin_factory=factory
)
rotary_encoder_d.steps = 0
rotary_encoder_d.when_rotated_clockwise = rotated_clockwise_d
rotary_encoder_d.when_rotated_counter_clockwise = rotated_counter_clockwise_d

# calc duty-cycle
def calc_duty_cycle(delta, acc):
    duty_cycle = (acc + Decimal(str(192.066))) / Decimal(str(39.784))
    return round(duty_cycle)

before_count_a = 0
before_count_d = 0
rt_a_counter = 0
rt_d_counter = 0
prev_angle_y = 0

e = 0
e1 = 0
e2 = 0
ref = 0

direction = [
    'forward',
    'backward',
]

# PWM制御
pwm = PCA9685(0x40, debug=False)
pwm.setPWMFreq(50)
motor_driver = MotorDriver()
pwm.setPWMFreq(50000000.0)

# モータ初期化
direction_a = 0
direction_d = 0
motor_driver.MotorRun(pwm, 0, direction[direction_a], SPEED)
motor_driver.MotorRun(pwm, 1, direction[direction_d], SPEED)

#imu.set_gyro_automatic_calibration(True)

start = time.time()
getcontext().prec = 28


def main():
    """
    global ref
    global e, e1, e2
    global start
    global rotary_encoder_a
    global rotary_encoder_d
    global before_count_a
    global before_count_d
    global rt_a_counter
    global rt_d_counter
    global prev_angle_y
    global rotation_a
    global rotation_d
    global motor_driver
    global direction_a
    global direction_d
    ki = 5
    kd = 0.05
    kp = 1

    while True:
        rt_a_counter = rotary_encoder_a.steps
        rt_d_counter = rotary_encoder_d.steps
        # １ステップ前からのモータ回転角度
        delta_theta_a = Decimal(abs(abs(rt_a_counter) - abs(before_count_a)))
        delta_theta_d = Decimal(abs(abs(rt_d_counter) - abs(before_count_d)))
        if rotation_a == BCK:
            delta_theta_a = -delta_theta_a
        if rotation_d == BCK:
            delta_theta_d = -delta_theta_d
        
        

        print("rt_a_counter  : " + str("{:.7f}".format(rt_a_counter))
              + "\tdelta_theta_a  : " + str("{:.7f}".format(delta_theta_a)))
              

        # deleta theata計測用
        before_count_a = rt_a_counter
        before_count_d = rt_d_counter

        # モータへデューティ比を印加
        motor_driver.MotorRun(pwm, 0, direction[direction_a], 20)
        motor_driver.MotorRun(pwm, 1, direction[direction_d], 20)
    """
    ser = serial.Serial('/dev/ttyACM0', 115200, timeout = 0.5)
    print(str(ser))

    while True:
        #コマンドの結果を受信(4byte)
        #data = ser.read(28)
        #print(data.decode())
        #コマンドの結果を受信
        data = ser.readline()
        data = data.decode().replace('\n','').split(',')

        print(data)

    ser.close() # ポートのクローズ


    return 0

if __name__ == "__main__":
    main()