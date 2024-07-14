#!/usr/bin/python

import os
import time
import numpy as np
from decimal import *
from threading import Event
from sense_hat import SenseHat
from MotorDriver import *
from ConnectionSenseHat import *
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
imu = IMU()

# ロータリーエンコーダのピン設定
PIN_ROTAR_A1 = 19
PIN_ROTAR_A2 = 26
PIN_ROTAR_D1 = 16
PIN_ROTAR_D2 = 20

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

# sense hatを初期化
sense = SenseHat()
angle = []
# sense hat指示値読み替え用配列定義
for i in range(360):
    if i <= 180:
        angle.append(i)
    else:
        angle.append(i - 360)

# calc duty-cycle
def calc_duty_cycle(delta, acc):
    duty_cycle = (acc + Decimal(str(192.066))) / Decimal(str(39.784))
    return round(duty_cycle)
    
 
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

before_count_a = 0
before_count_d = 0
rt_a_counter = 0
rt_d_counter = 0
prev_angle_y = 0

direction = [
    'forward',
    'backward',
]

# PWM制御
pwm = PCA9685(0x40, debug=False)
pwm.setPWMFreq(50)
motor_driver = MotorDriver()
pwm.setPWMFreq(50000000.0)

# 制御パラメータ設定
k1 = Decimal(str(1659.69504036))
k2 = Decimal(str(679.07016698))
k3 = Decimal(str(60.26645638))

# モータ初期化
direction_a = 0
direction_d = 0
motor_driver.MotorRun(pwm, 0, direction[direction_a], SPEED)
motor_driver.MotorRun(pwm, 1, direction[direction_d], SPEED)

imu.set_gyro_automatic_calibration(True)

def worker():
    global imu
    global k1,k2,k3
    global sense
    global angle
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
    
    # 振子の回転角度を取得(witmotion)
    angle_x,angle_y,angle_z = imu.get_angle()
    angle_y = Decimal(str(angle_y))
    omega_x,omega_y,omega_z = imu.get_angular_velocity()
    omega_y = Decimal(str(omega_y))
    # 振子の回転角度を取得(sense-hat)
    #orientation = sense.get_orientation_degrees()
    #angle_y = Decimal(str(angle[int(round(orientation['pitch'])) - OFFSET]))

    delta_angle_y = Decimal(angle_y - prev_angle_y)
    rt_a_counter = rotary_encoder_a.steps
    rt_d_counter = rotary_encoder_d.steps
    # １ステップ前からのモータ回転角度
    delta_theta_a = Decimal(abs(abs(rt_a_counter) - abs(before_count_a)))
    delta_theta_d = Decimal(abs(abs(rt_d_counter) - abs(before_count_d)))
    if rotation_a == BCK:
        delta_theta_a = -delta_theta_a
    if rotation_d == BCK:
        delta_theta_d = -delta_theta_d
    # motorの振子に対する角度
    motor_a_angle = delta_theta_a - delta_angle_y
    motor_d_angle = delta_theta_d - delta_angle_y

    delta_time = Decimal(time.time() - start)
    start = time.time()
    # 振子の角速度
    if delta_angle_y == 0:
        angular_velocity_y = Decimal(0)
    else:
        angular_velocity_y = Decimal(delta_angle_y/delta_time)
    # モータの角速度
    velocity_a = Decimal(delta_theta_a/delta_time)
    velocity_d = Decimal(delta_theta_d/delta_time)

    # 加速度を制御入力とする
    acc_a = ((angle_y * k1) + (omega_y * k2) + (velocity_a * k3))
    acc_d = ((angle_y * k1) + (omega_y * k2) + (velocity_d * k3))

    # 回転方向を設定
    if acc_a < 0:
        direction_a = BCK
    else:
        direction_a = FWD

    if acc_d < 0:
        direction_d = BCK
    else:
        direction_d = FWD

    # 制御入力を絶対値に変換
    acc_a = abs(acc_a)
    acc_d = abs(acc_d)

    # 加速度をデューティー比に変換
    pwm_a = calc_duty_cycle(delta_time, acc_a)
    pwm_d = calc_duty_cycle(delta_time, acc_d)

    # デューティ比の最大値を超過する場合は足切り
    if pwm_a > U_MAX:
        pwm_a = U_MAX
    if pwm_d > U_MAX:
        pwm_d = U_MAX

    if pwm_a < U_MIN:
        pwm_a = U_MIN
    if pwm_d < U_MIN:
        pwm_d = U_MIN
        
    print("angle_y  : " + str("{:.3f}".format(angle_y))
                    + "\tangular_velocity_y  : " + str(omega_y) 
                    + "\t motor angle : " + str(motor_a_angle)
                    + "\t motor velocity : " + str(velocity_a)
                    + "\t acc : " + str("{:.3f}".format(acc_a))
                    + "\t pwm : " + str(pwm_a)
                    + "\t delta : " + str("{:.4f}".format(delta_time)))
        
    # deleta theata計測用
    before_count_a = rt_a_counter
    before_count_d = rt_d_counter
    prev_angle_y = angle_y

    # モータへデューティ比を印加
    motor_driver.MotorRun(pwm, 0, direction[direction_a], pwm_a)
    motor_driver.MotorRun(pwm, 1, direction[direction_d], pwm_d)
    # ハードウェア側の応答を見て待ち時間を設定
    #time.sleep(0.002)

start = time.time()
getcontext().prec = 28
def mainloop(interval, worker):
    global start
    # 積算開始
    now = time.time()
    while True:
        t = threading.Thread(target=worker)
        t.setDaemon(True)
        t.start()
        t.join()
        wait_time = interval - ((time.time() - now) % interval)
        time.sleep(wait_time)

if __name__ == "__main__":
    mainloop(0.01, worker)