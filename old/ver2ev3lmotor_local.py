#!/usr/bin/python

import time
import subprocess
import concurrent.futures
import numpy as np
from decimal import *
from threading import Event
from sense_hat import SenseHat
from modules.MotorDriver import *
from modules.ConnectionSenseHat import *
from modules.PCA9685 import PCA9685
from multiprocessing import Value, Array, Process
from gpiozero import RotaryEncoder, Button
from gpiozero.pins.pigpio import PiGPIOFactory
#from WitMotionSensorConnection import JY901S

U_MAX = 100
U_MIN = 0
SPEED = 0
OFFSET = 1
FWD = 1
BCK = 0

# ロータリーエンコーダのピン設定
PIN_ROTAR_A1 = 17
PIN_ROTAR_A2 = 27
PIN_ROTAR_D1 = 22
PIN_ROTAR_D2 = 23

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

# Wit Motion
'''
obj = None
device = None
# WitMotion角度センサ初期化
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
'''
'''
# sense hatを初期化
sense = SenseHat()
angle = []
# sense hat指示値読み替え用配列定義
for i in range(360):
    if i <= 180:
        angle.append(i)
    else:
        angle.append(i - 360)
'''
# sense hatの指示値読み取りをスレッド化
def read_sense_hat_angle(th_angle_y):
    while True:
        angle_y = subprocess.run("cat /opt/angle_y", encoding='utf-8', stdout=subprocess.PIPE, shell=True)
        angle_y = str(angle_y.stdout)
        if angle_y == '' or angle_y == None:
            continue
        else:
            th_angle_y.value = int(angle_y)

getcontext().prec = 28
def request_controler(th_angle_y):
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

    # 積算開始
    start = time.time()

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
    k1 = Decimal(str(363.41817119))
    k2 = Decimal(str(43.69756113))
    k3 = Decimal(str(7.46787171))
    k4 = Decimal(str(6.50978926))

    # モータ初期化
    direction_a = 0
    direction_d = 0
    motor_driver.MotorRun(pwm, 0, direction[direction_a], SPEED)
    motor_driver.MotorRun(pwm, 1, direction[direction_d], SPEED)

    getcontext().prec = 28
    while True:
        # 振子の回転角度を取得
        angle_y = Decimal(str(th_angle_y.value))
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
        '''
        if np.sign(delta_theta_a) == np.sign(delta_angle_y):
            motor_a_angle = delta_theta_a - delta_angle_y
        else:
            motor_a_angle = delta_angle_y - delta_theta_a
        if np.sign(delta_theta_d) == np.sign(delta_angle_y):
            motor_d_angle = delta_theta_d - delta_angle_y
        else:
            motor_d_angle = delta_angle_y - delta_theta_d
        '''
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

        # U(t)を導出
        ua = ((angle_y * k1) + (angular_velocity_y * k2) + (motor_a_angle * k3) + (velocity_a * k4))
        ud = ((angle_y * k1) + (angular_velocity_y * k2) + (motor_d_angle * k3) + (velocity_d * k4))

        # 回転方向を設定
        if ua < 0:
            direction_a = BCK
        else:
            direction_a = FWD

        if ud < 0:
            direction_d = BCK
        else:
            direction_d = FWD
        ua = abs(ua)
        ud = abs(ud)

        # 系に対してPWMデューティ比として与える制御入力をスケーリング
        if ua != U_MIN:
            ua = round(Decimal(0.02) * ua)
        if ud != U_MIN:
            ud = round(Decimal(0.02) * ud)

        # デューティ比の最大値を超過する場合は足切り
        if ua > U_MAX:
            ua = U_MAX
        if ud > U_MAX:
            ud = U_MAX
        
        print("angle_y  : " + str(angle_y)
                    + "\tangular_velocity_y  : " + str(angular_velocity_y) 
                    + "\t motor angle : " + str(motor_a_angle)
                    + "\t motor velocity : " + str(velocity_a)
                    + "\t motor delta_theta : " + str(delta_theta_a)
                    + "\t direction : " + direction[direction_a]
                    + "\t u(t)  : " + str(ua)
                    + "\t delta : " + str(delta_time))

        # deleta theata計測用
        before_count_a = rt_a_counter
        before_count_d = rt_d_counter
        prev_angle_y = angle_y

        # モータへデューティ比を印加
        motor_driver.MotorRun(pwm, 0, direction[direction_a], ua)
        motor_driver.MotorRun(pwm, 1, direction[direction_d], ua)
        # ハードウェア側の応答を見て待ち時間を設定
        #time.sleep(0.002)

if __name__ == "__main__":
    th_angle_y = Value('i', 0)
    p1 = Process(target=request_controler, args=[th_angle_y])
    p2 = Process(target=read_sense_hat_angle, args=[th_angle_y])
    p1.start()
    p2.start()