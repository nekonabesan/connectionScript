#!/usr/bin/python

import time
import numpy as np
from decimal import *
from threading import Event
from sense_hat import SenseHat
#from MotorDriver import *
from ConnectionSenseHat import *
from modules.PCA9685 import PCA9685
from gpiozero import RotaryEncoder, Button
from gpiozero.pins.pigpio import PiGPIOFactory
#from WitMotionSensorConnection import JY901S

U_MAX = 100
U_MIN = 0
SPEED = 0
OFFSET = 35
FWD = 0
BCK = 1

# ロータリーエンコーダのピン設定
PIN_ROTAR_A1 = 17
PIN_ROTAR_A2 = 27
PIN_ROTAR_D1 = 22
PIN_ROTAR_D2 = 23

direction = [
    'forward',
    'backward',
]

# PWM制御
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
            if(index == direction[0]):
                pwm.setLevel(self.AIN1, 0)
                pwm.setLevel(self.AIN2, 1)
            else:
                pwm.setLevel(self.AIN1, 1)
                pwm.setLevel(self.AIN2, 0)
        else:
            pwm.setDutycycle(self.PWMB, speed)
            if(index == direction[0]):
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



motor_driver = MotorDriver()
pwm.setPWMFreq(50000000.0)

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
prev_angle_y = 0

# sense hatを初期化
sense = SenseHat()
angle = []
# sense hat指示値読み替え用配列定義
for i in range(360):
    if i <= 180:
        angle.append(i)
    else:
        angle.append(i - 360)

# sense hatの指示値読み取りをスレッド化
th_angle_y = 0
def read_sense_hat_angle():
    global th_angle_y
    while True:
        orientation = sense.get_orientation_degrees()
        th_angle_y = angle[round(orientation['pitch']) - OFFSET]
th = threading.Thread(target=read_sense_hat_angle, args=(), name='th')
th.start()

# 積算開始
start = time.time()

# 制御パラメータ設定
k1 = Decimal(str(338.05839545))
k2 = Decimal(str(46.78625234))
k3 = Decimal(str(10.95445115))
k4 = Decimal(str(7.6766376))

# モータ初期化
direction_a = 0
direction_d = 0
motor_driver.MotorRun(0, direction[direction_a], SPEED)
motor_driver.MotorRun(1, direction[direction_d], SPEED)

getcontext().prec = 28
while True:
    # 振子の回転角度を取得
    angle_y = Decimal(str(th_angle_y))
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
    if np.sign(delta_theta_a) == np.sign(delta_angle_y):
        motor_a_angle = delta_theta_a - delta_angle_y
    else:
        motor_a_angle = delta_angle_y - delta_theta_a
    if np.sign(delta_theta_d) == np.sign(delta_angle_y):
        motor_d_angle = delta_theta_d - delta_angle_y
    else:
        motor_d_angle = delta_angle_y - delta_theta_d
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
    motor_driver.MotorRun(0, direction[direction_a], ua)
    motor_driver.MotorRun(1, direction[direction_d], ua)
    # ハードウェア側の応答を見て待ち時間を設定
    #time.sleep(0.002)
    