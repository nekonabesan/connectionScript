#!/usr/bin/python

import time
import requests
import numpy as np
import simplejson as json
import concurrent.futures
from decimal import *
from threading import Event
from sense_hat import SenseHat
from modules.MotorDriver import *
from ConnectionSenseHat import *
from modules.PCA9685 import PCA9685
from multiprocessing import Value, Array, Process
from gpiozero import RotaryEncoder, Button
from gpiozero.pins.pigpio import PiGPIOFactory

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

URL = 'http://192.168.0.56:8000'
PATH = '/controller/feedback/v1/optimal_regulator/'

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

# sense hatの指示値読み取りをスレッド化
#th_angle_y = 0
def read_sense_hat_angle(th_angle_y):
    global angle
    while True:
        orientation = sense.get_orientation_degrees()
        th_angle_y.value = angle[round(orientation['pitch']) - OFFSET]

getcontext().prec = 28
def request_controler(th_angle_y):
    global rotation_a,rotation_d

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

    counter = 0
    session_id = 0
    headers = {"Content-Type" : "application/json"}

    direction = [
        'forward',
        'backward',
    ]

    # 積算開始
    start = time.time()
    delta_time = 0

    # PWM制御
    pwm = PCA9685(0x40, debug=False)
    pwm.setPWMFreq(50)

    motor_driver = MotorDriver()
    pwm.setPWMFreq(50000000.0)

    # カウンタ初期化
    before_count_a = 0
    before_count_d = 0
    rt_a_counter = 0
    rt_d_counter = 0

    prev_angle_y = 0

    # モータ初期化
    direction_a = FWD
    direction_d = FWD
    motor_driver.MotorRun(pwm, 0, direction[direction_a], SPEED)
    motor_driver.MotorRun(pwm, 1, direction[direction_d], SPEED)
    while True:
        # 振子の回転角度を取得
        angle_y = Decimal(str(th_angle_y.value))
        rt_a_counter = rotary_encoder_a.steps
        rt_d_counter = rotary_encoder_d.steps
        json_data={
            'session_id': str(session_id)
            ,'counter': counter
            ,'angle_y': angle_y
            ,'prev_angle_y': prev_angle_y
            ,'rt_a_counter': rt_a_counter
            ,'rt_d_counter': rt_d_counter
            ,'before_count_a': before_count_a
            ,'before_count_d': before_count_d
            ,'rotation_a': rotation_a
            ,'rotation_d': rotation_d
            ,'delta': delta_time
        }
        delta_time = Decimal(time.time() - start)
        start = time.time()
        # 制御器へRequest
        response = requests.post(URL + PATH, headers=headers, data = json.dumps(json_data))
        # Responseをキャスト
        response = json.loads(response.text)
        response = response[0]
        print(response)
        # Responseの内容でローカル変数を上書き
        session_id = response['session_id']
        counter = int(response['counter']) + 1
        stop_signal = int(response['stop_signal'])
        # 制御入力を取得
        direction_a = int(response['direction_a'])
        direction_d = int(response['direction_d'])
        ua = int(response['ua'])
        ud = int(response['ud'])

        # deleta theata計測用
        before_count_a = rt_a_counter
        before_count_d = rt_d_counter
        prev_angle_y = angle_y

        # モータへデューティ比を印加
        motor_driver.MotorRun(pwm, 0, direction[direction_a], ua)
        motor_driver.MotorRun(pwm, 1, direction[direction_d], ua)

        # 停止コード受信でループを抜ける
        if stop_signal == 1:
            # モータ停止
            motor_driver.MotorRun(pwm, 0, direction[direction_a], 0)
            motor_driver.MotorRun(pwm, 1, direction[direction_d], 0)
            break
        else:
            continue

if __name__ == "__main__":
    th_angle_y = Value('i', 0)
    p1 = Process(target=read_sense_hat_angle, args=[th_angle_y])
    p2 = Process(target=request_controler, args=[th_angle_y])
    p1.start()
    p2.start()