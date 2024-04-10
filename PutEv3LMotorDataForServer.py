#!/usr/bin/python

import time
import requests
import simplejson as json

from modules.PCA9685 import PCA9685
#from RPi import GPIO

from threading import Event
from gpiozero import RotaryEncoder, Button
from gpiozero.pins.pigpio import PiGPIOFactory

SPEED = 20
FWD = 0
BCK = 1

# APIサーバ
URL = 'http://192.168.0.56:8000'
PATH = '/controller/observer/send/'
# Requestパラメータ
session_id = 0
counter = 0
mode = -1
headers = {"Content-Type" : "application/json"}

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
delta_theta_a = 0
delta_theta_d = 0
rt_a_counter = 0  #エンコーダー積算
rt_d_counter = 0  #エンコーダー積算
# 積算開始
start = time.time()
delta = time.time() - start

while True:
    #delta = time.time() - start
    #start = time.time()
    json_data = {
            'session_id': str(session_id)
            ,'counter': counter
            ,'mode': mode
            ,'a_speed': 0
            ,'a_position': rt_a_counter
            ,'delta_theta_a': delta_theta_a
            ,'d_speed': 0
            ,'d_position': rt_d_counter
            ,'delta_theta_d': delta_theta_d
            ,'delta': delta
    }
    # 制御器へRequest
    response = requests.post(URL + PATH, headers=headers, data = json.dumps(json_data))
    # Responseをキャスト
    response = json.loads(response.text)
    response = response[0]

    rt_a_counter = rotary_encoder_a.steps
    rt_d_counter = rotary_encoder_d.steps

    # １ステップ前からのモータ回転角度A
    if (rt_a_counter >= 0 & before_count_a < 0):
        delta_theta_a = abs(rt_a_counter + (180 + before_count_a))
    elif (before_count_a >= 0 & rt_a_counter < 0):
        delta_theta_a = abs(before_count_a  + rt_a_counter)
    else:
        delta_theta_a = abs(abs(before_count_a) - abs(rt_a_counter))

    # １ステップ前からのモータ回転角度D
    if (rt_d_counter >= 0 & before_count_d < 0):
        delta_theta_d = abs(rt_d_counter + (180 + before_count_d))
    elif (before_count_d >= 0 & rt_d_counter < 0):
        delta_theta_d = abs(before_count_d  + rt_d_counter)
    else:
        delta_theta_d = abs(abs(before_count_d) - abs(rt_d_counter))
    
    session_id = response['session_id']
    counter = int(response['counter']) + 1
    mode = int(response['mode'])
    stop_signal = int(response['stop_signal'])

    # deleta theata計測用
    before_count_a = rt_a_counter
    before_count_d = rt_d_counter
    
    delta = time.time() - start
    start = time.time()

    print(
        "rt_a_counter  : " + str(rt_a_counter) 
        + "\t delta_theta_a : " + str(delta_theta_a)
        + "\t rt_d_counter  : " + str(rt_d_counter) 
        + "\t delta_theta_d : " + str(delta_theta_d)
        + "\t delta : " + str(delta)
    )

    time.sleep(0.04)
    # モータ回転方向を初期化
    if counter == 1:
        Motor.MotorRun(0, 'forward', SPEED)
        Motor.MotorRun(1, 'forward', SPEED)
    # 停止コード受信でループを抜ける
    if stop_signal == 1:
        print("stop")
        Motor.MotorStop(0)
        Motor.MotorStop(1)
        print("stop")
        break
    else:
        continue