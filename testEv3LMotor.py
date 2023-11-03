#!/usr/bin/python

import time
import requests
import simplejson as json

from modules.PCA9685 import PCA9685
#from RPi import GPIO

from threading import Event
from gpiozero import RotaryEncoder, Button
from gpiozero.pins.pigpio import PiGPIOFactory

SPEED = 100
# APIサーバ
URL = 'http://192.168.0.56:8000'
PATH = '/controller/observer/send/'
# Requestパラメータ
session_id = 0
counter = 0
mode = -1
headers = {"Content-Type" : "application/json"}

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

while True:
    json_data = {
            'session_id': str(session_id)
            ,'counter': counter
            ,'mode': mode
            ,'a_speed': 0
            ,'a_position': rt_a_counter
            #,'a_aposition': int(motor_a.get_aposition())
            ,'b_speed': 0
            ,'b_position': rt_d_counter
            #,'b_aposition': int(motor_b.get_aposition())
            ,'delta': time.time() - start
    }
    # 制御器へRequest
    response = requests.post(URL + PATH, headers=headers, data = json.dumps(json_data))
    # Responseをキャスト
    response = json.loads(response.text)
    response = response[0]

    rt_a_counter = rotorA.steps
    rt_d_counter = rotorD.steps
    print("rt_a_counter  : " + str(rt_a_counter) + "\t rt_d_counter  : " + str(rt_d_counter) + "\t delta : " + str(time.time() - start))
    # deleta theata計測用
    #beforeACount = rt_a_counter
    #beforeDCount = rt_d_counter
    
    session_id = response['session_id']
    counter = int(response['counter']) + 1
    mode = int(response['mode'])
    stop_signal = int(response['stop_signal'])
    start = time.time()
    time.sleep(0.05)
    # 
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