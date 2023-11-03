import time
import requests
import subprocess
import numpy as np
import simplejson as json
from WitMotionSensor import *
#from imu_ekf import *
#from buildhat import Motor

URL = 'http://192.168.0.56:8000'
PATH = '/controller/observer/physical/'

# モータ初期化
#motor_a = Motor('A')
#motor_b = Motor('B')
#motor_a.release = True
#motor_b.release = True
counter = 0
session_id = 0
headers = {"Content-Type" : "application/json"}
# タイマ初期化
start = time.time()
obj = BWT901("/dev/ttyUSB0")
angle_x = 0
angle_y = 0
angle_z = 0


while True:
    accel_x,angular_velocity_x,angle_x,accel_y,angular_velocity_y,angle_y, accel_z,angular_velocity_z,angle_z = obj.getAngle()
    json_data={
        'session_id': str(session_id)
        ,'counter': counter
        ,'Rx_acc': accel_x
        ,'Rx_velocity': angular_velocity_x
        ,'Rx_degrees': angle_x
        ,'Ry_acc': accel_y
        ,'Ry_velocity': angular_velocity_y
        ,'Ry_degrees': angle_y
        ,'Rz_acc': accel_z
        ,'Rz_velocity': angular_velocity_z
        ,'Rz_degrees': angle_z
        ,'delta': time.time() - start
    }
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
    start = time.time()
    # 停止コード受信でループを抜ける
    if stop_signal == 1:
        break
    else:
        continue