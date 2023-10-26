import time
import simplejson as json
import requests
from buildhat import Motor

URL = 'http://192.168.0.85:8000'
PATH = '/controller/observer/send/'

session_id = 0
counter = 0
mode = -1
headers = {"Content-Type" : "application/json"}

def handle_motor(speed, pos, apos):
    print("Motor", speed, pos, apos)

speed = 20
degrees = 90
a_position = 0
# モータ初期化
motor_a = Motor('A')
motor_b = Motor('B')
start = time.time()
motor_a.set_default_speed(speed)
#motor_a.pwm(0.2)
motor_a.release = False
motor_a.when_rotated = handle_motor
motor_b.set_default_speed(speed)
#motor_b.pwm(0.2)
motor_b.release = False
motor_a.when_rotated = handle_motor
while True:
    json_data={
            'session_id': str(session_id)
            ,'counter': counter
            ,'mode': mode
            ,'a_speed': int(motor_a.get_speed())
            ,'a_position': int(motor_a.get_position())
            #,'a_aposition': int(motor_a.get_aposition())
            ,'b_speed': int(motor_b.get_speed())
            ,'b_position': int(motor_b.get_position())
            #,'b_aposition': int(motor_b.get_aposition())
            ,'delta': time.time() - start
    }
    # 制御器へRequest
    response = requests.post(URL + PATH, headers=headers, data = json.dumps(json_data))
    # Responseをキャスト
    response = json.loads(response.text)
    response = response[0]
    #print(response)
    #print(str(time.time() - start) + "\t" + str(json_data['a_position']) + "\t " + str(json_data['b_position']))
    # Responseの内容でローカル変数を上書き
    session_id = response['session_id']
    counter = int(response['counter']) + 1
    mode = int(response['mode'])
    stop_signal = int(response['stop_signal'])
    start = time.time()
    # 
    print(counter)
    print(abs(abs(json_data['a_position']) - abs(a_position)))
    if counter == 1 or abs(abs(json_data['a_position']) - abs(a_position)) >= abs(degrees):
        motor_a.stop()
        motor_b.stop()
        print(abs(abs(json_data['a_position']) - abs(a_position)))
        a_position = json_data['a_position']
        degrees = degrees * (-1)
        motor_a.run_for_degrees(degrees=degrees, speed=speed, blocking=False)
        motor_a.start()
        motor_b.run_for_degrees(degrees=degrees, speed=speed, blocking=False)
        motor_b.start()
    # 停止コード受信でループを抜ける
    if stop_signal == 1:
        motor_a.stop()
        motor_b.stop()
        del motor_a
        del motor_b
        break
    else:
        continue
    
