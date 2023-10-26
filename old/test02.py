import gc
import sys
import time
from buildhat import Motor
from buildhat import MotorPair
import ConnectMotors as ConnectMotors

connection = ConnectMotors.ConnectMotors()
connection.motor_a.set_default_speed(20)
connection.motor_a.pwm(1.0)



connection.motor_a.run_for_degrees(-330)
'''
connection.motor_a.run_for_seconds(1, 10, False)
time.sleep(0.8)
connection.motor_a.run_for_seconds(1, 20, False)
time.sleep(0.8)
connection.motor_a.run_for_seconds(1, 30, False)
time.sleep(0.8)
connection.motor_a.run_for_seconds(1, 40, False)
time.sleep(0.8)
connection.motor_a.run_for_seconds(1, 50, False)
time.sleep(0.8)
'''

