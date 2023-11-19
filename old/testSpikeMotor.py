import time
import math
import numpy as np
from decimal import *
from buildhat import Motor

def handle_motor(speed, pos, apos):
    print("Motor", speed, pos, apos)

motor_a = Motor('A')
motor_b = Motor('B')

#motor_a.set_default_speed(0)
motor_a.plimit(1.0)
motor_a.pwm(1.0)
motor_a.when_rotated = handle_motor

#motor_b.set_default_speed(0)
motor_b.plimit(1.0)
motor_b.pwm(1.0)
motor_b.when_rotated = handle_motor


while(True):
    motor_a.start(20)
    motor_b.start(20)
    time.sleep(0.01)
    