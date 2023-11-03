import time
from buildhat import Motor
from WitMotionSensor import *
from buildhat import MotorPair

def handle_motor(speed, pos, apos):
    print("Motor", speed, pos, apos)

# タイマ初期化
start = time.time()
# ------------------------------------------------------ #
# モータ初期化
# ------------------------------------------------------ #
motor_a = Motor('A')
motor_b = Motor('B')
speed = 0
# モータA
motor_a.plimit(1)
#motor_a.set_default_speed(speed)
motor_a.pwm(0.2)
motor_a.release = False
#motor_a.when_rotated = handle_motor
motor_a.stop()
# モータB
motor_b.plimit(1)
#motor_b.set_default_speed(speed)
motor_b.pwm(0.2)
motor_b.release = False
#motor_a.when_rotated = handle_motor
motor_b.stop()

# Pare
#pair = MotorPair('A', 'B')
#pair.set_default_speed(20)

# ------------------------------------------------------ #
# センサ初期化
# ------------------------------------------------------ #
obj = BWT901("/dev/ttyUSB0")
angle_x = 0
angle_y = 0
angle_z = 0

# ------------------------------------------------------ #
# 制御パラメータ
# ------------------------------------------------------ #
K1 = -485.64150485
K2 = -15.48803596
K3 = 3.16227766
K4 = 2.92929776
alpha = 893.2330827067667
beta = -2.9887005248213923
gumma = -4.043902175497531



while True:
    accel_x,angular_velocity_x,angle_x,accel_y,angular_velocity_y,angle_y, accel_z,angular_velocity_z,angle_z = obj.getAngle()
    angle_a = int(motor_a.get_aposition())
    angle_b = int(motor_b.get_aposition())

    
    print(angle_b)
    #motor_a.run_for_degrees(u)
    #motor_b.run_for_degrees(-u)
