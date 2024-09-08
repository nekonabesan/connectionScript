import math
import time
import threading
from decimal import *
from modules.PCA9685 import PCA9685
from modules.MotorEncoder import MotorEncoder
from modules.MotorDriver import MotorDriver
from modules.RtSensor import RtSensor

FWD = 0
REV = 1

angle = 0
driver = None
enocoder = None
sensor = None
pwm = None
integral = None

kp = Decimal(str(80))
kd = Decimal(str(10))
ki = Decimal(str(0.2))

# calc duty-cycle
def calc_duty_cycle(delta, acc):
    duty_cycle = (acc + Decimal(str(192.066))) / Decimal(str(39.784))
    return round(duty_cycle)

def worker ():
    global angle
    global start
    global driver
    global enocoder
    global sensor
    global pwm
    global integral

    delta_time = Decimal(time.time() - start)
    start = time.time()

    # 傾斜角取得
    # 0:タイムスタンプ,1:角速度 X,2:角速度 Y,3:角速度 Z. 4:加速度 X, 5:加速度 Y, 6:加速度 Z, 7:地磁気 X, 8:地磁気 Y, 9:地磁気 Z, 10:温度
    # それぞれの値の単位系は以下のようになります.
    # タイムスタンプ:0 から 255 までの整数
    # 角速度:小数値[rad/sec]
    # 加速度:小数値[G]
    # 地磁気:小数値[μT]
    # 温度:小数値[℃]
    result = sensor.getSensorValues()
    velosity_y, gyro_offset = sensor.get_robot_body_angle_and_speed()
    velosity_y = Decimal(str(velosity_y)) * Decimal(str(180/math.pi))
    acc_a = Decimal(str(result[5])) * Decimal(str(9.8))
    angle = angle + (velosity_y * delta_time)

    if angle > 45:
        angle = Decimal(str(45))
    if angle < -45:
        angle = Decimal(str(-45))

    integral = Decimal(str(integral)) + angle * delta_time
    if integral < -2:
        integral = Decimal(str(-2))
    if integral > 2:
        integral = Decimal(str(2))

    # 加速度を制御入力とする
    acc_a = (Decimal(str(-1)) * ((angle * kp) + (velosity_y * kd) + (integral * ki)))
    acc_d = (Decimal(str(-1)) * ((angle * kp) + (velosity_y * kd) + (integral * ki)))

    # 回転方向を設定
    if acc_a < 0:
        direction_a = REV
    else:
        direction_a = FWD

    if acc_d < 0:
        direction_d = REV
    else:
        direction_d = FWD

    # 制御入力を絶対値に変換
    acc_a = abs(acc_a)
    acc_d = abs(acc_d)

    # 加速度をデューティー比に変換
    pwm_a = calc_duty_cycle(delta_time, acc_a)
    pwm_d = calc_duty_cycle(delta_time, acc_d)

    print(
        "dt : " + str("{:.3f}".format(delta_time))
        + "\tangle_y  : " + str("{:.3f}".format(angle))
        + "\tvelosity_y : " + str("{:.3f}".format(velosity_y))
        + "\t acc : " + str("{:.3f}".format(acc_a))
        + "\t pwm : " + str("{:.3f}".format(pwm_a))
    )

    # モータへデューティ比を印加
    driver.MotorRun(pwm, 0, MotorDriver.direction[direction_a], pwm_a)
    driver.MotorRun(pwm, 1, MotorDriver.direction[direction_d], pwm_d)
    
    # ハードウェア側の応答を見て待ち時間を設定
    #time.sleep(0.002)


start = time.time()
getcontext().prec = 28
def mainloop(interval):
    global start
    # 積算開始
    now = time.time()
    while True:
        t = threading.Thread(target=worker)
        t.setDaemon(True)
        t.start()
        t.join()
        wait_time = interval - ((time.time() - now) % interval)
        time.sleep(wait_time)

if __name__ == "__main__":
    encoder = MotorEncoder(FWD, FWD, 16, 20, 19, 26)
    driver = MotorDriver()
    sensor = RtSensor()
    integral = Decimal(str(0))
    # PWM制御
    pwm = PCA9685(0x40, debug=False)
    pwm.setPWMFreq(50)
    pwm.setPWMFreq(50000000.0)
    mainloop(0.01)