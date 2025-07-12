import asyncio
import math
import time
import threading
from decimal import *
from modules.PCA9685 import PCA9685
from modules.MotorEncoder import MotorEncoder
from modules.MotorDriver import MotorDriver
from modules.RtSensor import RtSensor
from serial_asyncio import open_serial_connection



FWD = 0
REV = 1

angle = 0
driver = None
enocoder = None
sensor = None
pwm = None
integral = None
port = None


kp = Decimal(str(80))
kd = Decimal(str(10))
ki = Decimal(str(0.2))

# calc duty-cycle
def calc_duty_cycle(delta, acc):
    duty_cycle = (acc + Decimal(str(192.066))) / Decimal(str(39.784))
    return round(duty_cycle)

def compute_pid_output(angle, velosity_y, integral, kp, ki, kd):
    return -(angle * kp + velosity_y * kd + integral * ki)

def get_direction(value):
    return REV if value < 0 else FWD

async def read_sensor():
    global port
    reader, writer = await open_serial_connection(url=port, baudrate=115200)
    while True:
        line = await reader.readline()
        decoded = line.decode().strip()
        print("Sensor:", decoded)
        await asyncio.sleep(0.001)  # 制御周期に合わせて調整

def worker ():
    global angle
    global start
    global driver
    global enocoder
    global sensor
    global pwm
    global integral
    global port

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

    raw = sensor.get_raw_data()
    calibrated = sensor.apply_calibration(raw)
    acc_y = Decimal(str(calibrated[1]))
    velosity_y = Decimal(str(calibrated[4]))

    # 加速度を角度に変換
    angle = Decimal(angle) + (velosity_y * Decimal(delta_time))

    # PID演算（1回のみ）
    acc = compute_pid_output(angle, velosity_y, integral, kp, ki, kd)

    # 回転方向の判定
    direction_a = get_direction(acc)
    direction_d = get_direction(acc)

    # 制御入力を絶対値に変換
    acc_abs = abs(acc)

    # デューティー比に変換
    pwm_a = calc_duty_cycle(delta_time, acc_abs)
    pwm_d = calc_duty_cycle(delta_time, acc_abs)

    # ログ出力
    print(f"dt: {delta_time:.3f}\tangle_y: {angle:.3f}\tvelosity_y: {velosity_y:.3f}\tacc: {acc_abs:.3f}\tpwm: {pwm_a:.3f}")

    # モータ制御
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
    port = sensor.get_port()
    integral = Decimal(str(0))
    # PWM制御
    pwm = PCA9685(0x40, debug=False)
    pwm.setPWMFreq(50)
    pwm.setPWMFreq(50000000.0)
    mainloop(0.01)