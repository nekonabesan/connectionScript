import asyncio
import time
from decimal import Decimal, getcontext
from modules.PCA9685 import PCA9685
from modules.MotorEncoder import MotorEncoder
from modules.MotorDriver import MotorDriver
from modules.RtSensor import RtSensor

FWD, REV = 0, 1
getcontext().prec = 28

kp = Decimal("80")
kd = Decimal("10")
ki = Decimal("0.2")

def calc_duty_cycle(delta, acc):
    duty_cycle = (acc + Decimal("192.066")) / Decimal("39.784")
    return round(duty_cycle)

def compute_pid_output(angle, velocity_y, integral):
    return -(angle * kp + velocity_y * kd + integral * ki)

def get_direction(value):
    return REV if value < 0 else FWD

async def mainloop(interval, sensor, driver, pwm):
    angle = Decimal("0.0")
    integral = Decimal("0.0")
    start = time.time()

    while True:
        delta_time = Decimal(str(time.time() - start))
        start = time.time()

        raw = sensor.get_raw_data()  # 同期読み出しでOK
        calibrated = sensor.apply_calibration(raw)

        acc_y = Decimal(str(calibrated[1]))
        velocity_y = Decimal(str(calibrated[4]))

        angle += velocity_y * delta_time
        integral += angle * delta_time

        acc = compute_pid_output(angle, velocity_y, integral)
        direction = get_direction(acc)
        acc_abs = abs(acc)

        pwm_val = calc_duty_cycle(delta_time, acc_abs)

        print(f"dt: {delta_time:.3f}\tangle_y: {angle:.3f}\tvelocity_y: {velocity_y:.3f}\tacc: {acc_abs:.3f}\tpwm: {pwm_val:.3f}")

        driver.MotorRun(pwm, 0, MotorDriver.direction[direction], pwm_val)
        driver.MotorRun(pwm, 1, MotorDriver.direction[direction], pwm_val)

        await asyncio.sleep(interval)

# 同期初期化領域
if __name__ == "__main__":
    encoder = MotorEncoder(FWD, FWD, 16, 20, 19, 26)
    driver = MotorDriver()
    pwm = PCA9685(0x40, debug=False)
    pwm.setPWMFreq(50)

    sensor = RtSensor()
    port = sensor.get_port()
    sensor.initialize()  # 必要に応じてポートを引数で指定

    asyncio.run(mainloop(0.01, sensor, driver, pwm))