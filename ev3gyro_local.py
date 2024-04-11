#!/usr/bin/python

from gpiozero import InputDevice
from gpiozero.pins.pigpio import PiGPIOFactory

# ロータリーエンコーダのピン設定
PIN_GYRO_A1 = 19
PIN_GYRO_A2 = 26

factory = PiGPIOFactory()
input1 = InputDevice(
    pin=PIN_GYRO_A1, pull_up=None, active_state=True, pin_factory=factory
)
input2 = InputDevice(
    pin=PIN_GYRO_A2, pull_up=None, active_state=True, pin_factory=factory
)

while True:
    print(str(input1.value) + str(input2.value))
