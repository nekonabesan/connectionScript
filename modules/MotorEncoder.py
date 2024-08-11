#!/usr/bin/python
from gpiozero import RotaryEncoder, Button
from gpiozero.pins.pigpio import PiGPIOFactory

FWD = 0
REV = 1

class MotorEncoder():
    

    rotary_encoder_a = None
    rotary_encoder_b = None
    pin_rotor_a1 = None
    pin_rotor_a2 = None
    pin_rotor_d1 = None
    pin_rotor_d2 = None
    rotation_a = None
    rotation_b = None
    factory = None

    def __init__(self, init_rotation_a, init_rotation_b, pin_rotor_a1, pin_rotor_a2, pin_rotor_d1, pin_rotor_d2):
        self.rotation_a = init_rotation_a
        self.rotation_b = init_rotation_b
        # ロータリーエンコーダのピン設定
        self.pin_rotor_a1 = pin_rotor_a1
        self.pin_rotor_a2 = pin_rotor_a2
        self.pin_rotor_d1 = pin_rotor_d1
        self.pin_rotor_d2 = pin_rotor_d2
        # ロータリーエンコーダ初期化
        self.factory = PiGPIOFactory()
        self.rotary_encoder_a = RotaryEncoder(
            self.pin_rotor_a1, self.pin_rotor_a2, wrap=True, max_steps=180, pin_factory=self.factory
        )
        self.rotary_encoder_a.steps = 0
        self.rotary_encoder_a.when_rotated_clockwise = self.rotated_clockwise_a
        self.rotary_encoder_a.when_rotated_counter_clockwise = self.rotated_counter_clockwise_a

        self.rotary_encoder_b = RotaryEncoder(
            self.pin_rotor_d1, self.pin_rotor_d2, wrap=True, max_steps=180, pin_factory=self.factory
        )
        self.rotary_encoder_b.steps = 0
        self.rotary_encoder_b.when_rotated_clockwise = self.rotated_clockwise_d
        self.rotary_encoder_b.when_rotated_counter_clockwise = self.rotated_counter_clockwise_d

    def rotated_clockwise_a(self):
        self.rotation_a = REV

    def rotated_clockwise_d(self):
        self.rotation_b = REV

    def rotated_counter_clockwise_a(self):
        self.rotation_a = FWD

    def rotated_counter_clockwise_d(self):
        self.rotation_b = FWD
    
    

    

    