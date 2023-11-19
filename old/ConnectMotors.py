import gc
from buildhat import Motor

class ConnectMotors():
    MOTOR_NAME_A = 'A'
    MOTOR_NAME_B = 'B'

    motor_a = None
    motor_b = None
    logger = None
    
    def __init__(self):
        self.init_motor()

    def init_motor(self):
        self.motor_a = Motor(self.MOTOR_NAME_A)
        self.motor_a.release = False
        self.motor_a.set_default_speed(0)
        self.motor_a.plimit(1.0)
        self.motor_a.when_rotated = self.handle_motor
        self.motor_b = Motor(self.MOTOR_NAME_B)
        self.motor_b.release = False
        self.motor_b.set_default_speed(0)
        self.motor_b.plimit(1.0)
        self.motor_b.when_rotated = self.handle_motor

    def handle_motor(self, speed, pos, apos):
        print("Motor", speed, pos, apos)
    
    def stopMotors(self):
        self.motor_a.stop()
        self.motor_b.stop()
        return True

    def releaseMotors(self):
        del self.motor_a
        del self.motor_b
        gc.collect()
        return True

    def __del__(self):
        print("Delete Connnection")