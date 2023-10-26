# ---------------------------------------------------------- #
# モータの動作例
# ---------------------------------------------------------- #
import gc
import sys
import time
from buildhat import Motor
from buildhat import MotorPair
import ConnectMotors as ConnectMotors

# ---------------------------------------------------------- #
# https://qiita.com/Inoue_Minoru/items/23cc2c9fbe24bd686a16
# ---------------------------------------------------------- #
connection = ConnectMotors.ConnectMotors()
connection.motor_a.set_default_speed(30)
connection.motor_b.set_default_speed(30)
connection.motor_a.run_for_degrees(360)
connection.motor_b.run_for_degrees(360)
del connection
gc.collect()
time.sleep(0.05)
print("----------------")
connection = ConnectMotors.ConnectMotors()
connection.motor_a.set_default_speed(50)
connection.motor_b.set_default_speed(50)
connection.motor_a.run_for_degrees(360)
connection.motor_b.run_for_degrees(360)
connection.motor_a.set_default_speed(20)
connection.motor_b.set_default_speed(20)
connection.motor_a.run_for_degrees(-180)
connection.motor_b.run_for_degrees(-180)


del connection
gc.collect()

sys.exit(0)