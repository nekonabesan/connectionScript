import os
import time
import math
from decimal import *
from WitMotionSensorConnection import JY901S

obj = None
device = None

# 角度センサ初期化
devicePath = "/dev/ttyUSB0"
if os.path.exists("/dev/ttyUSB1"):
    devicePath = "/dev/ttyUSB1"
obj = JY901S()
device = obj.getDevice()
device.serialConfig.portName = devicePath   # Set serial port
device.serialConfig.baud = 9600                     # Set baud rate
device.openDevice()                                 # Open serial port
# Read configuration information
obj.readConfig(device)
# Data update event
device.dataProcessor.onVarChanged.append(obj.onUpdate)

# 積算開始
start = time.time()

getcontext().prec = 28
if __name__ == "__main__":
    accel_x,angular_velocity_x,angle_x,accel_y,angular_velocity_y,angle_y, accel_z,angular_velocity_z,angle_z = obj.getDataAxisAll()
    #accel_y,angular_velocity_y,angle_y = obj.getYDataAxisY()
    angle_x = Decimal(str(math.floor(angle_x)))
    angle_y = Decimal(str(math.floor(angle_y)))
    angle_z = Decimal(str(math.floor(angle_z)))

    while True:
        print(
            'angle_x : ' + str(angle_x)
            + '\tangle_y : ' + str(angle_y)
            + '\tangle_z : ' + str(angle_z))
        time.sleep(0.01)