# -*- coding: utf-8 -*-
import smbus
import time
from time import sleep
import numpy as np
from imu_ekf import *

class angelFromSensor():
    DEV_ADDR = 0x68

    ACCEL_XOUT = 0x3b
    ACCEL_YOUT = 0x3d
    ACCEL_ZOUT = 0x3f
    TEMP_OUT = 0x41
    GYRO_XOUT = 0x43
    GYRO_YOUT = 0x45
    GYRO_ZOUT = 0x47

    PWR_MGMT_1 = 0x6b
    PWR_MGMT_2 = 0x6c   

    bus = None
    Rxyz = None
    ts_pre = None
    #bus = smbus.SMBus(1)
    #bus.write_byte_data(DEV_ADDR, PWR_MGMT_1, 0)

    def __init__(self):
        self.ts_pre = time.time()
        self.bus = smbus.SMBus(1)
        self.bus.write_byte_data(self.DEV_ADDR, self.PWR_MGMT_1, 0)

    def read_word(self, adr):
        high = self.bus.read_byte_data(self.DEV_ADDR, adr)
        low = self.bus.read_byte_data(self.DEV_ADDR, adr+1)
        val = (high << 8) + low
        return val

    def read_word_sensor(self, adr):
        val = self.read_word(adr)
        if (val >= 0x8000):  return -((65535 - val) + 1)
        else:  return val

    def get_temp(self):
        temp = self.read_word_sensor(self.TEMP_OUT)
        x = temp / 340 + 36.53      # data sheet(register map)記載の計算式.
        return x

    def getGyro(self):
        x = self.read_word_sensor(self.GYRO_XOUT)/ 131.0
        y = self.read_word_sensor(self.GYRO_YOUT)/ 131.0
        z = self.read_word_sensor(self.GYRO_ZOUT)/ 131.0
        return [x, y, z]


    def getAccel(self):
        x = self.read_word_sensor(self.ACCEL_XOUT)/ 16384.0
        y= self.read_word_sensor(self.ACCEL_YOUT)/ 16384.0
        z= self.read_word_sensor(self.ACCEL_ZOUT)/ 16384.0
        return [x, y, z]


    def calc_u(self, gyro, dt):
        gyro = np.array([
            [gyro[0]],
            [gyro[1]],
            [gyro[2]]
        ])
        u = gyro * dt
        return u

    def calc_z(self, acc):
        z = np.array([
            [np.arctan(acc[1]/acc[2])], 
            [-np.arctan(acc[0]/np.sqrt(acc[1]**2+acc[2]**2))]
            ])
        return z

    def convert_euler_to_Rxyz(self, x):
        c1 = np.cos(x[0][0])
        s1 = np.sin(x[0][0])
        c2 = np.cos(x[1][0])
        s2 = np.sin(x[1][0])
        c3 = np.cos(x[2][0])
        s3 = np.sin(x[2][0])
        Rx = np.array([
            [1, 0, 0],
            [0, c1, -s1],
            [0, s1, c1],
        ])
        Ry = np.array([
            [c2, 0, s2],
            [0, 1, 0],
            [-s2, 0, c2],
        ])
        Rz = np.array([
            [c3, -s3, 0],
            [s3, c3, 0],
            [0, 0, 1],
        ])
        Rxyz = Rz @ Ry @ Rx
        return Rxyz


if __name__ == '__main__':
    obj = angelFromSensor()
    acc = None

    # ekf init
    dt = time.time() - obj.ts_pre
    x = np.array([[0], [0], [0]])
    P = np.diag([1.74E-2*dt**2, 1.74E-2*dt**2, 1.74E-2*dt**2])

    while True:
        ts = time.time()

        # ACC (deg/s^2)
        ax, ay, az = obj.getAccel()
        # angular velocity (deg/sec)
        gx, gy, gz = obj.getGyro()

        acc = -np.array([ax, ay, az])
        gyro = np.array([gx, gy, gz])
            
        dt = ts - obj.ts_pre
        u = obj.calc_u(gyro, dt)
        z = obj.calc_z(acc)
        R = np.diag([1.0*dt**2, 1.0*dt**2])
        Q = np.diag([1.74E-2*dt**2, 1.74E-2*dt**2, 1.74E-2*dt**2])
        # ekf
        x, P = ekf(x, u, z, P, R, Q)
        # send to viz
        obj.Rxyz = obj.convert_euler_to_Rxyz(x)
        '''with open('angle.tsv', mode='w') as f:
            f.write(
                str(obj.Rxyz[0][0])
                + "," + str(obj.Rxyz[0][1])
                + "," + str(obj.Rxyz[0][2])
                + "," + str(obj.Rxyz[1][0])
                + "," + str(obj.Rxyz[1][1])
                + "," + str(obj.Rxyz[1][2])
                + "," + str(obj.Rxyz[2][0])
                + "," + str(obj.Rxyz[2][1])
                + "," + str(obj.Rxyz[2][2]))
        '''
        obj.ts_pre = ts
        print(obj.Rxyz)
