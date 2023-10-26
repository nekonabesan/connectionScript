# -*- coding: utf-8 -*-
import smbus
import time
from time import sleep
import numpy as np

class AngelFromSensor():
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

    def __init__(self):
        self.ts_pre = time.time()
        self.bus = smbus.SMBus(1)
        self.bus.write_byte_data(self.DEV_ADDR, self.PWR_MGMT_1, 0)

    # ---------------------------------------------------------------------- #
    # extended kalman filter
    # ---------------------------------------------------------------------- #
    def f(self, x, u):
        u_x, u_y, u_z = u[0][0], u[1][0], u[2][0]
        c1, s1 = np.cos(x[0][0]), np.sin(x[0][0])
        c2, s2 = np.cos(x[1][0]), np.sin(x[1][0])
        c3, s3 = np.cos(x[2][0]), np.sin(x[2][0])
        x = np.array([
            [x[0][0]+u_x+u_y*s1*s2/c2+u_z*c1*s2/c2],
            [x[1][0]+u_y*c1-u_z*s1],
            [x[2][0]+u_y*s1/c2+u_z*c1/c2]
        ])
        return x

    def h(self, x):
        y = np.eye(2, 3) @ x
        return y

    def predict_x(self, x, u):
        x = self.f(x, u)
        return x

    def predict_P(self, P, F, Q):
        P = F @ P @ F.T + Q
        return P

    def calc_F(self, x, u):
        u_x, u_y, u_z = u[0][0], u[1][0], u[2][0]
        c1, s1 = np.cos(x[0][0]), np.sin(x[0][0])
        c2, s2 = np.cos(x[1][0]), np.sin(x[1][0])
        c3, s3 = np.cos(x[2][0]), np.sin(x[2][0])
        F = np.array([
            [1+u_y*c1*s2/c2-u_z*s1*s2/c2, u_y*s1/c2**2+u_z*c1/c2**2, 0],
            [-u_y*s1-u_z*c1, 1, 0],
            [u_y*c1/c2-u_z*s1/c2, u_y*s1*s2/c2**2+u_z*c1*s2/c2**2, 1]
        ])
        return F

    def calc_H(self):
        H = np.eye(2, 3)
        return H

    def update_y_res(self, z, x):
        y_res = z - self.h(x)
        return y_res

    def update_S(self, P, H, R):
        S = H @ P @ H.T + R
        return S

    def update_K(self, P, H, S):
        K = P @ H.T @ np.linalg.inv(S)
        return K

    def update_x(self, x, y_res, K):
        x = x + K @ y_res
        return x

    def update_P(self, P, H, K):
        I = np.identity(3)
        P = (I - K @ H) @ P
        return P

    def extendedKalmanFilter(self, x, u, z, P, R, Q):
        # Predict
        F = self.calc_F(x, u)
        x = self.predict_x(x, u)
        H = self.calc_H()
        P = self.predict_P(P, F, Q)
        # Update
        y_res = self.update_y_res(z, x)
        S = self.update_S(P, H, R)
        K = self.update_K(P, H, S)
        x = self.update_x(x, y_res, K)
        P = self.update_P(P, H, K)
        return x, P
    # ---------------------------------------------------------------------- #

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
    
    def __del__(self):
        return True
'''
    def calc(self):
        acc = None

        # ekf init
        dt = time.time() - self.ts_pre
        x = np.array([[0], [0], [0]])
        P = np.diag([1.74E-2*dt**2, 1.74E-2*dt**2, 1.74E-2*dt**2])

        while True:
            ts = time.time()

            # ACC (deg/s^2)
            ax, ay, az = self.getAccel()
            # angular velocity (deg/sec)
            gx, gy, gz = self.getGyro()

            acc = -np.array([ax, ay, az])
            gyro = np.array([gx, gy, gz])
            
            dt = ts - self.ts_pre
            u = self.calc_u(gyro, dt)
            z = self.calc_z(acc)
            R = np.diag([1.0*dt**2, 1.0*dt**2])
            Q = np.diag([1.74E-2*dt**2, 1.74E-2*dt**2, 1.74E-2*dt**2])
            # ekf
            x, P = ekf(x, u, z, P, R, Q)
            # send to viz
            self.Rxyz = self.convert_euler_to_Rxyz(x)

            self.ts_pre = ts
'''