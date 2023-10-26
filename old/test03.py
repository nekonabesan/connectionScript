import AngelFromSensor
import time
import numpy as np

def main():
    sensorObj = AngelFromSensor()
    acc = None


    # ekf init
    dt = time.time() - sensorObj.ts_pre
    x = np.array([[0], [0], [0]])
    P = np.diag([1.74E-2*dt**2, 1.74E-2*dt**2, 1.74E-2*dt**2])

    while True:
        ts = time.time()

        # ACC (deg/s^2)
        ax, ay, az = sensorObj.getAccel()
        # angular velocity (deg/sec)
        gx, gy, gz = sensorObj.getGyro()

        acc = -np.array([ax, ay, az])
        gyro = np.array([gx, gy, gz])
            
        dt = ts - sensorObj.ts_pre
        u = sensorObj.calc_u(gyro, dt)
        z = sensorObj.calc_z(acc)
        R = np.diag([1.0*dt**2, 1.0*dt**2])
        Q = np.diag([1.74E-2*dt**2, 1.74E-2*dt**2, 1.74E-2*dt**2])
        # ekf
        x, P = sensorObj.ekf(x, u, z, P, R, Q)
        # send to viz
        sensorObj.Rxyz = sensorObj.convert_euler_to_Rxyz(x)

        sensorObj.ts_pre = ts


if __name__ == '__main__' :
    main()