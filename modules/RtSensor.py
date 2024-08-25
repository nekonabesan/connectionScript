#!/usr/bin/python
import serial
import time
from decimal import *

class RtSensor():
    port = None
    rate = None
    time_out_sec = None
    ser = None
    gyro_offset = None

    def get_gyro_offset(self):
        return self.gyro_offset

    def set_gyro_offset(self, gyro_offset):
        self.gyro_offset = gyro_offset

    # /--------------------------------------------------------------/ #
    # constructor
    # /--------------------------------------------------------------/ #
    def __init__(self, port = '/dev/ttyACM0', rate = 115200, time_out_sec = 0.5):
        self.port = port
        self.rate = rate
        self.time_out_sec = time_out_sec
        self.ser = serial.Serial(self.port, self.rate, timeout = self.time_out_sec)
        self.calibration_gyro_ofset()

    # /--------------------------------------------------------------/ #
    # センサを校正
    # /--------------------------------------------------------------/ #
    def calibration_gyro_ofset(self):
        gyro_minimam_rate = 0
        gyro_maximum_rate = 2
        gyro_sum = 0
        while (gyro_maximum_rate - gyro_minimam_rate) < 2:
            gyro_minimam_rate = 440
            gyro_maximum_rate = -440
            gyro_sum = 0
            for index in range(200):
                result = self.getSensorValues()
                gyro_sensor_value = result[3]
                gyro_sum = gyro_sum + gyro_sensor_value
                if gyro_sensor_value > gyro_maximum_rate:
                    gyro_maximum_rate = gyro_sensor_value
                if gyro_sensor_value < gyro_minimam_rate:
                    gyro_minimam_rate = gyro_sensor_value
                time.sleep(0.004)
        self.gyro_offset = (gyro_sum / 200)

    # /--------------------------------------------------------------/ #
    # 角速度と角度を導出 
    # /--------------------------------------------------------------/ #
    def get_robot_body_angle_and_speed(self):
        result = self.getSensorValues()
        gyro_offset = self.get_gyro_offset()
        velosity_y =  Decimal(str(result[2]))
        gyro_offset = (Decimal(str(0.0005)) * Decimal(str(velosity_y))) + (Decimal(str(1 - 0.0005)) * Decimal(str(gyro_offset)))
        self.gyro_offset = gyro_offset
        velosity_y = velosity_y - gyro_offset
        return velosity_y, gyro_offset

    # /---------------------------------------------------------------------------------------------------------------/ #
    # センサー出力値を返す
    # 4.10 ASCII 出力時通信プロトコル
    # 出力データは次のようなカンマ区切りのフォーマットになります. 行の最後には改行が入っています.
    # 改行コードは LF です. 
    # ASCII 出力時は USB のみから出力が行われます.
    # タイムスタンプ,角速度 X,角速度 Y,角速度 Z. 加速度 X, 加速度 Y, 加速度 Z, 地磁気 X, 地磁気 Y, 地磁気 Z, 温度(改行 LF)
    # /---------------------------------------------------------------------------------------------------------------/ #
    def getSensorValues(self):
        data = self.ser.readline()
        return data.decode().replace('\n','').split(',')

