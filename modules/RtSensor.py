#!/usr/bin/python
import serial
import serial.tools.list_ports
import time
from decimal import *
from serial_asyncio import open_serial_connection


class RtSensor():
    EXPECTED_LENGTH = 11 
    PORTS = ["/dev/ttyACM0", "/dev/ttyAMA0", "/dev/ttyS0"]

    port = None
    rate = None
    time_out_sec = None
    ser = None
    gyro_offset = None
    offset = None
    scale = None
    reader = None
    writer = None

    # /--------------------------------------------------------------/ #
    # センサの角速度オフセットを取得
    # /--------------------------------------------------------------/ #
    def get_gyro_offset(self):
        return self.gyro_offset

    # /--------------------------------------------------------------/ #
    # センサの角速度オフセットを設定
    # /--------------------------------------------------------------/ #
    def set_gyro_offset(self, gyro_offset):
        self.gyro_offset = gyro_offset

    # /--------------------------------------------------------------/ #
    # センサのポートを取得
    # /--------------------------------------------------------------/ #
    def get_port(self):
        return self.port

    # /--------------------------------------------------------------/ #
    # constructor
    # /--------------------------------------------------------------/ #
    def __init__(self, rate = 115200, time_out_sec = 0.5):
        self.offset = [0.0, 0.0, 0.0]
        self.scale = [1.0, 1.0, 1.0]
        port, _ = self.find_sensor_port()
        self.port = port
        self.rate = rate
        self.time_out_sec = time_out_sec
        self.ser = serial.Serial(self.port, self.rate, timeout = self.time_out_sec)
        self.calibrate_static_orientation()

    # /--------------------------------------------------------------/ #
    # センサの初期化
    # /--------------------------------------------------------------/ #
    async def initialize(self , baudrate=115200):
        self.reader, self.writer = await open_serial_connection(url=self.port, baudrate=baudrate)


    # /--------------------------------------------------------------/ #
    # センサの生データを非同期で読み取る
    # /--------------------------------------------------------------/ #
    async def get_raw_values(self):
        line = await self.reader.readline()
        values = line.decode().strip().split(',')
        if len(values) < 7:
            return None
        accel = [float(values[4]), float(values[5]), float(values[6])]
        gyro = [float(values[1]), float(values[2]), float(values[3])]
        return accel + gyro

    # /--------------------------------------------------------------/ #
    # シリアルポートの一覧を取得
    # /--------------------------------------------------------------/ #
    def list_serial_ports(self):
        ports = serial.tools.list_ports.comports()
        return [port.device for port in ports]

    # /--------------------------------------------------------------/ #
    # シリアルポートに接続を試みる
    # /--------------------------------------------------------------/ #
    def try_connect(self, port, rate=115200, time_out_sec=0.5):
        try:
            ser = serial.Serial(port, rate, timeout=time_out_sec)
            line = ser.readline().decode('utf-8').strip()
            ser.close()

            if line is not None:
                return line
        except Exception as e:
            print(f"Port {port} failed: {e}")
        return None

    # /--------------------------------------------------------------/ #
    # センサのポートを自動検出
    # /--------------------------------------------------------------/ #
    def find_sensor_port(self):
        for port in self.PORTS:
            print(f"Trying port: {port}")
            result = self.try_connect(port)
            if result is not None:
                print(f"Sensor found on {port}")
                return port, result
        print("Sensor not found on listed ports.")
        return None, None

    # /--------------------------------------------------------------/ #
    # センサの生データから角速度,角加速度を取得
    # /--------------------------------------------------------------/ #
    def get_raw_data(self):
        raw_line = self.get_sensor_values()
        parsed = self.parse_sensor_values(raw_line)
        return [parsed['velocity_x'], parsed['velocity_y'], parsed['velocity_z'], parsed["accel_x"], parsed["accel_y"], parsed["accel_z"]]

    # /--------------------------------------------------------------/ #
    # センサの生データを取得し、校正値を適用
    # /--------------------------------------------------------------/ #
    def apply_calibration(self, raw):
        """
        raw: [accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z]
        self.accel_offset: [ox, oy, oz]
        self.accel_scale: [sx, sy, sz]
        self.gyro_offset: [gx, gy, gz]
        return: [calibrated_accel_x, y, z, calibrated_gyro_x, y, z]
        """
        calibrated_accel = [
            (raw[0] - self.accel_offset[0]) / self.accel_scale[0],
            (raw[1] - self.accel_offset[1]) / self.accel_scale[1],
            (raw[2] - self.accel_offset[2]) / self.accel_scale[2]
        ]
        calibrated_gyro = [
            raw[3] - self.gyro_offset[0],
            raw[4] - self.gyro_offset[1],
            raw[5] - self.gyro_offset[2]
        ]
        return calibrated_accel + calibrated_gyro

    # /--------------------------------------------------------------/ #
    # 平均化されたセンサデータを取得
    # /--------------------------------------------------------------/ #
    def average_data(self, num_samples=100, wait=0.01):
        data_list = []
        for _ in range(num_samples):
            data_list.append(self.get_raw_data())
            time.sleep(wait)
            return [sum(axis) / len(axis) for axis in zip(*data_list)]

    # /--------------------------------------------------------------/ #
    # センサの静的な姿勢をキャリブレーション
    # センサを固定した状態で各軸の正負方向を測定
    # /--------------------------------------------------------------/ #
    def calibrate_static_orientation(self):
        def average_data(num_samples=100, wait=0.01):
            accel_samples = []
            gyro_samples = []
            for _ in range(num_samples):
                line = self.ser.readline().decode().strip()
                values = line.split(',')
                if len(values) < 11:
                    continue  # 不正なデータをスキップ
                # accel: index 4,5,6 / gyro: index 1,2,3
                accel = [float(values[4]), float(values[5]), float(values[6])]
                gyro = [float(values[1]), float(values[2]), float(values[3])]
                accel_samples.append(accel)
                gyro_samples.append(gyro)
                time.sleep(wait)
            accel_avg = [sum(axis) / len(axis) for axis in zip(*accel_samples)]
            gyro_avg = [sum(axis) / len(axis) for axis in zip(*gyro_samples)]
            return accel_avg, gyro_avg

        axp, gxp = average_data()
        axm, gxm = average_data()
        ayp, gyp = average_data()
        aym, gym = average_data()
        azp, gzp = average_data()
        azm, gzm = average_data()

        # 加速度のオフセットとスケール
        self.accel_offset = [
            (axp[0] + axm[0]) / 2,
            (ayp[1] + aym[1]) / 2,
            (azp[2] + azm[2]) / 2
        ]
        self.accel_scale = [
            (axp[0] - axm[0]) / 2,
            (ayp[1] - aym[1]) / 2,
            (azp[2] - azm[2]) / 2
        ]

        # 角速度のオフセット（静止状態なので理想値は0）
        self.gyro_offset = [
            (gxp[0] + gxm[0]) / 2,
            (gyp[1] + gym[1]) / 2,
            (gzp[2] + gzm[2]) / 2
        ]
    
    # /--------------------------------------------------------------/ #
    # センサを校正
    # 廃止
    # /--------------------------------------------------------------/ #
    """
    def calibration_gyro_ofset(self):
        gyro_minimam_rate = 0
        gyro_maximum_rate = 2
        gyro_sum = 0
        while (gyro_maximum_rate - gyro_minimam_rate) < 2:
            gyro_minimam_rate = 440
            gyro_maximum_rate = -440
            gyro_sum = 0
            for index in range(200):
                result = self.get_sensor_values()
                gyro_sensor_value = result[3]
                gyro_sum = gyro_sum + gyro_sensor_value
                if gyro_sensor_value > gyro_maximum_rate:
                    gyro_maximum_rate = gyro_sensor_value
                if gyro_sensor_value < gyro_minimam_rate:
                    gyro_minimam_rate = gyro_sensor_value
                time.sleep(0.004)
        self.gyro_offset = (gyro_sum / 200)
    """

    # /--------------------------------------------------------------/ #
    # 角速度と角度を導出 
    # /--------------------------------------------------------------/ #
    def get_robot_body_angle_and_speed(self):
        result = self.get_sensor_values()
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
    def get_sensor_values(self):
        raw_line = self.ser.readline().decode('utf-8').strip().split(',')
        if len(raw_line) != self.EXPECTED_LENGTH:
            while len(raw_line) != self.EXPECTED_LENGTH:
                print(raw_line)
                self.ser.reset_input_buffer()
                raw_line = self.ser.readline().decode('utf-8').strip().split(',')
        if not raw_line:
            print("センサからデータが取得できませんでした")
            return []
        return raw_line
        
    

    # /--------------------------------------------------------------/ #
    # センサデータをパースして辞書形式で返す
    # 配列のindexと格納値は以下
    # 0:タイムスタンプ,1:角速度 X,2:角速度 Y,3:角速度 Z. 4:加速度 X, 5:加速度 Y, 6:加速度 Z, 7:地磁気 X, 8:地磁気 Y, 9:地磁気 Z, 10:温度
    # /--------------------------------------------------------------/ #
    def parse_sensor_values(self, values):
        keys = [
            "timestamp", 
            "velocity_x", "velocity_y", "velocity_z", 
            "accel_x", "accel_y", "accel_z",
            "mag_x", "mag_y", "mag_z",
            "temperature"
        ]
        return dict(zip(keys, map(float, values)))

    import time

    # /--------------------------------------------------------------/ #
    # 平均化されたセンサデータを取得
    # /--------------------------------------------------------------/ #
    def average_samples(self, get_data_func, duration=2.0, interval=0.05):
        """
        指定時間内に複数回センサデータを取得して平均化
        get_data_func: センサから[x, y, z]を取得する関数
        """
        samples = []
        start = time.time()
        while time.time() - start < duration:
            samples.append(get_data_func())
            time.sleep(interval)
        avg = [sum(axis) / len(axis) for axis in zip(*samples)]
        return avg

    # /--------------------------------------------------------------/ #
    # 加速度センサのキャリブレーション
    # /--------------------------------------------------------------/ #
    def calibrate_accelerometer(self, get_data_func_dict):
        """
        get_data_func_dict: 各方向のセンサ取得関数を格納した辞書
        例: {'x+': func1, 'x-': func2, ...}
        """
        x_max = self.average_samples(get_data_func_dict['x+'])[0]
        x_min = self.average_samples(get_data_func_dict['x-'])[0]
        y_max = self.average_samples(get_data_func_dict['y+'])[1]
        y_min = self.average_samples(get_data_func_dict['y-'])[1]
        z_max = self.average_samples(get_data_func_dict['z+'])[2]
        z_min = self.average_samples(get_data_func_dict['z-'])[2]

        self.offset = [(x_max + x_min) / 2, (y_max + y_min) / 2, (z_max + z_min) / 2]
        self.scale = [(x_max - x_min) / 2, (y_max - y_min) / 2, (z_max - z_min) / 2]

