#!/usr/bin/python
import serial
import serial.tools.list_ports
import time
from decimal import *
from serial_asyncio import open_serial_connection


class RtSensor():
    EXPECTED_LENGTH = 11
    BIN_EXPECTED_LENGTH = 28
    ASCII = 0
    BINARY = 1
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
    def __init__(self, mode = 1, rate = 115200, time_out_sec = 0.5):
        self.offset = [0.0, 0.0, 0.0]
        self.scale = [1.0, 1.0, 1.0]
        self.rate = rate
        self.time_out_sec = time_out_sec
        self.ser = serial.Serial(self.port, self.rate, timeout = self.time_out_sec)
        if mode == self.ASCII:
            self.port = self.find_sensor_port()
            self.calibrate_static_orientation()
        elif mode == self.BINARY:
            self.port, _ = self.find_sensor_port_binary()
        

    # /--------------------------------------------------------------/ #
    # センサの初期化
    # ASCII 出力時通信プロトコル
    # /--------------------------------------------------------------/ #
    async def initialize(self , baudrate=115200):
        self.reader, self.writer = await open_serial_connection(url=self.port, baudrate=baudrate)

    # /--------------------------------------------------------------/ #
    # センサの初期化
    # Binary 出力時通信プロトコル
    # /--------------------------------------------------------------/ #
    async def initialize_binary(self, baudrate=115200):
        self.reader, self.writer = await open_serial_connection(url=self.port, baudrate=baudrate)
        #self.sync_header = b'\xFF\xFF\x52\x54'  # Binaryモードのヘッダー


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
    # ASCII 出力時通信プロトコル
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
    # シリアルポートに接続を試みる
    # Binary 出力時通信プロトコル
    # 先頭4バイトがヘッダで、残り24バイト
    # /--------------------------------------------------------------/ #
    def try_connect_binary(self, port, rate=115200, time_out_sec=0.5):
        try:
            ser = serial.Serial(port, rate, timeout=time_out_sec)
            sync_bytes = ser.read(4)  # 先頭4バイトを同期確認
            if len(sync_bytes) == 4 and sync_bytes == b'\xFF\xFF\x52\x54':
                payload = ser.read(24)  # 残りのバイナリデータ
                ser.close()
                if len(payload) == 24:
                    return sync_bytes + payload  # 28バイトの完全なパケット
            else:
                ser.close()
                print(f"Port {port}: Header mismatch or insufficient data.")
        except Exception as e:
            print(f"Port {port} failed: {e}")
        return None

    # /--------------------------------------------------------------/ #
    # センサのポートを自動検出
    # ASCII 出力時通信プロトコル
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
    # センサのポートを自動検出
    # Binary 出力時通信プロトコル
    # /--------------------------------------------------------------/ #
    def find_sensor_port_binary(self):
        for port in self.PORTS:
            print(f"Trying port: {port}")
            result = self.try_connect_binary(port)
            if result is not None:
                print(f"Sensor found on {port}")
                return port, result
        print("Sensor not found on listed ports.")
        return None, None

    # /--------------------------------------------------------------/ #
    # センサの生データから角速度,角加速度を取得
    # ASCII 出力時通信プロトコル
    # /--------------------------------------------------------------/ #
    def get_raw_data(self):
        raw_line = self.get_sensor_values()
        parsed = self.parse_sensor_values(raw_line)
        return [parsed['velocity_x'], parsed['velocity_y'], parsed['velocity_z'], parsed["accel_x"], parsed["accel_y"], parsed["accel_z"]]

    # /--------------------------------------------------------------/ #
    # センサの生データを取得し、校正値を適用
    # ASCII 出力時通信プロトコル
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
    # ASCII 出力時通信プロトコル
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
    # ASCII 出力時通信プロトコル
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
    # ASCII 出力時通信プロトコル
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
    # ASCII 出力時通信プロトコル
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
    # ASCII 出力時通信プロトコル
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

    # /--------------------------------------------------------------/ #
    # Binaryモードで28バイトのセンサデータを取得
    # /--------------------------------------------------------------/ #
    def get_raw_binary(self):
        buffer = self.ser.read(self.BIN_EXPECTED_LENGTH)
        if len(buffer) != self.BIN_EXPECTED_LENGTH:
            while len(buffer) != self.BIN_EXPECTED_LENGTH:
                print(buffer)
                self.ser.reset_input_buffer()
                buffer = self.get_raw_binary()
        if not buffer:
            print("センサからデータが取得できませんでした")
            return []

        # バイナリデータの構造に従ってパース（例: 8bit x 2 → 16bit）
        def to_signed_16bit(low, high):
            value = low + (high << 8)
            return value - 65536 if value >= 32768 else value

        # 各センサ値の抽出（例: 加速度、角速度、地磁気、温度）
        acc_x = to_signed_16bit(buffer[8], buffer[9]) / 2048.0
        acc_y = to_signed_16bit(buffer[10], buffer[11]) / 2048.0
        acc_z = to_signed_16bit(buffer[12], buffer[13]) / 2048.0

        velocity_x = to_signed_16bit(buffer[16], buffer[17]) / 16.4
        velocity_y = to_signed_16bit(buffer[18], buffer[19]) / 16.4
        velocity_z = to_signed_16bit(buffer[20], buffer[21]) / 16.4

        temp = to_signed_16bit(buffer[14], buffer[15]) / 333.87 + 21.0

        return [acc_x, acc_y, acc_z, velocity_x, velocity_y, velocity_z, temp]

