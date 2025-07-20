#!/usr/bin/python
import serial
import serial.tools.list_ports
import time
from decimal import *
from serial_asyncio import open_serial_connection


class RtSensor():
    EXPECTED_LENGTH = 11
    BIN_EXPECTED_LENGTH = 28
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

    def get_gyro_offset(self):
        """
        センサの角速度オフセットを取得
        Returns:
            float: センサの角速度オフセット
        Raises:
            ValueError: センサの角速度オフセットが設定されていない場合
        """
        return self.gyro_offset

    def set_gyro_offset(self, gyro_offset):
        """
        センサの角速度オフセットを設定
        Parameters:
            gyro_offset (float): センサの角速度オフセット
        Raises:
            ValueError: センサの角速度オフセットが無効な場合
        """
        self.gyro_offset = gyro_offset

    def get_port(self):
        """
        センサのポートを取得
        Returns:
            str: センサのポート
        Raises:
            ValueError: センサのポートが設定されていない場合
        """
        return self.port

    def __init__(self, mode = 1, rate = 115200, time_out_sec = 0.5):
        """
        RtSensorのコンストラクタ
        Parameters:
            mode (int): センサのモード（1: ASCII, 2: Binary）
            rate (int): シリアル通信のボーレート
            time_out_sec (float): タイムアウト秒数
        Raises:
            ValueError: 無効なモードが指定された場合
        """
        self.offset = [0.0, 0.0, 0.0]
        self.scale = [1.0, 1.0, 1.0]
        self.rate = rate
        self.time_out_sec = time_out_sec
        if mode == self.ASCII:
            self.port = self.find_sensor_port()
            self.calibrate_static_orientation()
        elif mode == self.BINARY:
            self.port, _ = self.find_sensor_port_binary()
        self.ser = serial.Serial(self.port, self.rate, timeout = self.time_out_sec)

    async def initialize_binary(self, baudrate=115200):
        """
        センサの初期化
        Parameters:
            baudrate (int): シリアル通信のボーレート
        Raises:
            ValueError: センサのポートが設定されていない場合
        """
        self.reader, self.writer = await open_serial_connection(url=self.port, baudrate=baudrate)
        #self.sync_header = b'\xFF\xFF\x52\x54'  # Binaryモードのヘッダー

    async def get_raw_values(self):
        """
        センサの生データを非同期で読み取る
        Returns:
            list: センサの生データ（加速度、角速度など）
        Raises:
            ValueError: センサのポートが設定されていない場合
        """
        line = await self.reader.readline()
        values = line.decode().strip().split(',')
        if len(values) < 7:
            return None
        accel = [float(values[4]), float(values[5]), float(values[6])]
        gyro = [float(values[1]), float(values[2]), float(values[3])]
        return accel + gyro

    def list_serial_ports(self):
        """
        シリアルポートの一覧を取得
        Returns:
            list: シリアルポートの一覧
        """
        ports = serial.tools.list_ports.comports()
        return [port.device for port in ports]

    def try_connect_binary(self, port, rate=115200, time_out_sec=0.5):
        """
        シリアルポートに接続を試みる
        先頭4バイトがヘッダで、残り24バイト
        Parameters:
            port (str): シリアルポートの名前
            rate (int): シリアル通信のボーレート
            time_out_sec (float): タイムアウト秒数
        Returns:
            bytes: 接続成功時のデータ（ヘッダとペイロード）
        Raises:
            Exception: 接続に失敗した場合
            ValueError: 無効なポートが指定された場合
        """
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

    def find_sensor_port_binary(self):
        """
        センサのポートを自動検出
        Returns:
            tuple: センサのポートと接続結果（ポート名、接続結果）
        Raises:
            ValueError: センサのポートが見つからない場合
        """
        for port in self.PORTS:
            print(f"Trying port: {port}")
            result = self.try_connect_binary(port)
            if result is not None:
                print(f"Sensor found on {port}")
                return port, result
        print("Sensor not found on listed ports.")
        return None, None

    def get_robot_body_angle_and_speed(self):
        """
        センサから角速度と角度を取得
        Returns:
            tuple: 角速度と角度（角速度, 角度）
        Raises:
            ValueError: センサのポートが設定されていない場合
        """
        result = self.get_sensor_values()
        gyro_offset = self.get_gyro_offset()
        velosity_y =  Decimal(str(result[2]))
        gyro_offset = (Decimal(str(0.0005)) * Decimal(str(velosity_y))) + (Decimal(str(1 - 0.0005)) * Decimal(str(gyro_offset)))
        self.gyro_offset = gyro_offset
        velosity_y = velosity_y - gyro_offset
        return velosity_y, gyro_offset

    def get_raw_binary(self):
        """
        Binaryモードで28バイトのセンサデータを取得
        Returns:
            list: センサの生データ（加速度、角速度、温度など）
        Raises:
            ValueError: センサのポートが設定されていない場合    
        """
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

