#!/usr/bin/python
import serial
import serial.tools.list_ports
import time
import struct
from decimal import *
from serial_asyncio import open_serial_connection
import smbus2 as smbus


class RtSensor():
    # モード定数
    ASCII = 1
    BINARY = 2
    
    EXPECTED_LENGTH = 11
    BIN_EXPECTED_LENGTH = 28
    PORTS = ["/dev/serial0", "/dev/ttyAMA0", "/dev/ttyS0"]
    
    # バイナリモードのヘッダー（aa.pyに合わせて修正）
    SYNC_HEADER = b'\xff\xffRT9A'

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

    def __init__(self, mode = 1, rate = 57600, time_out_sec = 0.01):
        """
        RtSensorのコンストラクタ
        Parameters:
            mode (int): センサのモード（1: ASCII, 2: Binary）
            rate (int): シリアル通信のボーレート
            time_out_sec (float): タイムアウト秒数（高速化のため0.01秒に短縮）
        Raises:
            ValueError: 無効なモードが指定された場合
        """
        self.offset = [0.0, 0.0, 0.0]
        self.scale = [1.0, 1.0, 1.0]
        self.rate = rate
        self.time_out_sec = time_out_sec
        self.gyro_offset = 0.0
        self.last_sensor_data = None  # 前回のセンサーデータをキャッシュ
        self.last_read_time = 0.0     # 前回の読み取り時刻
        
        if mode == self.ASCII:
            # ASCIIモードは現在未実装
            raise NotImplementedError("ASCII mode is not implemented yet")
        elif mode == self.BINARY:
            self.port, _ = self.find_sensor_port_binary()
            if self.port is None:
                raise ValueError("センサのポートが見つかりませんでした")
        else:
            raise ValueError(f"無効なモード: {mode}")
        
        self.ser = serial.Serial(self.port, self.rate, timeout = self.time_out_sec)

    async def initialize_binary(self, baudrate=57600):
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

    def try_connect_binary(self, port, rate=57600, time_out_sec=0.5):
        """
        シリアルポートに接続を試みる
        先頭6バイトがヘッダで、残り22バイト
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
            # バッファをクリア
            ser.reset_input_buffer()
            time.sleep(0.1)  # センサーの安定化を待つ
            
            # aa.pyと同じ同期方法を使用
            buffer = b''
            max_attempts = 100
            attempts = 0
            
            while attempts < max_attempts:
                byte = ser.read(1)
                if not byte:
                    attempts += 1
                    continue
                    
                buffer += byte
                attempts += 1
                
                # バッファが28バイト以上なら判定
                if len(buffer) >= 28:
                    if buffer[:6] == self.SYNC_HEADER:
                        packet = buffer[:28]
                        ser.close()
                        print(f"Port {port}: Valid packet found: {packet[:6].hex()}")
                        return packet
                    else:
                        # 1バイトずつずらして再同期
                        buffer = buffer[1:]
            
            ser.close()
            print(f"Port {port}: No valid header found after {max_attempts} attempts")
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
        result = self.get_raw_binary()
        if not result:
            return 0.0, self.gyro_offset
            
        gyro_offset = self.get_gyro_offset()
        velocity_y = Decimal(str(result[4]))  # Y軸角速度
        gyro_offset = (Decimal(str(0.0005)) * Decimal(str(velocity_y))) + (Decimal(str(1 - 0.0005)) * Decimal(str(gyro_offset)))
        self.gyro_offset = gyro_offset
        velocity_y = velocity_y - gyro_offset
        return velocity_y, gyro_offset

    async def get_raw_binary_async(self):
        """
        Binaryモードで28バイトのセンサデータを非同期取得
        Returns:
            list: センサの生データ（加速度、角速度、温度など）
        Raises:
            ValueError: シリアルポートが設定されていない場合    
        """
        if not self.ser or not self.ser.is_open:
            raise ValueError("シリアルポートが開かれていません")
        
        try:
            # 非ブロッキングでデータをチェック
            available = self.ser.in_waiting
            
            if available >= 28:
                # 十分なデータがある場合は即座に読み取り
                data = self.ser.read(available)
                
                # 最新のパケットを探す（後ろから検索して最新データを取得）
                for i in range(len(data) - 28, -1, -1):
                    if i + 6 <= len(data) and data[i:i+6] == self.SYNC_HEADER:
                        packet = data[i:i+28]
                        
                        # aa.pyと同じデータ解析方法を使用
                        acc = struct.unpack_from('<hhh', packet, 8)
                        gyro = struct.unpack_from('<hhh', packet, 16)
                        mag = struct.unpack_from('<hhh', packet, 22)
                        temp_raw = struct.unpack_from('<h', packet, 14)[0]

                        # 物理量への変換
                        acc_g = [x / 2048.0 for x in acc]
                        gyro_dps = [x / 16.4 for x in gyro]
                        mag_uT = [x * 0.3 for x in mag]
                        temp_c = temp_raw / 340.0 + 35.0

                        # データをキャッシュ
                        sensor_data = [acc_g[0], acc_g[1], acc_g[2], gyro_dps[0], gyro_dps[1], gyro_dps[2], temp_c]
                        self.last_sensor_data = sensor_data
                        self.last_read_time = time.time()
                        
                        return sensor_data
            
            # データが不足している場合は前回のデータを返す
            if self.last_sensor_data is not None:
                return self.last_sensor_data
            
        except Exception as e:
            print(f"非同期センサー読み取りエラー: {e}")
            if self.last_sensor_data is not None:
                return self.last_sensor_data
        
        # 初回やエラー時でキャッシュもない場合は空を返す
        return []

    def get_raw_binary(self):
        """
        同期版：Binaryモードで28バイトのセンサデータを取得
        """
        if not self.ser or not self.ser.is_open:
            raise ValueError("シリアルポートが開かれていません")
        
        try:
            available = self.ser.in_waiting
            if available >= 28:
                data = self.ser.read(available)
                for i in range(len(data) - 28, -1, -1):
                    if i + 6 <= len(data) and data[i:i+6] == self.SYNC_HEADER:
                        packet = data[i:i+28]
                        acc = struct.unpack_from('<hhh', packet, 8)
                        gyro = struct.unpack_from('<hhh', packet, 16)
                        mag = struct.unpack_from('<hhh', packet, 22)
                        temp_raw = struct.unpack_from('<h', packet, 14)[0]
                        acc_g = [x / 2048.0 for x in acc]
                        gyro_dps = [x / 16.4 for x in gyro]
                        mag_uT = [x * 0.3 for x in mag]
                        temp_c = temp_raw / 340.0 + 35.0
                        sensor_data = [acc_g[0], acc_g[1], acc_g[2], gyro_dps[0], gyro_dps[1], gyro_dps[2], temp_c]
                        self.last_sensor_data = sensor_data
                        return sensor_data
            if self.last_sensor_data is not None:
                return self.last_sensor_data
        except Exception as e:
            if self.last_sensor_data is not None:
                return self.last_sensor_data
        return []

    async def get_calibrated_data_async(self):
        """
        非同期版：キャリブレーション済みのセンサーデータを取得
        """
        raw_data = await self.get_raw_binary_async()
        if not raw_data or len(raw_data) < 7:
            return None
            
        acc_x, acc_y, acc_z, velocity_x, velocity_y, velocity_z, temp = raw_data
        
        result = {
            'acc_x': acc_x,
            'acc_y': acc_y,
            'acc_z': acc_z,
            'velocity_x': velocity_x,
            'velocity_y': velocity_y,
            'velocity_z': velocity_z,
            'temp': temp,
            'velocity_x_corrected': velocity_x - (self.gyro_offset or 0.0)  # X軸角速度（倒立振子用）
        }
        
        if hasattr(self, 'acc_x_offset'):
            result['acc_x_corrected'] = acc_x - self.acc_x_offset
        if hasattr(self, 'gyro_z_offset'):
            result['velocity_z_corrected'] = velocity_z - self.gyro_z_offset
            
        return result

    async def get_complementary_angle_async(self, dt, cutoff_freq=0.04):
        """
        非同期版：シンプルな相補フィルタによる角度計算
        X軸加速度とジャイロのみを使用
        """
        if not hasattr(self, 'angle'):
            self.angle = 0.0
            
        calibrated_data = await self.get_calibrated_data_async()
        if not calibrated_data:
            return self.angle
            
        velocity_x = calibrated_data['velocity_x_corrected']  # X軸角速度（倒立振子用）
        
        if 'acc_x_corrected' in calibrated_data:
            acc_x = calibrated_data['acc_x_corrected']
        else:
            acc_x = calibrated_data['acc_x']
        
        # 加速度から角度を直接計算（度）
        accel_angle = acc_x * 90.0  # 1Gを90度として換算
        
        # 相補フィルタ：ジャイロ積分 + 加速度補正
        gyro_angle = self.angle + velocity_x * dt  # ジャイロによる角度積分
        
        # 重み付き相補フィルタ
        alpha = 1.0 - cutoff_freq
        self.angle = alpha * gyro_angle + cutoff_freq * accel_angle
        
        # 角度制限（±85度以内）
        if self.angle > 85.0:
            self.angle = 85.0
        elif self.angle < -85.0:
            self.angle = -85.0
            
        return self.angle

    def calibration1(self, samples=100):
        """
        第1回キャリブレーション（起動時）
        ジャイロX軸のオフセット計算（倒立振子の前後傾斜用）
        
        Parameters:
            samples (int): キャリブレーション用サンプル数
        Returns:
            float: 計算されたジャイロX軸オフセット値
        """
        print("Calibration 1 開始 - ジャイロX軸オフセット計算...")
        time.sleep(0.5)  # センサー安定化待機
        
        gyro_sum = 0.0
        valid_samples = 0
        
        for i in range(samples):
            sensor_data = self.get_raw_binary()
            if sensor_data and len(sensor_data) >= 7:
                velocity_x = sensor_data[3]  # X軸角速度（倒立振子の前後傾斜）
                gyro_sum += velocity_x
                valid_samples += 1
            # 高速キャリブレーション：5ms間隔
            time.sleep(0.005)
            
            if (i + 1) % 20 == 0:
                print(f"進行状況: {i + 1}/{samples}")
                
        if valid_samples > 0:
            self.gyro_offset = gyro_sum / valid_samples
            print(f"Gyro X offset: {self.gyro_offset:.6f}")
            print("Calibration 1 完了")
            return self.gyro_offset
        else:
            print("Calibration 1 失敗 - 有効なデータが取得できませんでした")
            return 0.0

    def calibration2(self, samples=100):
        """
        第2回キャリブレーション（倒立開始前）
        加速度X軸とジャイロZ軸のオフセット計算
        
        Parameters:
            samples (int): キャリブレーション用サンプル数
        Returns:
            tuple: (加速度X軸オフセット, ジャイロZ軸オフセット)
        """
        print("Calibration 2 開始 - 加速度X軸・ジャイロZ軸オフセット計算...")
        time.sleep(0.3)  # センサー安定化待機
        
        acc_x_sum = 0.0
        gyro_z_sum = 0.0
        valid_samples = 0
        
        for i in range(samples):
            sensor_data = self.get_raw_binary()
            if sensor_data and len(sensor_data) >= 7:
                acc_x = sensor_data[0]      # X軸加速度
                velocity_z = sensor_data[5] # Z軸角速度
                acc_x_sum += acc_x
                gyro_z_sum += velocity_z
                valid_samples += 1
            # 高速キャリブレーション：5ms間隔
            time.sleep(0.005)
            
            if (i + 1) % 20 == 0:
                print(f"進行状況: {i + 1}/{samples}")
                
        if valid_samples > 0:
            acc_x_offset = acc_x_sum / valid_samples
            gyro_z_offset = gyro_z_sum / valid_samples
            
            # オフセット値を内部変数として保存
            self.acc_x_offset = acc_x_offset
            self.gyro_z_offset = gyro_z_offset
            
            print(f"Acc X offset: {acc_x_offset:.6f}")
            print(f"Gyro Z offset: {gyro_z_offset:.6f}")
            print("Calibration 2 完了")
            return acc_x_offset, gyro_z_offset
        else:
            print("Calibration 2 失敗 - 有効なデータが取得できませんでした")
            return 0.0, 0.0

    def get_calibrated_data(self):
        """
        キャリブレーション済みのセンサーデータを取得
        
        Returns:
            dict: キャリブレーション済みセンサーデータ
                - acc_x, acc_y, acc_z: 加速度（m/s²）
                - velocity_x, velocity_y, velocity_z: 角速度（°/s）
                - temp: 温度（°C）
                - velocity_y_corrected: Y軸角速度（オフセット補正済み）
                - acc_x_corrected: X軸加速度（オフセット補正済み、calibration2実行後のみ）
                - velocity_z_corrected: Z軸角速度（オフセット補正済み、calibration2実行後のみ）
        """
        raw_data = self.get_raw_binary()
        if not raw_data or len(raw_data) < 7:
            return None
            
        acc_x, acc_y, acc_z, velocity_x, velocity_y, velocity_z, temp = raw_data
        
        result = {
            'acc_x': acc_x,
            'acc_y': acc_y,
            'acc_z': acc_z,
            'velocity_x': velocity_x,
            'velocity_y': velocity_y,
            'velocity_z': velocity_z,
            'temp': temp,
            'velocity_x_corrected': velocity_x - (self.gyro_offset or 0.0)  # X軸角速度（倒立振子用）
        }
        
        # calibration2が実行されている場合は補正値も追加
        if hasattr(self, 'acc_x_offset'):
            result['acc_x_corrected'] = acc_x - self.acc_x_offset
        if hasattr(self, 'gyro_z_offset'):
            result['velocity_z_corrected'] = velocity_z - self.gyro_z_offset
            
        return result

    def get_complementary_angle(self, dt, cutoff_freq=0.04):
        """
        シンプルな相補フィルタによる角度計算
        X軸加速度とジャイロのみを使用（Z軸判定なし）
        
        Parameters:
            dt (float): 制御周期（秒）
            cutoff_freq (float): カットオフ周波数
        Returns:
            float: 相補フィルタによる角度（度）
        """
        if not hasattr(self, 'angle'):
            self.angle = 0.0
            
        calibrated_data = self.get_calibrated_data()
        if not calibrated_data:
            return self.angle
            
        velocity_x = calibrated_data['velocity_x_corrected']  # X軸角速度（倒立振子用）
        
        # calibration2が実行されている場合は補正済み加速度を使用
        if 'acc_x_corrected' in calibrated_data:
            acc_x = calibrated_data['acc_x_corrected']
        else:
            acc_x = calibrated_data['acc_x']
        
        # 加速度から角度を直接計算（度）
        accel_angle = acc_x * 90.0  # 1Gを90度として換算
        
        # 相補フィルタ：ジャイロ積分 + 加速度補正
        gyro_angle = self.angle + velocity_x * dt  # ジャイロによる角度積分
        
        # 重み付き相補フィルタ
        alpha = 1.0 - cutoff_freq
        self.angle = alpha * gyro_angle + cutoff_freq * accel_angle
        
        # 角度制限（±85度以内）
        if self.angle > 85.0:
            self.angle = 85.0
        elif self.angle < -85.0:
            self.angle = -85.0
            
        return self.angle

    def reset_angle(self):
        """
        角度積分値をリセット
        ドリフト補正のため定期的なリセット機能追加
        """
        if hasattr(self, 'angle'):
            self.angle = 0.0
        if hasattr(self, 'angle_initialized'):
            self.angle_initialized = False
        print("角度リセット完了")

    def get_angle_drift_correction(self, target_angle=0.0, correction_strength=0.001):
        """
        角度ドリフト補正
        長期間の運用でのドリフトを補正
        
        Parameters:
            target_angle (float): 目標角度（通常は0度）
            correction_strength (float): 補正強度
        Returns:
            float: 補正後の角度
        """
        if hasattr(self, 'angle'):
            drift_error = target_angle - self.angle
            self.angle += drift_error * correction_strength
            return self.angle
        return 0.0

    def check_balance_start_condition(self, threshold=0.9):
        """
        簡単な倒立制御開始条件の判定
        基本的な傾斜角のみで判定（Z軸複雑性を除去）
        """
        calibrated_data = self.get_calibrated_data()
        if not calibrated_data:
            return False
            
        # 現在の傾斜角を取得
        angle = self.get_complementary_angle(0.01)
        
        # 角度が安定範囲内（±10度）であれば制御開始可能
        return abs(angle) < 10.0

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
        if self.writer:
            self.writer.close()

    def __del__(self):
        """
        デストラクタ - リソースのクリーンアップ
        """
        self.close()

    def __enter__(self):
        """
        コンテキストマネージャのエントリ
        """
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """
        コンテキストマネージャの終了
        """
        self.close()

