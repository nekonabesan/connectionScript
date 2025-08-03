import sys
import os
import pytest
import asyncio
import time
from unittest.mock import Mock, patch

sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'modules'))
from RtSensor import RtSensor

def test_find_sensor_port_binary():
    """バイナリモードでのセンサーポート検出テスト"""
    try:
        with RtSensor(RtSensor.BINARY, 57600, 0.01) as sensor:
            port, result = sensor.find_sensor_port_binary()
            print(f"Found sensor on port: {port}")
            print(f"result: {result}")
            print(f"ser: {sensor.ser}")
            assert port is not None
            assert result is not None
            assert sensor.ser is not None
    except ValueError as e:
        pytest.skip(f"センサーが見つかりませんでした: {e}")

def test_get_raw_binary():
    """バイナリモードでの生データ取得テスト"""
    try:
        with RtSensor(RtSensor.BINARY, 57600, 0.01) as sensor:
            result = sensor.get_raw_binary()
            print(f"Raw binary data: {result}")
            
            # データが取得できた場合のテスト
            if result:
                assert len(result) == 7  # acc_x, acc_y, acc_z, velocity_x, velocity_y, velocity_z, temp
                # 加速度データ（0-2）
                assert isinstance(result[0], float)  # acc_x
                assert isinstance(result[1], float)  # acc_y
                assert isinstance(result[2], float)  # acc_z
                # 角速度データ（3-5）
                assert isinstance(result[3], float)  # velocity_x
                assert isinstance(result[4], float)  # velocity_y
                assert isinstance(result[5], float)  # velocity_z
                # 温度データ（6）
                assert isinstance(result[6], float)  # temp
                
                # 妥当な範囲のテスト
                for i in range(3):  # 加速度
                    assert -20.0 <= result[i] <= 20.0, f"加速度が範囲外: {result[i]}"
                for i in range(3, 6):  # 角速度
                    assert -2000.0 <= result[i] <= 2000.0, f"角速度が範囲外: {result[i]}"
                assert -40.0 <= result[6] <= 85.0, f"温度が範囲外: {result[6]}"
            else:
                pytest.skip("センサーからデータを取得できませんでした")
    except ValueError as e:
        pytest.skip(f"センサーが見つかりませんでした: {e}")

@pytest.mark.asyncio
async def test_get_raw_binary_async():
    """非同期バイナリモードでの生データ取得テスト"""
    try:
        with RtSensor(RtSensor.BINARY, 57600, 0.01) as sensor:
            result = await sensor.get_raw_binary_async()
            print(f"Async raw binary data: {result}")
            
            # データが取得できた場合のテスト
            if result:
                assert len(result) == 7  # acc_x, acc_y, acc_z, velocity_x, velocity_y, velocity_z, temp
                # 加速度データ（0-2）
                assert isinstance(result[0], float)  # acc_x
                assert isinstance(result[1], float)  # acc_y
                assert isinstance(result[2], float)  # acc_z
                # 角速度データ（3-5）
                assert isinstance(result[3], float)  # velocity_x
                assert isinstance(result[4], float)  # velocity_y
                assert isinstance(result[5], float)  # velocity_z
                # 温度データ（6）
                assert isinstance(result[6], float)  # temp
                
                # 妥当な範囲のテスト
                for i in range(3):  # 加速度
                    assert -20.0 <= result[i] <= 20.0, f"加速度が範囲外: {result[i]}"
                for i in range(3, 6):  # 角速度
                    assert -2000.0 <= result[i] <= 2000.0, f"角速度が範囲外: {result[i]}"
                assert -40.0 <= result[6] <= 85.0, f"温度が範囲外: {result[6]}"
            else:
                pytest.skip("センサーからデータを取得できませんでした")
    except ValueError as e:
        pytest.skip(f"センサーが見つかりませんでした: {e}")

def test_get_calibrated_data():
    """キャリブレーション済みデータ取得テスト"""
    try:
        with RtSensor(RtSensor.BINARY, 57600, 0.01) as sensor:
            result = sensor.get_calibrated_data()
            print(f"Calibrated data: {result}")
            
            if result:
                # 必須フィールドのテスト
                required_fields = ['acc_x', 'acc_y', 'acc_z', 'velocity_x', 'velocity_y', 'velocity_z', 'temp', 'velocity_x_corrected']
                for field in required_fields:
                    assert field in result, f"必須フィールド {field} がありません"
                    assert isinstance(result[field], float), f"{field} は float である必要があります"
                
                # calibration2実行後に追加されるフィールドのテスト
                optional_fields = ['acc_x_corrected', 'velocity_z_corrected']
                for field in optional_fields:
                    if field in result:
                        assert isinstance(result[field], float), f"{field} は float である必要があります"
                        
            else:
                pytest.skip("キャリブレーション済みデータを取得できませんでした")
    except ValueError as e:
        pytest.skip(f"センサーが見つかりませんでした: {e}")

@pytest.mark.asyncio
async def test_get_calibrated_data_async():
    """非同期キャリブレーション済みデータ取得テスト"""
    try:
        with RtSensor(RtSensor.BINARY, 57600, 0.01) as sensor:
            result = await sensor.get_calibrated_data_async()
            print(f"Async calibrated data: {result}")
            
            if result:
                # 必須フィールドのテスト
                required_fields = ['acc_x', 'acc_y', 'acc_z', 'velocity_x', 'velocity_y', 'velocity_z', 'temp', 'velocity_x_corrected']
                for field in required_fields:
                    assert field in result, f"必須フィールド {field} がありません"
                    assert isinstance(result[field], float), f"{field} は float である必要があります"
                        
            else:
                pytest.skip("非同期キャリブレーション済みデータを取得できませんでした")
    except ValueError as e:
        pytest.skip(f"センサーが見つかりませんでした: {e}")

def test_calibration1():
    """第1回キャリブレーション（ジャイロX軸オフセット計算）テスト"""
    try:
        with RtSensor(RtSensor.BINARY, 57600, 0.01) as sensor:
            # 少ないサンプル数でテスト実行
            gyro_offset = sensor.calibration1(samples=10)
            print(f"Calibration 1 result: {gyro_offset}")
            
            assert isinstance(gyro_offset, float)
            assert -100.0 <= gyro_offset <= 100.0, f"ジャイロオフセットが範囲外: {gyro_offset}"
            
            # オフセットが内部変数に設定されているかテスト
            assert sensor.gyro_offset == gyro_offset
            
    except ValueError as e:
        pytest.skip(f"センサーが見つかりませんでした: {e}")

def test_calibration2():
    """第2回キャリブレーション（加速度X軸・ジャイロZ軸オフセット計算）テスト"""
    try:
        with RtSensor(RtSensor.BINARY, 57600, 0.01) as sensor:
            # 少ないサンプル数でテスト実行
            acc_x_offset, gyro_z_offset = sensor.calibration2(samples=10)
            print(f"Calibration 2 result: acc_x={acc_x_offset}, gyro_z={gyro_z_offset}")
            
            assert isinstance(acc_x_offset, float)
            assert isinstance(gyro_z_offset, float)
            assert -5.0 <= acc_x_offset <= 5.0, f"加速度X軸オフセットが範囲外: {acc_x_offset}"
            assert -100.0 <= gyro_z_offset <= 100.0, f"ジャイロZ軸オフセットが範囲外: {gyro_z_offset}"
            
            # オフセットが内部変数に設定されているかテスト
            assert hasattr(sensor, 'acc_x_offset')
            assert hasattr(sensor, 'gyro_z_offset')
            assert sensor.acc_x_offset == acc_x_offset
            assert sensor.gyro_z_offset == gyro_z_offset
            
    except ValueError as e:
        pytest.skip(f"センサーが見つかりませんでした: {e}")

def test_get_complementary_angle():
    """相補フィルタによる角度計算テスト"""
    try:
        with RtSensor(RtSensor.BINARY, 57600, 0.01) as sensor:
            # 初期角度テスト
            angle1 = sensor.get_complementary_angle(0.01)
            print(f"Initial angle: {angle1}")
            assert isinstance(angle1, float)
            
            # 複数回実行して角度更新をテスト
            angle2 = sensor.get_complementary_angle(0.01)
            angle3 = sensor.get_complementary_angle(0.01)
            print(f"Subsequent angles: {angle2}, {angle3}")
            
            # 角度制限テスト（±85度以内）
            assert -85.0 <= angle1 <= 85.0, f"角度が制限範囲外: {angle1}"
            assert -85.0 <= angle2 <= 85.0, f"角度が制限範囲外: {angle2}"
            assert -85.0 <= angle3 <= 85.0, f"角度が制限範囲外: {angle3}"
            
    except ValueError as e:
        pytest.skip(f"センサーが見つかりませんでした: {e}")

@pytest.mark.asyncio
async def test_get_complementary_angle_async():
    """非同期相補フィルタによる角度計算テスト"""
    try:
        with RtSensor(RtSensor.BINARY, 57600, 0.01) as sensor:
            # 初期角度テスト
            angle1 = await sensor.get_complementary_angle_async(0.01)
            print(f"Async initial angle: {angle1}")
            assert isinstance(angle1, float)
            
            # 複数回実行して角度更新をテスト
            angle2 = await sensor.get_complementary_angle_async(0.01)
            angle3 = await sensor.get_complementary_angle_async(0.01)
            print(f"Async subsequent angles: {angle2}, {angle3}")
            
            # 角度制限テスト（±85度以内）
            assert -85.0 <= angle1 <= 85.0, f"角度が制限範囲外: {angle1}"
            assert -85.0 <= angle2 <= 85.0, f"角度が制限範囲外: {angle2}"
            assert -85.0 <= angle3 <= 85.0, f"角度が制限範囲外: {angle3}"
            
    except ValueError as e:
        pytest.skip(f"センサーが見つかりませんでした: {e}")

def test_check_balance_start_condition():
    """倒立制御開始条件判定テスト"""
    try:
        with RtSensor(RtSensor.BINARY, 57600, 0.01) as sensor:
            result = sensor.check_balance_start_condition()
            print(f"Balance start condition: {result}")
            
            assert isinstance(result, bool)
            # 実際の判定結果は環境によって変わるため、型チェックのみ
            
    except ValueError as e:
        pytest.skip(f"センサーが見つかりませんでした: {e}")

def test_reset_angle():
    """角度リセット機能テスト"""
    try:
        with RtSensor(RtSensor.BINARY, 57600, 0.01) as sensor:
            # まず角度を設定
            sensor.get_complementary_angle(0.01)
            
            # 角度リセット
            sensor.reset_angle()
            
            # リセット後の角度が0.0であることを確認
            if hasattr(sensor, 'angle'):
                assert sensor.angle == 0.0
            
    except ValueError as e:
        pytest.skip(f"センサーが見つかりませんでした: {e}")

def test_get_angle_drift_correction():
    """角度ドリフト補正テスト"""
    try:
        with RtSensor(RtSensor.BINARY, 57600, 0.01) as sensor:
            # まず角度を設定
            sensor.get_complementary_angle(0.01)
            
            # ドリフト補正
            corrected_angle = sensor.get_angle_drift_correction(target_angle=0.0, correction_strength=0.001)
            print(f"Drift corrected angle: {corrected_angle}")
            
            assert isinstance(corrected_angle, float)
            
    except ValueError as e:
        pytest.skip(f"センサーが見つかりませんでした: {e}")

def test_gyro_offset_operations():
    """ジャイロオフセットの設定・取得テスト"""
    try:
        with RtSensor(RtSensor.BINARY, 57600, 0.01) as sensor:
            # 初期値のテスト
            initial_offset = sensor.get_gyro_offset()
            assert initial_offset == 0.0
            
            # オフセット設定のテスト
            test_offset = 1.5
            sensor.set_gyro_offset(test_offset)
            retrieved_offset = sensor.get_gyro_offset()
            assert retrieved_offset == test_offset
            
            print(f"Gyro offset test passed: {retrieved_offset}")
    except ValueError as e:
        pytest.skip(f"センサーが見つかりませんでした: {e}")

def test_get_robot_body_angle_and_speed():
    """ロボット本体の角度と速度取得テスト（レガシーメソッド）"""
    try:
        with RtSensor(RtSensor.BINARY, 57600, 0.01) as sensor:
            velocity_y, gyro_offset = sensor.get_robot_body_angle_and_speed()
            print(f"Velocity Y: {velocity_y}, Gyro offset: {gyro_offset}")
            
            # Decimal型の結果をテスト
            assert velocity_y is not None
            assert gyro_offset is not None
            
            # 妥当な範囲のテスト（Decimalを考慮）
            velocity_y_float = float(velocity_y)
            gyro_offset_float = float(gyro_offset)
            assert -2000.0 <= velocity_y_float <= 2000.0, f"Y軸角速度が範囲外: {velocity_y_float}"
            assert -100.0 <= gyro_offset_float <= 100.0, f"ジャイロオフセットが範囲外: {gyro_offset_float}"
    except ValueError as e:
        pytest.skip(f"センサーが見つかりませんでした: {e}")

def test_ascii_mode_not_implemented():
    """ASCIIモードが未実装であることのテスト"""
    with pytest.raises(NotImplementedError):
        RtSensor(RtSensor.ASCII, 57600, 0.01)

def test_invalid_mode():
    """無効なモード指定のテスト"""
    with pytest.raises(ValueError):
        RtSensor(999, 57600, 0.01)  # 無効なモード

def test_context_manager():
    """コンテキストマネージャーテスト"""
    try:
        with RtSensor(RtSensor.BINARY, 57600, 0.01) as sensor:
            assert sensor.ser is not None
            assert sensor.ser.is_open
        # with文を抜けた後、シリアルポートが閉じられているかテスト
        # （実際の実装によっては確認が困難な場合があります）
    except ValueError as e:
        pytest.skip(f"センサーが見つかりませんでした: {e}")

def test_port_detection():
    """ポート自動検出テスト"""
    try:
        sensor = RtSensor(RtSensor.BINARY, 57600, 0.01)
        port = sensor.get_port()
        print(f"Detected port: {port}")
        assert port is not None
        assert port in RtSensor.PORTS
        sensor.close()
    except ValueError as e:
        pytest.skip(f"センサーが見つかりませんでした: {e}")

def test_list_serial_ports():
    """シリアルポート一覧取得テスト"""
    try:
        sensor = RtSensor(RtSensor.BINARY, 57600, 0.01)
        ports = sensor.list_serial_ports()
        print(f"Available ports: {ports}")
        assert isinstance(ports, list)
        sensor.close()
    except ValueError as e:
        pytest.skip(f"センサーが見つかりませんでした: {e}")

# データキャッシュ機能のテスト
def test_data_caching():
    """データキャッシュ機能テスト"""
    try:
        with RtSensor(RtSensor.BINARY, 57600, 0.01) as sensor:
            # 初回データ取得
            data1 = sensor.get_raw_binary()
            
            # キャッシュされたデータが存在することを確認
            assert sensor.last_sensor_data is not None
            assert sensor.last_read_time > 0.0
            
            # 2回目のデータ取得
            data2 = sensor.get_raw_binary()
            
            # データが取得できていることを確認
            if data1 and data2:
                assert len(data1) == 7
                assert len(data2) == 7
                
    except ValueError as e:
        pytest.skip(f"センサーが見つかりませんでした: {e}")

# エラーハンドリングテスト
def test_error_handling():
    """エラーハンドリングテスト"""
    try:
        with RtSensor(RtSensor.BINARY, 57600, 0.01) as sensor:
            # センサーを強制的に閉じる
            sensor.ser.close()
            
            # 閉じられたセンサーからのデータ取得時のエラーハンドリング
            with pytest.raises(ValueError):
                sensor.get_raw_binary()
                
    except ValueError as e:
        pytest.skip(f"センサーが見つかりませんでした: {e}")

# 統合テスト
def test_full_workflow():
    """完全なワークフローテスト"""
    try:
        with RtSensor(RtSensor.BINARY, 57600, 0.01) as sensor:
            print("=== 完全ワークフローテスト開始 ===")
            
            # 1. 第1回キャリブレーション
            gyro_offset = sensor.calibration1(samples=5)
            print(f"Step 1 - Gyro offset: {gyro_offset}")
            
            # 2. 第2回キャリブレーション
            acc_x_offset, gyro_z_offset = sensor.calibration2(samples=5)
            print(f"Step 2 - Acc X offset: {acc_x_offset}, Gyro Z offset: {gyro_z_offset}")
            
            # 3. キャリブレーション済みデータ取得
            calibrated_data = sensor.get_calibrated_data()
            print(f"Step 3 - Calibrated data keys: {list(calibrated_data.keys()) if calibrated_data else 'None'}")
            
            # 4. 相補フィルタによる角度計算
            angle = sensor.get_complementary_angle(0.016)  # 16ms制御周期
            print(f"Step 4 - Complementary filter angle: {angle}")
            
            # 5. 倒立制御開始条件判定
            can_start = sensor.check_balance_start_condition()
            print(f"Step 5 - Can start balance control: {can_start}")
            
            print("=== 完全ワークフローテスト完了 ===")
            
    except ValueError as e:
        pytest.skip(f"センサーが見つかりませんでした: {e}")

@pytest.mark.asyncio
async def test_async_workflow():
    """非同期ワークフローテスト"""
    try:
        with RtSensor(RtSensor.BINARY, 57600, 0.01) as sensor:
            print("=== 非同期ワークフローテスト開始 ===")
            
            # 1. 非同期生データ取得
            raw_data = await sensor.get_raw_binary_async()
            print(f"Step 1 - Async raw data: {len(raw_data) if raw_data else 0} values")
            
            # 2. 非同期キャリブレーション済みデータ取得
            calibrated_data = await sensor.get_calibrated_data_async()
            print(f"Step 2 - Async calibrated data keys: {list(calibrated_data.keys()) if calibrated_data else 'None'}")
            
            # 3. 非同期相補フィルタによる角度計算
            angle = await sensor.get_complementary_angle_async(0.016)
            print(f"Step 3 - Async complementary filter angle: {angle}")
            
            print("=== 非同期ワークフローテスト完了 ===")
            
    except ValueError as e:
        pytest.skip(f"センサーが見つかりませんでした: {e}")