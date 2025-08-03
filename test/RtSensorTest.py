import sys
import os
import pytest

sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'modules'))
from RtSensor import RtSensor 

def cast_numbers(arr):
    casted = []
    for item in arr:
        try:
            # 整数かつ少数点がない場合は int に
            if '.' not in item:
                casted.append(int(item))
            else:
                casted.append(float(item))
        except ValueError:
            casted.append(item)  # 数値化できない場合はそのまま保持
    return casted

def test_find_sensor_port_binary():
    """バイナリモードでのセンサーポート検出テスト"""
    try:
        with RtSensor(RtSensor.BINARY, 115200, 0.01) as sensor:
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
        with RtSensor(RtSensor.BINARY, 115200, 0.01) as sensor:
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
                pytest.fail("センサーからデータを取得できませんでした")
    except ValueError as e:
        pytest.skip(f"センサーが見つかりませんでした: {e}")

def test_get_robot_body_angle_and_speed():
    """ロボット本体の角度と速度取得テスト"""
    try:
        with RtSensor(RtSensor.BINARY, 115200, 0.01) as sensor:
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

def test_gyro_offset_operations():
    """ジャイロオフセットの設定・取得テスト"""
    try:
        with RtSensor(RtSensor.BINARY, 115200, 0.01) as sensor:
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

def test_ascii_mode_not_implemented():
    """ASCIIモードが未実装であることのテスト"""
    with pytest.raises(NotImplementedError):
        RtSensor(RtSensor.ASCII, 115200, 0.01)

def test_invalid_mode():
    """無効なモード指定のテスト"""
    with pytest.raises(ValueError):
        RtSensor(999, 115200, 0.01)  # 無効なモード

# 以下は未実装メソッドのため、将来的に実装される場合のためにコメントアウト
"""
def test_get_sensor_values_data():
    # このメソッドはASCIIモード用で現在未実装
    sensor = RtSensor(RtSensor.ASCII, 115200, 0.01)
    print(sensor)
    result = cast_numbers(sensor.get_sensor_values())
    print(result)
    assert isinstance(result, list)
    # 正常にデータが取得できた時は以下の配列が取得できる
    # ['6', '0.005321', '-0.063854', '0.018092', '0.005371', '-0.002930', '-1.025391', '65.550003', '12.450000', '25.500000', '38.800343']
    assert isinstance(result[0], int)
    assert isinstance(result[1], float)
    assert isinstance(result[2], float)
    assert isinstance(result[3], float)
    assert isinstance(result[4], float)
    assert isinstance(result[5], float)
    assert isinstance(result[6], float)
    assert isinstance(result[7], float)
    assert isinstance(result[8], float)
    assert isinstance(result[9], float)
    assert isinstance(result[10], float)

def test_get_raw_data():
    # このメソッドは未実装
    sensor = RtSensor(RtSensor.ASCII, 115200, 0.01)
    result = sensor.get_raw_data()
    print(result)
    assert isinstance(result, list)
    assert len(result) == 6
    for item in result:
        assert isinstance(item, float)

def test_calibrate_static_orientation():
    # このメソッドは未実装
    sensor = RtSensor(RtSensor.ASCII, 115200, 0.01)
    sensor.calibrate_static_orientation()
    raw_data = sensor.get_raw_data()
    calibrated_data = sensor.apply_calibration(raw_data)
    print(calibrated_data)
    assert isinstance(calibrated_data, list)
    assert len(calibrated_data) == 6
    for item in calibrated_data:
        assert isinstance(item, float)
"""