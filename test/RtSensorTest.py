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
    sensor = RtSensor(1, 115200, 0.01)
    port, result = sensor.find_sensor_port_binary()
    print(f"Found sensor on port: {port}")
    print(f"result: {result}")
    print(f"ser: {sensor.ser}")
    assert port is not None
    assert result is not None
    assert sensor.ser is not None

def test_get_raw_binary():
    sensor = RtSensor(1, 115200, 0.01)
    result = sensor.get_raw_binary()
    print(result)
    assert len(result) > 0
    assert isinstance(result[0], float)
    assert isinstance(result[1], float)
    assert isinstance(result[2], float)
    assert isinstance(result[3], float)
    assert isinstance(result[4], float)
    assert isinstance(result[5], float)
    assert isinstance(result[6], float)
"""
def test_get_sensor_values_data():
    sensor = RtSensor(1, 115200, 0.01)
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
    sensor = RtSensor(0, 115200, 0.01)
    result = sensor.get_raw_data()
    print(result)
    assert isinstance(result, list)
    assert len(result) == 6
    for item in result:
        assert isinstance(item, float)

def test_calibrate_static_orientation():
    sensor = RtSensor(0, 115200, 0.01)
    sensor.calibrate_static_orientation()
    raw_data = sensor.get_raw_data()
    calibrated_data = sensor.apply_calibration(raw_data)
    print(calibrated_data)
    assert isinstance(calibrated_data, list)
    assert len(calibrated_data) == 6
    for item in calibrated_data:
        assert isinstance(item, float)

"""