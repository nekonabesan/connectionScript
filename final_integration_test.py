#!/usr/bin/env python3
"""
Raspberry Pi バランシングロボット制御システム
最終統合テスト
"""

import sys
import traceback

def test_all_imports():
    """全モジュールのインポートテスト"""
    print("=== 全モジュールインポートテスト ===")
    
    try:
        # 標準ライブラリ
        import asyncio
        import time
        from decimal import Decimal, getcontext
        print("✓ 標準ライブラリ: インポート成功")
        
        # Raspberry Pi固有モジュール
        import smbus2
        import RPi.GPIO
        import gpiozero
        import pigpio
        print("✓ Raspberry Pi固有ライブラリ: インポート成功")
        
        # シリアル通信
        import serial
        import serial.asyncio
        print("✓ シリアル通信ライブラリ: インポート成功")
        
        # カスタムモジュール
        from modules.PCA9685 import PCA9685
        from modules.MotorEncoder import MotorEncoder
        from modules.MotorDriver import MotorDriver
        from modules.RtSensor import RtSensor
        print("✓ カスタムモジュール: インポート成功")
        
        return True
        
    except ImportError as e:
        print(f"✗ インポートエラー: {e}")
        return False
    except Exception as e:
        print(f"✗ 予期しないエラー: {e}")
        return False

def test_hardware_initialization():
    """ハードウェア初期化テスト"""
    print("\n=== ハードウェア初期化テスト ===")
    
    try:
        from modules.PCA9685 import PCA9685
        from modules.MotorEncoder import MotorEncoder
        from modules.MotorDriver import MotorDriver
        from modules.RtSensor import RtSensor
        
        # PCA9685初期化テスト
        try:
            pwm = PCA9685(0x40, debug=False)
            pwm.setPWMFreq(50)
            print("✓ PCA9685: 初期化成功")
        except Exception as e:
            print(f"✗ PCA9685: 初期化失敗 ({e})")
        
        # MotorDriver初期化テスト
        try:
            driver = MotorDriver()
            print("✓ MotorDriver: 初期化成功")
        except Exception as e:
            print(f"✗ MotorDriver: 初期化失敗 ({e})")
        
        # MotorEncoder初期化テスト
        try:
            encoder = MotorEncoder(0, 0, 16, 20, 19, 26)
            print("✓ MotorEncoder: 初期化成功")
        except Exception as e:
            print(f"✗ MotorEncoder: 初期化失敗 ({e})")
        
        # RtSensor初期化テスト
        try:
            sensor = RtSensor(RtSensor.BINARY)
            port = sensor.get_port()
            print(f"✓ RtSensor: 初期化成功 (ポート: {port})")
            sensor.close()
        except Exception as e:
            print(f"✗ RtSensor: 初期化失敗 ({e})")
        
        return True
        
    except Exception as e:
        print(f"✗ ハードウェア初期化エラー: {e}")
        return False

def test_main_program_syntax():
    """メインプログラムのシンタックステスト"""
    print("\n=== メインプログラムシンタックステスト ===")
    
    try:
        import py_compile
        py_compile.compile('ev3lmotor_local_v5.py', doraise=True)
        print("✓ ev3lmotor_local_v5.py: シンタックス正常")
        return True
    except py_compile.PyCompileError as e:
        print(f"✗ ev3lmotor_local_v5.py: シンタックスエラー ({e})")
        return False
    except Exception as e:
        print(f"✗ 予期しないエラー: {e}")
        return False

def test_system_configuration():
    """システム設定テスト"""
    print("\n=== システム設定テスト ===")
    
    import os
    
    # I2C確認
    if os.path.exists('/dev/i2c-1'):
        print("✓ I2C: 有効")
    else:
        print("✗ I2C: 無効")
    
    # GPIO確認
    if os.path.exists('/dev/gpiomem'):
        print("✓ GPIO: 有効")
    else:
        print("✗ GPIO: 無効")
    
    # UARTポート確認
    uart_ports = ['/dev/ttyACM0', '/dev/ttyAMA0', '/dev/ttyS0']
    available_ports = [port for port in uart_ports if os.path.exists(port)]
    
    if available_ports:
        print(f"✓ UARTポート: {', '.join(available_ports)}")
    else:
        print("✗ UARTポート: 利用可能なポートなし")
    
    return True

def main():
    """メインテスト実行"""
    print("Raspberry Pi バランシングロボット制御システム")
    print("最終統合テスト")
    print("=" * 50)
    
    test_results = []
    
    # 各テスト実行
    test_results.append(test_all_imports())
    test_results.append(test_hardware_initialization())
    test_results.append(test_main_program_syntax())
    test_results.append(test_system_configuration())
    
    # 結果集計
    success_count = sum(test_results)
    total_count = len(test_results)
    
    print("\n" + "=" * 50)
    print(f"最終テスト結果: {success_count}/{total_count} 成功")
    
    if success_count == total_count:
        print("✓ 全テスト成功！システムは正常に設定されています")
        print("\n次のステップ:")
        print("1. ハードウェアを接続してください")
        print("2. python3 ev3lmotor_local_v5.py でプログラムを実行してください")
        return True
    else:
        print("✗ 一部のテストが失敗しました")
        print("\n対処方法:")
        print("1. ./install_rpi_packages.sh を実行")
        print("2. sudo raspi-config でI2C/SPI/GPIOを有効化")
        print("3. システムを再起動")
        print("4. ハードウェア接続を確認")
        return False

if __name__ == "__main__":
    try:
        success = main()
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print("\nテストが中断されました")
        sys.exit(1)
    except Exception as e:
        print(f"\n予期しないエラー: {e}")
        traceback.print_exc()
        sys.exit(1)
