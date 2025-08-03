import asyncio
import time
from decimal import Decimal, getcontext
from modules.PCA9685 import PCA9685
from modules.MotorEncoder import MotorEncoder
from modules.MotorDriver import MotorDriver
from modules.RtSensor import RtSensor

FWD, BWD = 0, 1
REV = BWD  # 後方互換性のため
getcontext().prec = 28

# PID制御パラメータ（Motor Driver HAT用に調整）
kp = Decimal("20.0")    # 角度制御ゲイン（25.0→20.0に削減してPWM範囲を広げる）
kd = Decimal("0.5")     # 角速度制御ゲイン（0.6→0.5に削減）
ki = Decimal("250.0")   # 積分制御ゲイン（300.0→250.0に削減）

# 角度オフセット（キャリブレーション用）
angle_offset = 0.0

# 制御パラメータ（Waveshare Motor Driver HAT仕様に合わせて調整）
MAX_PWM = 100              # Motor Driver HAT仕様（TB6612FNG）
MIN_PWM = -100             # Motor Driver HAT仕様（TB6612FNG）
MAX_POWER = 120            # PWM範囲を有効活用（制限を緩和）
PUNCH_POWER = 15           # 静止摩擦突破用（Motor Driver HAT用に調整）
MACH_FACT = Decimal("0.8") # モーター補正係数（0.6→0.8に増加）
MAX_ANGLE = Decimal("45.0")  # 転倒判定角度
CONTROL_INTERVAL = 0.004     # 制御周期 4ms（TB6612FNG高速応答用）
CALIBRATION_SAMPLES = 100    # キャリブレーション回数

# 相補フィルタのカットオフ周波数（応答性向上）
CUTOFF_FREQ = Decimal("0.5")  # 0.1→0.5に増加で応答性向上

def compute_pid_output(angle, velocity_x, integral):
    """
    PID制御出力計算（M5のdriver()関数に基づく）
    X軸角速度を使用した倒立振子制御
    """
    # 基本PID制御
    power = -(angle * kp + velocity_x * kd + integral * ki)
    
    # M5と同様のパワー制限（Decimal型で統一）
    max_power_decimal = Decimal(str(MAX_POWER))
    if power > max_power_decimal:
        power = max_power_decimal
    elif power < -max_power_decimal:
        power = -max_power_decimal
        
    return power

def constrain_pwm_with_punch(value):
    """
    PWM値を安全範囲に制限（TB6612FNG最適化版）
    静止摩擦を克服し、デッドバンドを除去
    符号を保持してモーター方向制御に対応
    """
    # Decimal型で計算
    punch_power_decimal = Decimal(str(PUNCH_POWER))
    
    # デッドバンド除去（小さな信号でも確実に動作）
    if abs(value) > Decimal("1.0") and abs(value) < punch_power_decimal:
        value = punch_power_decimal if value > 0 else -punch_power_decimal
    
    # モーター補正係数適用（符号を保持）
    corrected_value = value * MACH_FACT
    
    # PWM範囲制限（符号を保持）
    return max(MIN_PWM, min(MAX_PWM, int(corrected_value)))

def constrain_pwm(value):
    """PWM値を安全範囲に制限（従来版）"""
    return max(MIN_PWM, min(MAX_PWM, int(value)))

def get_direction(value):
    return REV if value < 0 else FWD

async def motor_control_async(driver, pwm, direction, pwm_val):
    """
    非同期モーター制御（TB6612FNG最適化版）
    符号付きPWM値から方向を判定してモーター制御
    """
    try:
        abs_pwm = abs(pwm_val)
        
        # PWM値がゼロの場合は完全停止
        if abs_pwm <= 0:
            driver.MotorRun(pwm, 0, MotorDriver.direction[FWD], 0)
            driver.MotorRun(pwm, 1, MotorDriver.direction[FWD], 0)
            return
            
        # 正転/逆転制御
        # 正のPWM値: FWD（前進）、負のPWM値: REV（後退）
        motor_direction = direction
        
        # 両モーターを同時制御（倒立振子なので同じ方向）
        driver.MotorRun(pwm, 0, MotorDriver.direction[motor_direction], abs_pwm)
        driver.MotorRun(pwm, 1, MotorDriver.direction[motor_direction], abs_pwm)
        
    except Exception as e:
        print(f"モーター制御エラー: {e}")
        # エラー時は安全停止
        driver.MotorRun(pwm, 0, MotorDriver.direction[FWD], 0)
        driver.MotorRun(pwm, 1, MotorDriver.direction[FWD], 0)

async def emergency_stop_async(driver, pwm):
    """
    非同期緊急停止
    """
    try:
        driver.MotorRun(pwm, 0, MotorDriver.direction[FWD], 0)
        driver.MotorRun(pwm, 1, MotorDriver.direction[FWD], 0)
    except:
        pass

async def mainloop_async(interval, sensor, driver, pwm):
    """
    完全非同期版メインループ
    """
    # 第1回キャリブレーション実行
    sensor.calibration1(CALIBRATION_SAMPLES)
    
    # 制御変数の初期化
    angle = Decimal("0.0")
    integral = Decimal("0.0")
    standing = False
    over_power_counter = 0
    max_over_power = 20
    
    start_time = time.time()
    loop_counter = 0
    last_loop_time = time.time()
    
    print("非同期倒立制御開始 - ロボットを起立させてください")
    
    while True:
        loop_start = time.time()
        dt_actual = loop_start - last_loop_time
        last_loop_time = loop_start

        if not standing:
            # 非倒立時の処理
            if sensor.check_balance_start_condition():
                print("倒立開始条件を満たしました - 第2回キャリブレーション実行")
                sensor.calibration2(CALIBRATION_SAMPLES)
                sensor.reset_angle()
                angle = Decimal("0.0")
                integral = Decimal("0.0")
                standing = True
                print("非同期倒立制御開始！")
                
            calibrated_data = await sensor.get_calibrated_data_async()
            if calibrated_data:
                print(f"Δt: {dt_actual*1000:.1f}ms\t待機中 - acc_z: {calibrated_data['acc_z']:.3f}")
                
        else:
            # 倒立時の制御処理（完全非同期）
            # センサーデータ取得と角度計算を並行実行
            angle_task = asyncio.create_task(
                sensor.get_complementary_angle_async(CONTROL_INTERVAL, float(CUTOFF_FREQ))
            )
            data_task = asyncio.create_task(sensor.get_calibrated_data_async())
            
            # 両方のタスクの完了を待つ
            angle, calibrated_data = await asyncio.gather(
                angle_task, data_task, return_exceptions=True
            )
            
            # エラーハンドリング
            if isinstance(angle, Exception):
                angle = Decimal("0.0")
            else:
                angle = Decimal(str(angle))
                
            if isinstance(calibrated_data, Exception) or not calibrated_data:
                continue
                
            velocity_x = Decimal(str(calibrated_data['velocity_x_corrected']))  # X軸角速度（倒立振子用）
            
            # 転倒判定
            angle_over = abs(angle) > MAX_ANGLE
            power_over = over_power_counter > max_over_power
            
            if angle_over or power_over:
                print(f"転倒検出 - 角度: {angle:.2f}°, 過負荷カウンタ: {over_power_counter}")
                await emergency_stop_async(driver, pwm)
                angle = Decimal("0.0")
                integral = Decimal("0.0")
                over_power_counter = 0
                standing = False
                sensor.reset_angle()
                print("倒立制御を再開するにはロボットを起立させてください")
                continue
                
            # PID制御計算（M5ロジック基準）
            integral += angle * Decimal(str(CONTROL_INTERVAL))
            power = compute_pid_output(angle, velocity_x, integral)
            
            # 過負荷チェック（M5のMAX_POWERに基づく）
            max_power_decimal = Decimal(str(MAX_POWER))
            if abs(power) > max_power_decimal:
                over_power_counter += 1
            else:
                over_power_counter = 0
                
            # PWM値計算と制限（符号付きPWM値を生成）
            pwm_val = constrain_pwm_with_punch(power)
            direction = get_direction(pwm_val)  # PWM値から方向判定
            
            print(f"Δt: {dt_actual*1000:.1f}ms\tangle: {angle:.3f}°\tvel_x: {velocity_x:.3f}\tpwr: {power:.1f}\tpwm: {pwm_val}\tdir: {'REV' if direction == REV else 'FWD'}")

            # モーター制御を非同期で実行
            await motor_control_async(driver, pwm, direction, pwm_val)

        loop_counter += 1
        
        # 200ループ毎の状態表示（5ms周期なので200ループ≈1秒）
        if loop_counter >= 200:
            loop_counter = 0
            status = "倒立中" if standing else "待機中"
            print(f"状態: {status}, 角度: {angle:.2f}°, 積分: {integral:.2f}")

        # 制御周期の調整（高精度）
        elapsed = time.time() - loop_start
        target_sleep = interval - elapsed
        
        if target_sleep > 0.001:
            await asyncio.sleep(target_sleep)

# 同期初期化領域
if __name__ == "__main__":
    try:
        # モーター初期化（Motor Driver HAT最適化）
        print("Motor Driver HAT初期化中...")
        encoder = MotorEncoder(FWD, FWD, 16, 20, 19, 26)
        driver = MotorDriver()
        pwm = PCA9685(0x40, debug=False)
        pwm.setPWMFreq(1000)  # TB6612FNG用に1000Hzに設定（応答性向上）

        # RtSensorをBINARYモードで初期化
        with RtSensor(RtSensor.BINARY) as sensor:
            port = sensor.get_port()
            print(f"センサー接続: {port}")
            print(f"制御周期: {CONTROL_INTERVAL*1000:.0f}ms")
            print(f"PIDゲイン - P:{kp}, I:{ki}, D:{kd}")
            print("=" * 50)

            # 非同期メインループを実行
            asyncio.run(mainloop_async(CONTROL_INTERVAL, sensor, driver, pwm))
            
    except ValueError as e:
        print(f"センサー初期化エラー: {e}")
        print("RT-net 9軸IMUセンサーの接続を確認してください")
        print("対象ポート: /dev/ttyACM0, /dev/ttyAMA0, /dev/ttyS0")
    except KeyboardInterrupt:
        print("\n制御を終了します")
    except Exception as e:
        print(f"予期しないエラーが発生しました: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # モーター緊急停止処理
        try:
            if 'driver' in locals() and 'pwm' in locals():
                driver.MotorRun(pwm, 0, MotorDriver.direction[FWD], 0)
                driver.MotorRun(pwm, 1, MotorDriver.direction[FWD], 0)
                print("モーターを安全停止しました")
        except:
            pass
        print("システム終了")