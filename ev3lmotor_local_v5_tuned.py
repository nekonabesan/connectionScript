#!/usr/bin/env python3
"""
Raspberry PiMAX_POWER = 120            # PWM範囲拡大（80→120）
PUNCH_POWER = 20          # M5: 15 → 段階2: 20（PWM範囲拡大）
MACH_FACT = 0.8           # 機械係数増加（0.6→0.8）期倒立振子制御システム v5（調整版）
M5のPIDパラメータを段階的に調整したバージョン

- M5オリジナル: K_angle=37.0, K_omg=0.84, KI_angle=800.0
- 調整版: より穏やかな制御で様子を見る
"""

import asyncio
import time
import sys
import signal
from decimal import Decimal, getcontext

# 精度設定
getcontext().prec = 8

# パス設定
sys.path.append('./modules')
from MotorDriver import MotorDriver
from PCA9685 import PCA9685
from RtSensor import RTSensor

# 方向定数
FWD = "FWD"
REV = "REV"

# PID制御パラメータ（M5値を段階的に調整）
kp = Decimal("12.0")    # M5: 37.0 → 段階2: 12.0（PWM範囲拡大）
kd = Decimal("0.4")     # M5: 0.84 → 段階2: 0.4（PWM範囲拡大）  
ki = Decimal("150.0")   # M5: 800.0 → 段階2: 150.0（PWM範囲拡大）

# 角度オフセット（キャリブレーション用）
angle_offset = 0.0

# 制御パラメータ（M5オリジナル値に合わせて調整）
MAX_PWM = 127              # M5オリジナル値
MIN_PWM = -127             # M5オリジナル値  
MAX_POWER = 80             # M5: 120 → 段階1: 80（約67%）
PUNCH_POWER = 15           # M5: 20 → 段階1: 15（75%）
MACH_FACT = 0.6            # M5: 0.45 → 段階1: 0.6（モーター効率調整）
MAX_ANGLE = Decimal("30.0")  # 転倒判定角度
CONTROL_INTERVAL = 0.005     # 制御周期 5ms（非同期処理で高速化）
CALIBRATION_SAMPLES = 100    # キャリブレーション回数

# 相補フィルタのカットオフ周波数（応答性向上）
CUTOFF_FREQ = Decimal("0.5")  # 0.1→0.5に増加で応答性向上

def compute_pid_output(angle, velocity_x, integral):
    """
    PID制御出力計算（M5のdriver()関数に基づく調整版）
    X軸角速度を使用した倒立振子制御
    """
    # 基本PID制御
    power = -(angle * kp + velocity_x * kd + integral * ki)
    
    # M5と同様のパワー制限
    if power > MAX_POWER:
        power = MAX_POWER
    elif power < -MAX_POWER:
        power = -MAX_POWER
        
    return power

def constrain_pwm_with_punch(value):
    """
    PWM値を安全範囲に制限（TB6612FNG最適化版調整版）
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
    非同期モーター制御（TB6612FNG最適化版調整版）
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

async def mainloop_async(sensor, driver, pwm, interval):
    """
    非同期メインループ（M5ロジック調整版）
    """
    angle = Decimal("0.0")
    integral = Decimal("0.0")
    standing = False
    over_power_counter = 0
    max_over_power = 30  # M5より少し余裕を持つ
    
    start_time = time.time()
    loop_counter = 0
    last_loop_time = time.time()
    
    print("非同期倒立制御開始（調整版） - ロボットを起立させてください")
    print(f"PIDゲイン - Kp:{kp}, Kd:{kd}, Ki:{ki}")
    print(f"制御限界 - MAX_POWER:{MAX_POWER}, PUNCH_POWER:{PUNCH_POWER}, MACH_FACT:{MACH_FACT}")
    
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
                print("非同期倒立制御開始（調整版）！")
                
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
                
            # PID制御計算（M5ロジック調整版）
            integral += angle * Decimal(str(CONTROL_INTERVAL))
            power = compute_pid_output(angle, velocity_x, integral)
            
            # 過負荷チェック（調整されたMAX_POWERに基づく）
            if abs(power) > MAX_POWER:
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
            status = "倒立中（調整版）" if standing else "待機中"
            print(f"状態: {status}, 角度: {angle:.2f}°, 積分: {integral:.2f}")

        # 制御周期の調整（高精度）
        elapsed = time.time() - loop_start
        target_sleep = interval - elapsed
        
        if target_sleep > 0:
            await asyncio.sleep(target_sleep)

def signal_handler(sig, frame):
    """信号ハンドラ"""
    print("\n終了信号を受信しました。プログラムを終了します。")
    sys.exit(0)

if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    
    try:
        # センサー初期化
        print("RTセンサー初期化中...")
        sensor = RTSensor()
        
        print("初期キャリブレーション実行中...")
        sensor.calibration(CALIBRATION_SAMPLES)
        
        # モーター初期化  
        print("モーター初期化中...")
        pwm = PCA9685(0x60, debug=False)
        pwm.setPWMFreq(500)
        driver = MotorDriver()
        
        # 非同期メインループ開始
        print("非同期制御システム起動（調整版）")
        asyncio.run(mainloop_async(sensor, driver, pwm, CONTROL_INTERVAL))
        
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt: プログラム終了")
    except Exception as e:
        print(f"エラーが発生しました: {e}")
        import traceback
        traceback.print_exc()
    
    print("プログラム終了")
