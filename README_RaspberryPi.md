# Raspberry Pi バランシングロボット制御システム

## 概要
RT-net 9軸IMUセンサーを使用したバランシングロボット（倒立振子）制御システムです。
Raspberry Pi上で動作し、PID制御アルゴリズムにより高精度な姿勢制御を実現します。

## システム要件
- Raspberry Pi 4 Model B (推奨)
- Raspberry Pi OS (Bullseye以降)
- RT-net 9軸IMUセンサー
- PCA9685 PWMドライバ
- DCモーター x2
- ロータリーエンコーダ x2

## セットアップ手順

### 1. システムパッケージのインストール
```bash
sudo ./install_rpi_packages.sh
sudo reboot
```

### 2. 動作確認テスト
```bash
python3 test_rpi_modules.py
```

### 3. I2C・SPI・GPIOの有効化確認
```bash
sudo raspi-config
# Interface Options → I2C → Enable
# Interface Options → SPI → Enable  
# Interface Options → GPIO → Enable
```

### 4. ハードウェア接続
#### RT-net IMUセンサー (UART接続)
- VCC → 3.3V
- GND → GND
- TX → GPIO 14 (UART RX)
- RX → GPIO 15 (UART TX)

#### PCA9685 PWMドライバ (I2C接続)
- VCC → 3.3V
- GND → GND
- SDA → GPIO 2 (I2C SDA)
- SCL → GPIO 3 (I2C SCL)
- アドレス: 0x40

#### ロータリーエンコーダ
- エンコーダ1: GPIO 16, 20
- エンコーダ2: GPIO 19, 26

## 使用方法

### メインプログラムの実行
```bash
python3 ev3lmotor_local_v5.py
```

### 実行手順
1. プログラム開始
2. 自動的に第1回キャリブレーション実行
3. ロボットを手で起立状態にする
4. 倒立条件を満たすと第2回キャリブレーション実行
5. 自動倒立制御開始

### 制御パラメータ
- 制御周期: 10ms
- PIDゲイン: P=37.0, I=800.0, D=0.84
- 転倒判定角度: ±30度
- 最大PWM: ±100

## ファイル構成

### メインプログラム
- `ev3lmotor_local_v5.py`: メイン制御プログラム

### モジュール (modules/)
- `RtSensor.py`: 9軸IMUセンサー制御
- `PCA9685.py`: PWM制御
- `MotorDriver.py`: モーター制御
- `MotorEncoder.py`: エンコーダー制御

### テスト・セットアップ
- `test_rpi_modules.py`: モジュールインポートテスト
- `install_rpi_packages.sh`: 自動セットアップスクリプト
- `requirements.txt`: Python依存関係

## トラブルシューティング

### センサー接続エラー
```
センサー初期化エラー: No module named 'smbus'
```
→ `sudo ./install_rpi_packages.sh` を実行後、再起動

### I2Cデバイスが見つからない
```bash
sudo i2cdetect -y 1
```
→ PCA9685が0x40に表示されるか確認

### GPIO権限エラー
```bash
sudo usermod -a -G gpio $USER
sudo usermod -a -G i2c $USER
```
→ ログアウト・ログインし直す

### UARTポートエラー
- `/dev/ttyACM0`: USB接続
- `/dev/ttyAMA0`: GPIO UART
- `/dev/ttyS0`: ミニUART

## 安全機能

### 自動停止条件
- 転倒角度30度以上
- 制御出力過大（連続20回）
- Ctrl+C による緊急停止

### 緊急停止
プログラム終了時、全モーターが自動的に停止します。

## 開発・改良

### ログ出力レベル変更
プログラム内の `print` 文を調整してください。

### PIDパラメータ調整
```python
kp = Decimal("37.0")    # 角度制御ゲイン
kd = Decimal("0.84")    # 角速度制御ゲイン  
ki = Decimal("800.0")   # 積分制御ゲイン
```

### 制御周期変更
```python
CONTROL_INTERVAL = 0.01  # 10ms
```

## ライセンス
このプロジェクトはMITライセンスの下で公開されています。

## 作成者
RT-net 9軸IMUセンサーバランシングロボット制御システム
