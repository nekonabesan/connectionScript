# 倒立振子制御スクリプト
## Raspberry Py Sense HATから傾斜角度を取得する際の注意事項
### 前提
- Sense HATの読み取り周期は50から100Hz/sec程度と類推できる
- 繰り返し処理から直接センサ指示値を読み取ると待ち時間込みで1周期の時間計算量が250ミリ秒前後となる

### NW制御時の暫定対応
- センサ指示値読み取りをサービス化
- サービス起動したスクリプトから傾斜角度をテキストファイルへ書き出し
- サーバとの通信を行うスクリプト内の繰り返し処理内部でRequest＋モータ制御とテキストから傾斜角度を読み出す処理を並列処理で実行

### センサ指示値読み取りスクリプトのリリース並びにサービス起動
#### サービス定義
以下のディレクトリに引用内容のファイルを作成
/usr/lib/systemd/system/read_sense_hat_angle_y.service
'''
[Unit]
Description=PythonDaemon

[Service]
ExecStart=/opt/read_sense_hat_angle_y.py
Restart=always
Type=forking
PIDFile=/var/run/read_sense_hat_angle_y.pid

[Install]
WantedBy=multi-user.target
'''

#### スクリプトをリリース
/opt/配下にread_sense_hat_angle_y.pyをコピーしパーミッション775を付与

#### サービス起動
systemdをリロード
'''
sudo systemctl daemon-reload
'''

サービスを開始
'''
sudo systemctl start read_sense_hat_angle_y.service
'''

サービスの状態を確認
'''
sudo systemctl status read_sense_hat_angle_y.service
'''

## 開発環境構築手順

### 前提条件
- Python 3.11以上がインストールされていること
- Raspberry Pi OS環境であること
- RT-net USB 9軸IMUセンサーが接続されていること

### 1. Python仮想環境の作成

プロジェクトルートディレクトリで以下のコマンドを実行：

```bash
# 仮想環境の作成
python3 -m venv .venv

# 仮想環境のアクティベート
source .venv/bin/activate

# Pythonバージョンの確認
python --version
```

### 2. 必要なパッケージのインストール

```bash
# pipのアップグレード
pip install --upgrade pip

# 依存パッケージのインストール
pip install -r requirements.txt
```

### 3. 開発環境の使用

#### 手動での環境アクティベート
```bash
# 仮想環境をアクティベート
source .venv/bin/activate

# PYTHONPATHの設定
export PYTHONPATH="${PYTHONPATH}:$(pwd)/modules"

# 仮想環境のデアクティベート
deactivate
```

#### 開発用スクリプトの使用
```bash
# 開発環境をアクティベート（推奨）
source activate_dev.sh

# テスト実行付きでアクティベート
source activate_dev.sh test
```

### 4. テストの実行

```bash
# 仮想環境をアクティベート
source .venv/bin/activate

# PYTHONPATHの設定
export PYTHONPATH="${PYTHONPATH}:$(pwd)/modules"

# すべてのテストを実行
pytest test/ -v

# 特定のテストファイルのみ実行
pytest test/RtSensorTest.py -v

# 特定のテスト関数のみ実行
pytest test/RtSensorTest.py::test_get_raw_binary -v
```

### 5. 主要な依存パッケージ

| パッケージ | 用途 | バージョン |
|-----------|------|------------|
| pyserial | シリアル通信（RT-net 9軸IMU用） | >=3.5 |
| pyserial-asyncio | 非同期シリアル通信 | >=0.6 |
| pytest | テストフレームワーク | >=7.0.0 |
| RPi.GPIO | Raspberry Pi GPIO制御 | >=0.7.1 |
| gpiozero | GPIO制御の高レベルライブラリ | >=1.6.2 |
| pigpio | 精密なGPIO制御 | >=1.78 |
| smbus2 | I2C通信（PCA9685用） | >=0.4.2 |
| sense-hat | Sense HAT制御 | >=2.6.0 |
| requests | HTTP通信 | >=2.28.0 |
| numpy | 数値計算 | >=1.21.0 |

### 6. プロジェクト構成

```
connectionScript/
├── .venv/                    # Python仮想環境
├── modules/                  # 共通モジュール
│   ├── RtSensor.py          # RT-net 9軸IMUセンサー制御
│   ├── MotorDriver.py       # モーター制御
│   ├── PCA9685.py           # PWM制御
│   └── ConnectionSenseHat.py # Sense HAT接続
├── test/                     # テストファイル
│   └── RtSensorTest.py      # RtSensorのテスト
├── requirements.txt          # 依存パッケージリスト
├── activate_dev.sh          # 開発環境アクティベートスクリプト
└── README.md                # このファイル
```

### 7. RtSensorクラスの使用例

```python
from modules.RtSensor import RtSensor

# コンテキストマネージャーとして使用（推奨）
with RtSensor(RtSensor.BINARY) as sensor:
    # 生データの取得
    data = sensor.get_raw_binary()
    print(f"加速度: {data[:3]}")
    print(f"角速度: {data[3:6]}")
    print(f"温度: {data[6]}")
    
    # ロボット本体の角度と速度取得
    velocity_y, gyro_offset = sensor.get_robot_body_angle_and_speed()
    print(f"Y軸角速度: {velocity_y}")
    print(f"ジャイロオフセット: {gyro_offset}")
```

### 8. トラブルシューティング

#### センサーが見つからない場合
- USBケーブルの接続を確認
- `/dev/ttyACM0`, `/dev/ttyAMA0`, `/dev/ttyS0` のいずれかにデバイスが認識されているか確認
- 権限の問題がある場合は `sudo usermod -a -G dialout $USER` を実行後、再ログイン

#### テストでセンサーが見つからない場合
- テストは自動的にスキップされます
- 実際のセンサーを接続してからテストを実行してください

#### パッケージインストールエラー
- Raspberry Pi用の最適化されたパッケージを使用するため、piwheelsが有効になっていることを確認
- `pip install --index-url https://www.piwheels.org/simple/ <package_name>` で個別インストールを試行