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