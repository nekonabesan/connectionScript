#!/usr/bin/python
import os
import sys
import subprocess
from sense_hat import SenseHat

OFFSET = 1

# sense hatを初期化
angle = []
# sense hat指示値読み替え用配列定義
for i in range(360):
    if i <= 180:
        angle.append(i)
    else:
        angle.append(i - 360)

# sense hatの指示値読み取りをスレッド化
# read_sense_hat_angle_y.serviceを/usr/lib/systemd/system/配下へコピー
# sudo systemctl daemon-reload
# sudo systemctl start read_sense_hat_angle_y.service
# sudo systemctl status read_sense_hat_angle_y.service
def read_sense_hat_angle():
    global angle
    sense = SenseHat()
    while True:
        orientation = sense.get_orientation_degrees()
        subprocess.run("echo '" + str(angle[int(round(orientation['pitch'])) - OFFSET]) + "' > /opt/angle_y", shell=True)

def daemonize():
    pid = os.fork()
    if pid > 0:
        pid_file = open('/var/run/read_sense_hat_angle_y.pid','w')
        pid_file.write(str(pid)+"\n")
        pid_file.close()
        sys.exit()
    if pid == 0:
        read_sense_hat_angle()

if __name__ == '__main__':
    while True:
        daemonize()