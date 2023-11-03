import RPi.GPIO as GPIO
from time import sleep

PWMA1 = 6 
PWMA2 = 13
PWMB1 = 20
PWMB2 = 21
D1 = 12
D2 = 26

#GPIOのポート番号を使用
GPIO.setmode(GPIO.BCM)

#モーターで使うGPIOをセットアップ
GPIO.setup(PWMA1,GPIO.OUT)
GPIO.setup(PWMA2,GPIO.OUT)
GPIO.setup(PWMB1,GPIO.OUT)
GPIO.setup(PWMB2,GPIO.OUT)
GPIO.setup(D1,GPIO.OUT)
GPIO.setup(D2,GPIO.OUT)

#PWMの設定
motor1 = GPIO.PWM(D1,50)
motor2 = GPIO.PWM(D2,50)


#()内には0～100のスピードを指定
motor1.start(50)
motor2.start(50)

# モーターを動かす。
GPIO.output(PWMA1,1)
GPIO.output(PWMA2,0)

GPIO.output(PWMB1,1)
GPIO.output(PWMB2,0)

# モーターを3秒動かす
sleep(3)

# モーターを停止。
GPIO.output(PWMA1,0)
GPIO.output(PWMA2,0)

GPIO.output(PWMB1,0)
GPIO.output(PWMB2,0)

