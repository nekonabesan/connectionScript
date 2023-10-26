import serial
import logging
from serial.tools import list_ports

class ConnectGyro():
    TTY_VALUE = '/dev/ttyACM0'
    BAUDRATE = 115200
    TIMEOUT = 2.0

    logger = None
    connection = None

    def __init__(self):
        try:
            logging.basicConfig(format='%(levelname)s:%(asctime)s:%(pathname)s:%(lineno)s:%(message)s')
            self.logger = logging.getLogger(__name__)
            self.logger.setLevel(logging.DEBUG) 
            self.initConnection()
        except Exception as e:
            self.logger.debug(e.args, stacklevel = 2)

    def initConnection(self):
        try:
            self.connection = serial.Serial(self.TTY_VALUE, self.BAUDRATE, timeout=self.TIMEOUT)
            return True
        except Exception as e:
            self.logger.debug(e.args, stacklevel = 2)
            return False


print("===== Set Parameter Complete =====\n")
# # Read Serial
try:
    gyro = ConnectGyro()
    connection = gyro.connection
    line = connection.readline()
    while(1):
        lines = line.strip().decode('utf-8').replace('\r\n','').split('\t')
        if (len(lines) < 3):
            print(line)
            continue
        print("Read Serial:")
        print(str(lines[0]) + "\t " + str(lines[1]) + "\t" + str(lines[2]))
        break
    print("===== Read Serial Complete =====\n")
    connection.close()
except Exception as e:
    print(e)