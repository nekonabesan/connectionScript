import platform
from WitMotionSensorConnection import JY901S

if __name__ == '__main__':
    obj = JY901S()
    device = obj.getDevice()
    device.serialConfig.portName = "/dev/ttyUSB0"   # Set serial port
    device.serialConfig.baud = 9600                     # Set baud rate
    device.openDevice()                                 # Open serial port
    # Read configuration information
    obj.readConfig(device)
    # Data update event
    device.dataProcessor.onVarChanged.append(obj.onUpdate)

    # Start recording data
    obj.startRecord()
    
    while True:
        accY,gyroY,angleY = obj.getYData()
        print("acc : " + str(accY) + "\t velocity : " + str(gyroY) + "\tdegrees : " + str(angleY))
    input()
    device.closeDevice()
    obj.endRecord()