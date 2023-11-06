# coding:UTF-8

import time
import datetime
import platform
import struct
from decimal import *
import lib.device_model as deviceModel
from lib.data_processor.roles.jy901s_dataProcessor import JY901SDataProcessor
from lib.protocol_resolver.roles.wit_protocol_resolver import WitProtocolResolver

# ---------------------------------------------------------------------------------------------------- #
# 
# Sample Source
#   https://github.com/WITMOTION/WitStandardProtocol_JY901/blob/main/Python/Python-WitProtocol/chs/JY901S.py
# Git 
#   https://github.com/WITMOTION/WitStandardProtocol_JY901/tree/main
# manual
#   https://wit-motion.gitbook.io/witmotion-sdk/wit-standard-protocol/sdk/python_sdk-quick-start
# 
# ---------------------------------------------------------------------------------------------------- #
class JY901S:
    welcome = """
    Welcome to the Wit-Motoin sample program
    """
    _writeF = None                    #写文件  Write file
    _IsWriteF = False                 #写文件标识    Write file identification

    accX = 0.0
    accY = 0.0
    accZ = 0.0
    gyroX = 0.0
    gyroY = 0.0
    gyroZ = 0.0
    angleX = 0.0
    angleY = 0.0
    angleZ = 0.0


    def getDevice(self):
        return deviceModel.DeviceModel(
            "JY901",
            WitProtocolResolver(),
            JY901SDataProcessor(),
            "51_0"
        )

    def readConfig(self, device):
        """
        Example of reading configuration information
        :param device: Device model
        :return:
        """
        tVals = device.readReg(0x02,3)  # Read data content, return rate, communication rate
        if (len(tVals)>0):
            print("：" + str(tVals))
        else:
            print("")
        tVals = device.readReg(0x23,2)  # Read the installation direction and algorithm
        if (len(tVals)>0):
            print("：" + str(tVals))
        else:
            print("")

    def setConfig(self, device):
        """
        Example setting configuration information
        :param device: Device model
        :return:
        """
        #device.unlock()
        time.sleep(0.1)
        # Set the transmission back rate to 10HZ
        #device.writeReg(0x03, 6)       
        time.sleep(0.1)
        # Set the installation direction: horizontal and vertical
        #device.writeReg(0x23, 0)       
        time.sleep(0.1)
        # Set the installation direction: nine axis, six axis
        #device.writeReg(0x24, 0)       
        time.sleep(0.1)
        #device.save()

    def AccelerationCalibration(self, device):
        """
        Acceleration calibration
        :param device: Device model
        :return:
        """
        device.AccelerationCalibration()                 # Acceleration calibration
        print("")

    def FiledCalibration(self, device):
        """
        Magnetic field calibration
        :param device: Device model
        :return:
        """
        device.BeginFiledCalibration()
        #if input("（Y/N)？").lower()=="y":
        #    device.EndFiledCalibration()
        #    print("")
        device.EndFiledCalibration()

    def onUpdate(self, deviceModel):
        """
        Data update event
        :param deviceModel: Device model
        :return: None
        """
        self.accX = Decimal(str(deviceModel.getDeviceData("accX")))
        self.accY = Decimal(str(deviceModel.getDeviceData("accY")))
        self.accZ = Decimal(str(deviceModel.getDeviceData("accZ")))
        self.gyroX = Decimal(str(deviceModel.getDeviceData("gyroX")))
        self.gyroY = Decimal(str(deviceModel.getDeviceData("gyroY")))
        self.gyroZ = Decimal(str(deviceModel.getDeviceData("gyroZ")))
        self.angleX = Decimal(str(deviceModel.getDeviceData("angleX")))
        self.angleY = Decimal(str(deviceModel.getDeviceData("angleY")))
        self.angleZ = Decimal(str(deviceModel.getDeviceData("angleZ")))

    def getDataAxisAll(self):
        return self.accX,self.gyroX,self.angleX,self.accY,self.gyroY,self.angleY,self.accZ,self.gyroZ,self.angleZ

    def getYDataAxisY(self):
        return self.accY,self.gyroY,self.angleY

    def startRecord(self):
        """
        Start recording data
        :return:
        """
        #global _writeF
        #global _IsWriteF
        self._writeF = open(str(datetime.datetime.now().strftime('%Y%m%d%H%M%S')) + ".txt", "w")
        self._IsWriteF = True
        Tempstr = "Chiptime"
        Tempstr +=  "\tax(g)\tay(g)\taz(g)"
        Tempstr += "\twx(deg/s)\twy(deg/s)\twz(deg/s)"
        Tempstr += "\tAngleX(deg)\tAngleY(deg)\tAngleZ(deg)"
        Tempstr += "\tT(°)"
        Tempstr += "\tmagx\tmagy\tmagz"
        Tempstr += "\tlon\tlat"
        Tempstr += "\tYaw\tSpeed"
        Tempstr += "\tq1\tq2\tq3\tq4"
        Tempstr += "\r\n"
        self._writeF.write(Tempstr)
        print("")

    def endRecord(self):
        """
        End record data
        :return:
        """
        #global _writeF
        #global _IsWriteF
        self._IsWriteF = False             # Tag cannot write the identity
        self._writeF.close()               #Close file
        print("")


    
