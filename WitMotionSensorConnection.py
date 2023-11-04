# coding:UTF-8

import time
import datetime
import platform
import struct
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
    欢迎使用维特智能示例程序    Welcome to the Wit-Motoin sample program
    """
    _writeF = None                    #写文件  Write file
    _IsWriteF = False                 #写文件标识    Write file identification

    accX = None
    accY = None
    accZ = None
    gyroX = None
    gyroY = None
    gyroZ = None
    angleX = None
    angleY = None
    angleZ = None


    def getDevice(self):
        return deviceModel.DeviceModel(
            "JY901",
            WitProtocolResolver(),
            JY901SDataProcessor(),
            "51_0"
        )

    def readConfig(self, device):
        """
        读取配置信息示例    Example of reading configuration information
        :param device: 设备模型 Device model
        :return:
        """
        tVals = device.readReg(0x02,3)  #读取数据内容、回传速率、通讯速率   Read data content, return rate, communication rate
        if (len(tVals)>0):
            print("返回结果：" + str(tVals))
        else:
            print("无返回")
        tVals = device.readReg(0x23,2)  #读取安装方向、算法  Read the installation direction and algorithm
        if (len(tVals)>0):
            print("返回结果：" + str(tVals))
        else:
            print("无返回")

    def setConfig(self, device):
        """
        设置配置信息示例    Example setting configuration information
        :param device: 设备模型 Device model
        :return:
        """
        device.unlock()                # 解锁 unlock
        time.sleep(0.1)                # 休眠100毫秒    Sleep 100ms
        device.writeReg(0x03, 6)       # 设置回传速率为10HZ    Set the transmission back rate to 10HZ
        time.sleep(0.1)                # 休眠100毫秒    Sleep 100ms
        device.writeReg(0x23, 0)       # 设置安装方向:水平、垂直   Set the installation direction: horizontal and vertical
        time.sleep(0.1)                # 休眠100毫秒    Sleep 100ms
        device.writeReg(0x24, 0)       # 设置安装方向:九轴、六轴   Set the installation direction: nine axis, six axis
        time.sleep(0.1)                # 休眠100毫秒    Sleep 100ms
        device.save()                  # 保存 Save

    def AccelerationCalibration(self, device):
        """
        加计校准    Acceleration calibration
        :param device: 设备模型 Device model
        :return:
        """
        device.AccelerationCalibration()                 # Acceleration calibration
        print("加计校准结束")

    def FiledCalibration(self, device):
        """
        磁场校准    Magnetic field calibration
        :param device: 设备模型 Device model
        :return:
        """
        device.BeginFiledCalibration()                   # 开始磁场校准   Starting field calibration
        if input("请分别绕XYZ轴慢速转动一圈，三轴转圈完成后，结束校准（Y/N)？").lower()=="y":
            device.EndFiledCalibration()                 # 结束磁场校准   End field calibration
            print("结束磁场校准")

    def onUpdate(self, deviceModel):
        """
        Data update event
        :param deviceModel: Device model
        :return: None
        """
        self.accX = deviceModel.getDeviceData("accX")
        self.accY = deviceModel.getDeviceData("accY")
        self.accZ = deviceModel.getDeviceData("accZ")
        self.gyroX = deviceModel.getDeviceData("gyroX")
        self.gyroY = deviceModel.getDeviceData("gyroY")
        self.gyroZ = deviceModel.getDeviceData("gyroZ")
        self.angleX = deviceModel.getDeviceData("angleX")
        self.angleY = deviceModel.getDeviceData("angleY")
        self.angleZ = deviceModel.getDeviceData("angleZ")

    
    def getYData(self):
        return self.accY,self.gyroY,self.angleY

    def startRecord(self):
        """
        开始记录数据  Start recording data
        :return:
        """
        #global _writeF
        #global _IsWriteF
        self._writeF = open(str(datetime.datetime.now().strftime('%Y%m%d%H%M%S')) + ".txt", "w")    #新建一个文件
        self._IsWriteF = True                                                                        #标记写入标识
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
        print("开始记录数据")

    def endRecord(self):
        """
        结束记录数据  End record data
        :return:
        """
        #global _writeF
        #global _IsWriteF
        self._IsWriteF = False             # 标记不可写入标识    Tag cannot write the identity
        self._writeF.close()               #关闭文件 Close file
        print("结束记录数据")


    
