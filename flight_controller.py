import pywinusb.hid as hid
import struct
import threading
import time
import math
import keyboard

class RcStruct:
    def __init__(self):
        self.theta = 0
        self.yaw   = 0
        self.roll  = 0
        self.pitch = 0
        self.AUX1 = 0
        self.AUX2 = 0
        self.AUX3 = 0
        self.AUX4 = 0
        self.AUX5 = 0
        self.AUX6 = 0

        self.updateFlag = False
        
    def Unpack(self, data):
        self.theta, self.yaw, self.roll, self.pitch, \
        self.AUX1, self.AUX2, self.AUX3, self.AUX4, \
        self.AUX5, self.AUX6 = struct.unpack('>'+'h'*10, data[4:4+20])
#        print(self.theta, self.yaw, self.roll, self.pitch, \
#        self.AUX1, self.AUX2, self.AUX3, self.AUX4, \
#        self.AUX5, self.AUX6)
        self.updateFlag = True

class ControlStruct:
    def __init__(self):
        self.theta = 1000 
        self.yaw   = 1500
        self.roll  = 1500
        self.pitch = 1500
        self.AUX1 = 1500
        self.AUX2 = 2000
        self.AUX3 = 1500
        self.AUX4 = 0
        self.AUX5 = 250
        self.AUX6 = 250
        
    def Pack(self):
        head = b'\xAA\xAF\x03' + bytes([20])
        data = struct.pack('>'+'h'*10, self.theta, self.yaw, self.roll, self.pitch, \
                                       self.AUX1, self.AUX2, self.AUX3, self.AUX4, \
                                       self.AUX5, self.AUX6)
        checkSum = bytes([sum(head + data) % 256])
        return head + data + checkSum

class StatusStruct:
    def __init__(self):
        self.angle_rol = 0
        self.angle_pit = 0
        self.angle_yaw = 0
        self.alt = 0 # 高度
        self.fly_model = 0
        self.armed = 0
        
        self.updateFlag = False
        
    def Unpack(self, data):
        self.angle_rol, self.angle_pit, self.angle_yaw, \
        self.alt, self.fly_model,self.armed = struct.unpack('>'+'hhhibb', data[4:4+12])
        self.updateFlag = True
        #print(self.armed)

class SensorStruct:
    def __init__(self):
        self.a_x = 0
        self.a_y = 0
        self.a_z = 0
        self.g_x = 0
        self.g_y = 0
        self.g_z = 0
        
        #self.m_x = 0
        #self.m_y = 0
        #self.m_z = 0
        self.flow_x = 0
        self.flow_y = 0
        self.flow_qual = 0
        
        self.updateFlag = False
        
    def Unpack(self, data):
        self.a_x, self.a_y, self.a_z, \
        self.g_x, self.g_y, self.g_z, \
        self.flow_x, self.flow_y, self.flow_qual = struct.unpack('>'+'h'*9, data[4:4+18])
        self.updateFlag = True

class PowerStruct:
    def __init__(self):
        self.flight_votage = 0
        self.controller_votage = 0
        self.flag0 = 0
        self.flag1 = 0
        self.flag2 = 0
        self.flag3 = 0
        self.updateFlag = False
        
    def Unpack(self, data):
        flight_votage, controller_votage, self.flag0, self.flag1, \
        self.flag2, self.flag3 = struct.unpack('>'+'HHBBHH', data[4:4+10])
        self.flight_votage = flight_votage * 0.01
        self.controller_votage = controller_votage * 0.01
        if self.flight_votage < 3.4:
            print('无人机电量低：{} V'.format(self.flight_votage))
        #if self.controller_votage < 3.7:
        #    print('遥控器电量低：{} V'.format(self.controller_votage))
        self.updateFlag = True

class SpeedStruct:
    def __init__(self):
        self.rol_x = 0
        self.pit_y = 0
        self.speed_z = 0
        
        self.updateFlag = False
        
    def Unpack(self, data):
        self.rol_x, self.pit_y, self.speed_z = struct.unpack('>'+'HHH', data[4:4+6])


class Sensor2Struct:
    def __init__(self):
        self.bar_alt = 0
        self.csb_alt = 0
        
        self.updateFlag = False
        
    def Unpack(self, data):
        self.bar_alt, self.csb_alt = struct.unpack('>'+'iH', data[4:4+6])
        self.updateFlag = True

class CustomStruct:
    def __init__(self):
        self.sendData = b''
        self.recvData = b''
        self.updateFlag = False
        
        self.temperature = 0
        self.humidity = 0
        self.updateFlag_SHT3X = False
        
    def Unpack(self, data):
        if data[4:7] == '$HT': # 获取温湿度传感器
            self.temperature, self.humidity = struct.unpack('>ff', data[7:15])
            self.updateFlag_HT = True
        else:
            self.recvData += data[4:-1]
            self.updateFlag = False

    def Pack(self):
        if type(self.sendData) != type(b''):
            self.sendData = self.sendData.encode('utf-8')
        head = b'\xAA\xAF\xF2' + bytes([len(self.sendData)])
        data = self.sendData
        checkSum = bytes([sum(head + data) % 256])
        return head + data + checkSum

class CustomIOStruct:
    def __init__(self):
        self.TxData = b''
        self.RxData = b''
        
    def Unpack(self, data):
        self.RxData += data[4:-1]

    def PackIO(self, gpio, status):
        head = b'\xAA\xAF\xF3\x02'
        data = bytes([gpio, status])
        checkSum = bytes([sum(head + data) % 256])
        return head + data + checkSum
    
    def PackTx(self):
        if type(self.TxData) != type(b''):
            self.TxData = self.TxData.encode('utf-8')
        head = b'\xAA\xAF\xF3' + bytes([len(self.TxData)+1])
        data = b'\x00' + self.TxData
        checkSum = bytes([sum(head + data) % 256])
        return head + data + checkSum

class NoRcConnectedException(Exception):
    def __init__(self,ErrorInfo):
        self.ErrorInfo = ErrorInfo
    def __str__(self):
        return self.ErrorInfo

class FlightController:
    GPIO_S = 1
    GPIO_K = 2
    GPIO_I = 3
    GPIO_O = 4

    HIGH = 1
    LOW = 0
    def __init__(self):
        
        self.control = ControlStruct()
        
        self.custom = CustomStruct()
        self.customIO = CustomIOStruct()

        self.rc = RcStruct()
        self.status = StatusStruct()
        self.sensor = SensorStruct()
        self.sensor2 = Sensor2Struct()
        self.power = PowerStruct()
        self.speed = SpeedStruct()
        self.lastSendTime = 0
        self.setArmed = False
         # 功能字
        self.structDict = {0x03: ['rc', self.rc],
                           0x01: ['status', self.status],
                           0x02: ['sensor', self.sensor],
                           0x07: ['sensor2', self.sensor2],
                           0x0B: ['speed', self.speed],
                           0x05: ['power', self.power],
                           0xF2: ['custom', self.custom],
                           0xF3: ['customIO', self.customIO]
                           }

        self.checkThread = threading.Thread(target = self.CheckFunc, args=())
        self.checkThread.setDaemon(True)
        self.checkThread.start()  #打开收数据的线程

    def CheckFunc(self):
        checkStatus = 0
        while True:
            if self.setArmed:
                if time.time() - self.lastSendTime > 1.0 and checkStatus == 1:
                    print('控制指令发送间隔过长，遥控器切换为手动模式！')
                    checkStatus = 0
                elif time.time() - self.lastSendTime < 0.5 and checkStatus == 0:
                    print('开始自动控制模式！')
                    checkStatus = 1
            time.sleep(0.1)
    
    def recvCallback(self, data):
        bytesData = bytes(data[2:])
        self.recvBuffer += bytesData
        startPos = self.recvBuffer.find(b'\xAA\xAA')
        endPos = self.recvBuffer.find(b'\xAA\xAA', startPos+2)
        while endPos > startPos:
            dataBuffer = self.recvBuffer[startPos:endPos] # 判断功能字
            mark = dataBuffer[2]
            currentStruct = self.structDict.get(mark)
            if currentStruct != None:
                #####
                #print(currentStruct[0])
                #####
                currentStruct[1].Unpack(dataBuffer)
            self.recvBuffer = self.recvBuffer[endPos:]
            startPos = 0
            endPos = self.recvBuffer.find(b'\xAA\xAA', 2)
        
    def Start(self):
        self.recvBuffer = b''
        deviceList = hid.HidDeviceFilter(vendor_id = 0x0483, product_id = 0xa011).get_devices()
        if len(deviceList) == 0:
            raise NoRcConnectedException('未连接到遥控器！')
        self.device = deviceList[0]
        print(self.device)
        self.device.set_raw_data_handler(self.recvCallback)
        self.device.open()
        self.report = self.device.find_output_reports()[0]
        
    def Close(self):
        self.device.close()

    def _Send(self, data):
        # 发送间隔不应小于0.005s否则会丢帧
        # windows上的python最小延时时间是15ms。。。
        waitTime = 0.020 - (time.time()-self.lastSendTime)
        if waitTime > 0:
            time.sleep(waitTime)
        #print(time.time() - self.lastSendTime, waitTime)
        self.lastSendTime = time.time()
        
        sendData = bytes([0, len(data)]) + data + b'\x00'*(63-len(data))
        self.report.set_raw_data(sendData)
        self.report.send()

    def RequestCustomData(self):
        reqCmd = b'\xAA\xAF\x02\x01\xF2\x4E'
        self._Send(reqCmd)
        
    def SendControlData(self):
        # print(self.control.theta, self.control.yaw)
        self._Send(self.control.Pack())

    def SendCustomData(self, data):
        self.custom.sendData = data
        self._Send(self.custom.Pack())

    def SendTxData(self, data):
        self.customIO.TxData = data
        self._Send(self.customIO.PackTx())

    def SetGPIO(self, gpio, status):
        self._Send(self.customIO.PackIO(gpio, status))

    def GetRxData(self):
        RxData = self.customIO.RxData
        self.customIO.RxData = b''
        return RxData
        
    def RequestSHT3X(self):
        self.SendCustomData('$HT')

    def SetHeight(self, height):
        if height > 255:
            height = 255
        print('定高：{}'.format(height))
        #print(height, time.time())
        head = b'\xAA\xAF\xF5\x01'
        data = bytes([height])
        checkSum = bytes([sum(head + data) % 256])
        self._Send(head + data + checkSum)

    def MoveXY(self, dx, dy):
        print('移动：({}, {})'.format(dx, dy))
        distance = math.hypot(dx, dy)
        cx0 = 0
        cy0 = 0
        step = 10 # [cm]
        cur_distance = 0
        while cur_distance < distance:
            #print(self.sensor.flow_x, self.sensor.flow_y)
            ratio = cur_distance / distance
            cx1 = int(ratio * dx)
            cy1 = int(ratio * dy)
            head = b'\xAA\xAF\xF4\x02'
            data = struct.pack('=bb', cx1-cx0+50, cy1-cy0+50)
            #print(cx1-cx0, cy1-cy0)
            checkSum = bytes([sum(head + data) % 256])
            self._Send(head + data + checkSum)
            cur_distance += step
            cx0 = cx1
            cy0 = cy1
            #return
            self.Control(0.1)
        head = b'\xAA\xAF\xF4\x02'
        data = struct.pack('=bb', dx-cx0+50, dy-cy0+50)
        checkSum = bytes([sum(head + data) % 256])
        self._Send(head + data + checkSum)
        
    def Control(self, time_s):
        while time_s > 0:
            time_s -= 0.02
            self.SendControlData()
        self.SendControlData()
        
    def Unlock(self):
        # 先锁定，再解锁，防止多次Unlock的时候直接起飞！！
        # 锁定
        print('正在解锁飞控')
        self.control.__init__()
        self.control.theta = 1000
        self.control.yaw = 2000
        for i in range(50):
            self.SendControlData()
        # 解锁
        self.status.updateFlag = False
        self.control.theta = 1000
        self.control.yaw = 1500
        for i in range(100):
            self.SendControlData()
        self.control.theta = 2000
        self.control.yaw = 1500
        for i in range(50):
            self.SendControlData()
        self.control.theta = 1000
        self.control.yaw = 1500
        for i in range(50):
            self.SendControlData()
        if self.status.updateFlag and self.status.armed == 1:
            print('飞控解锁成功')
            self.setArmed = True
        else:
            print('飞控解锁失败')
        self.control.__init__()

    def Lock(self):
        print('正在锁定飞控')
        self.control.__init__()
        self.control.theta = 1000
        self.control.yaw = 2000
        self.status.updateFlag = False
        for i in range(50):
            self.SendControlData()
        if self.status.updateFlag and self.status.armed == 0:
            print('飞控锁定成功')
            self.setArmed = False
        else:
            print('飞控锁定失败，可关闭遥控器强行停止飞行！')
        self.control.__init__()

    def TakeOff(self, thr, height, offset_x = 0, offset_y = 0):
        print('起飞')
        self.control.theta = 1000 # 设定油门，最小为1000，最大为2000
        self.control.yaw = 1500   # 设定偏航，最小为1000，最大为2000，1500为无偏航
        self.control.roll = 1500 - offset_y  # 设定滚转，最小为1000，最大为2000，1500为无滚转
        self.control.pitch = 1500 + offset_x # 设定俯仰，最小为1000，最大为2000，1500为无俯仰
        self.control.AUX2 =  2000 # 设定飞行模式，1000为姿态控制，2000为定高定点控制
        
        self.control.theta = thr # 设置飞行油门

        self.Control(1)  # 推油门2秒达到一定高度

        if height < 20:
            height = 20

        self.SetHeight(height) # 设置起飞定高
        self.Control(3)  # 等5秒达到定高

    def Land(self):
        print('降落')
        self.SetHeight(0) # 设置降落高度
        
        t = 0
        # print('H: ', self.status.alt)
        print('H: ', self.sensor2.csb_alt)

        # while self.status.alt > 10 and t < 5:
        while self.sensor2.csb_alt > 10 and t < 5:
            self.Control(1)
            t += 1

        while self.control.theta > 1008:
            self.control.theta -= 1
            self.SendControlData()
        self.control.theta = 1000
        self.SendControlData()
        
        self.Lock() # 锁定落地
        
if __name__ == '__main__':
    import time
    controller = FlightController()
    controller.Start()
    
    controller.Unlock()
    controller.TakeOff(1500, 50, offset_x = -20, offset_y = 45)
    controller.SetHeight(70)
    controller.Control(5)
    controller.MoveXY(20,0)
    controller.Control(5)
    controller.MoveXY(0,20)
    controller.Control(5)
    controller.MoveXY(-20,0)
    controller.Control(5)
    controller.MoveXY(0,-20)
    controller.Control(5)
    controller.Land()
    controller.Lock()
    
##    while True:
##        time.sleep(0.5)
##        print(controller.sensor2.bar_alt, controller.sensor2.csb_alt, controller.status.alt)
##        print(controller.sensor.flow_x, controller.sensor.flow_y)
    
    
    

