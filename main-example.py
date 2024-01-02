##########################################################
# 遥控例程
from flight_controller import FlightController
import time

# 打开遥控
controller = FlightController()
controller.Start()

time.sleep(1)

# 获取姿态信息
if controller.status.updateFlag:
    print('angle_rol:{}'.format(controller.status.angle_rol))
    print('angle_pit:{}'.format(controller.status.angle_pit))
    print('angle_yaw:{}'.format(controller.status.angle_yaw))
    print('alt:{}'.format(controller.status.alt)) # 高度
    print('fly_model:{}'.format(controller.status.fly_model))
    print('armed:{}'.format(controller.status.armed)) # 飞控是否解锁
    controller.status.updateFlag = False # 手动清除Flag

# 获IMU信息
if controller.sensor.updateFlag:
    # 加速度
    print('a_x:{}'.format(controller.sensor.a_x))
    print('a_y:{}'.format(controller.sensor.a_y))
    print('a_z:{}'.format(controller.sensor.a_z))
    # 角速度
    print('g_x:{}'.format(controller.sensor.g_x))
    print('g_y:{}'.format(controller.sensor.g_y))
    print('g_z:{}'.format(controller.sensor.g_z))
    # 光流
    print('flow_x:{}'.format(controller.sensor.flow_x))
    print('flow_y:{}'.format(controller.sensor.flow_y))
    print('flow_qual:{}'.format(controller.sensor.flow_qual))
    controller.sensor.updateFlag = False # 手动清除Flag

# 获电源信息
if controller.power.updateFlag:
    print('flight_votage:{}'.format(controller.power.flight_votage))
    print('controller_votage:{}'.format(controller.power.controller_votage))
    # 不明Flag
    print('flag0:{}'.format(controller.power.flag0))
    print('flag1:{}'.format(controller.power.flag1))
    print('flag2:{}'.format(controller.power.flag2))
    print('flag3:{}'.format(controller.power.flag3))
    controller.power.updateFlag = False # 手动清除Flag

# 获取速度信息
if controller.speed.updateFlag:
    print('rol_x:{}'.format(controller.speed.rol_x))
    print('rol_y:{}'.format(controller.speed.rol_y))
    print('speed_z:{}'.format(controller.speed.speed_z))
    controller.speed.updateFlag = False # 手动清除Flag

# 获取高度信息
if controller.sensor2.updateFlag:
    print('bar_alt:{}'.format(controller.sensor2.bar_alt)) # 气压计高度
    print('csb_alt:{}'.format(controller.sensor2.csb_alt))
    controller.sensor2.updateFlag = False # 手动清除Flag
    
# 发送用户自定义数据
controller.SendCustomData(b'aabbcc')
time.sleep(0.2)

# 获取SHT3X温湿度传感器数据（基于用户自定义数据实现；需要安装传感器）
controller.RequestSHT3X()
time.sleep(0.2)
if controller.custom.updateFlag_SHT3X:
    print('temperature:{}'.format(controller.custom.temperature)) # 温度
    print('humidity:{}'.format(controller.custom.humidity))       # 湿度

# 强制要求飞控发送自定义数据发送缓冲区中尚未发送的内容（一般不使用，通常发回的是空字符串，这里只为了演示下面的接收数据代码）
controller.RequestCustomData()
time.sleep(0.2)
# 检查接收到的用户自定义数据（在飞控程序中使用SendCustomData函数发回的数据）
if controller.custom.updateFlag:
    print('custom data:', controller.custom.recvData)
    controller.custom.updateFlag = False

# 控制GPIO
# GPIO_S/GPIO_K/GPIO_I/GPIO_O分别为飞机主板上闲置的SPI2接口的四个引脚
controller.SetGPIO(FlightController.GPIO_S, FlightController.HIGH)
controller.SetGPIO(FlightController.GPIO_K, FlightController.HIGH)
controller.SetGPIO(FlightController.GPIO_I, FlightController.HIGH)
controller.SetGPIO(FlightController.GPIO_O, FlightController.HIGH)
time.sleep(1)
controller.SetGPIO(FlightController.GPIO_S, FlightController.LOW)
controller.SetGPIO(FlightController.GPIO_K, FlightController.LOW)
controller.SetGPIO(FlightController.GPIO_I, FlightController.LOW)
controller.SetGPIO(FlightController.GPIO_O, FlightController.LOW)

# 串口发送（Uart2）
controller.SendTxData(b'hello')
time.sleep(1)

# 串口接收（Uart2）
print('RX data:', controller.GetRxData())

# 飞行控制
# 【【飞行控制前应先通过短按遥控器上的右侧边按键，将遥控器切换到[PC MODE]】】

## 【【下面注释掉的这段代码请在确保安全的前提下调试，这段代码会控制飞机的桨叶转动！！！】】
## 【紧急停止的4种方法】
## 1.关闭PC端程序；
## 2.短按遥控器上的右侧边按键切换到手动模式；
## 3.拔掉遥控器的连接线；
## 4.关闭遥控器开关
##controller.Unlock() # 解锁飞控
##controller.TakeOff(1500, 50, offset_x = -20, offset_y = 45)  # 起飞，设置起飞油门，x、y操作偏置
##controller.SetHeight(70)    # 设置定高高度
##controller.Control(5)       # 停留5s
##controller.MoveXY(20,0)     # 移动指令dx = 20, dy = 0
##controller.Control(5)       # 停留5s
##controller.MoveXY(0,20)     # 移动指令
##controller.Control(5)       # 停留5s
##controller.MoveXY(-20,0)    # 移动指令
##controller.Control(5)       # 停留5s
##controller.MoveXY(0,-20)    # 移动指令
##controller.Control(5)       # 停留5s
##controller.Land()           # 降落
##controller.Lock()           # 锁定飞控
## 注意：实际使用中控制数据必须实时发送
## 若超过0.8s未发送控制数据，遥控器自动将切换为手动控制，直到再次收到控制数据
## 【【调试前请确保遥控器的油门处于低位！！】】

##########################################################
# 相机例程，需先连接飞机相机的wifi，名称为WIFI_4K_XXXXXX格式
from wifi_camera import WifiCamera
import cv2

# 打开相机
camera = WifiCamera()
camera.maxQSize = 5 # 设置缓存不超过5帧，避免图像处理不及时导致数据堆积
camera.Start()

cv2.namedWindow('img')
while True:
    # 接收到的图像在队列camera.imgQueue中
    if not camera.imgQueue.empty():
        # 从队列中取出图像并显示
        img = camera.imgQueue.get()
        cv2.imshow('img', img)
        cv2.waitKey(10)
    else:
        cv2.waitKey(10)

