from flight_controller import FlightController
import time
import threading

# 示例代码：
# 发送和接收一次自定义数据
# 此例程可用于测试温湿度传感器，带温湿度传感器的飞控程序逻辑是：
# 只要收到CustomData，就会回传一次温湿度传感器数据到controller.custom.recvData

# 打开遥控
controller = FlightController()
controller.Start()


while True:
    # 发送和接收一次自定义数据
    controller.SendCustomData(b'aabbcc') # 随便发点数据
    time.sleep(0.5)
    if controller.custom.updateFlag:
        print('custom data:', controller.custom.recvData)  # 显示接收的数据
        controller.custom.updateFlag = False
