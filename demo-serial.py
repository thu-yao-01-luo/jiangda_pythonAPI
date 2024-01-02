from flight_controller import FlightController
import time
import threading

# 示例代码：
# 将控制台输入的字符串发送到飞机的Uart2串口的Tx，并显示Uart2串口Rx接收到的数据

def CheckFunc():
    while True:
        RxData = controller.GetRxData() # 获取串口数据
        if RxData != b'': # 如果数据不为空
            recvString = RxData.decode('ascii') # 将接收的二进制数据按ascii解码为字符串
            print(recvString)
        time.sleep(0.2)
        
# 打开遥控
controller = FlightController()
controller.Start()

checkThread = threading.Thread(target = CheckFunc, args=())
checkThread.setDaemon(True)
checkThread.start()  # 打开轮询串口接收信息的线程

while True:
    inp = input('>->')
    if inp != '':
        sendString = inp + '\n'
        sendBytes = sendString.encode('ascii') # 将字符串按acsii编码为二进制数据
        controller.SendTxData(sendBytes)       # 发送二进制数据
