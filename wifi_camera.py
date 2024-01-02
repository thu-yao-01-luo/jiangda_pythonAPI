import numpy as np
import queue
import cv2
import socket
import time
import struct
import threading

class UDP_Manager:
    def __init__(self, callback, isServer = False, ip = '', port = 8083, frequency = 50, inet = 4):
        self.callback = callback
        
        self.isServer = isServer
        self.interval = 1.0 / frequency

        self.available_addr = socket.getaddrinfo(socket.gethostname(), port)
        self.hostname = socket.getfqdn(socket.gethostname())

        self.inet = inet
        self.af_inet = None
        self.ip = ip
        self.localIp = None
        self.port = port
        self.addr = (self.ip, self.port)
        self.running = False

        #self.serialNum = 0

        #self.recvPools = {} #{'IP:PORT': [{serialNum:[data..., recvNum, packetNum, timestamp]}]}

    def Start(self):
        if self.inet == 4:
            self.af_inet = socket.AF_INET  # ipv4
            self.localIp = '127.0.0.1'
        elif self.inet == 6:
            self.af_inet = socket.AF_INET6 # ipv6
            self.localIp = '::1'
        self.sockUDP = socket.socket(self.af_inet, socket.SOCK_DGRAM)

        if self.isServer:
            self.roleName = 'Server'
        else:
            self.port = 0
            self.roleName = 'Client'
        
        self.sockUDP.bind((self.ip, self.port))
        self.addr = self.sockUDP.getsockname()
        self.ip = self.addr[0]
        self.port = self.addr[1]
        
        print(self.roleName, '(UDP) at:', self.ip, ':', self.port)
            
        self.running = True
        self.thread = threading.Thread(target = self.Receive, args=())
        self.thread.setDaemon(True)
        self.thread.start()  #打开收数据的线程
        
    def ListAddr(self):
        for item in self.available_addr:
            if item[0] == self.af_inet:
                print(item[4])
    
    def Receive(self):
        print(self.roleName, '(UDP) is running...\n')
        while self.running:
            time.sleep(self.interval)
            while self.running:
                try:
                    recvData, recvAddr = self.sockUDP.recvfrom(65535) #等待接受数据
                    #print(recvData)
                except:
                    break
                if not recvData:
                    break
                #print(recvData)
                self.callback(recvData, recvAddr)
            
    def Send(self, data, addr):
        self.sockUDP.sendto(data, addr)
        #print(data, addr)

    def Close(self):
        self.running = False

class WifiCamera:
    def __init__(self):
        self.cameraAddr = ('192.168.4.153', 8080)
        self.udp = UDP_Manager(self.RecvCallback)
        self.imgQueue = queue.Queue()
        self.maxQSize = 5

        self.frameIndex = -1
        self.pkgCounter = 0
        self.pkgNum = -1
        
    def RecvCallback(self, data, addr):
        error = 0
        if self.frameIndex == -1:
            self.pkgNum = data[2]
            self.frameIndex = data[0]
            self.recvBuffer += data[8:]
            self.pkgCounter = 1
        elif self.frameIndex == data[0]:
            self.recvBuffer += data[8:]
            self.pkgCounter += 1

            if data[1] == 0x01: # 帧尾包
                if self.pkgNum == self.pkgCounter: # 包数正确
                    img = cv2.imdecode(np.frombuffer(self.recvBuffer, np.uint8), cv2.IMREAD_COLOR)
                    if not img is None:
                        self.imgQueue.put(img)
                        if self.imgQueue.qsize() > self.maxQSize:
                            self.imgQueue.get()
                        self.frameIndex = -1
                        self.pkgCounter = 0
                        self.recvBuffer = b''
                    else:
                        error = 3
                    
                else:
                    error = 2
        else:
            error = 1

        if error != 0:
            self.frameIndex = -1
            self.pkgCounter = 0
            self.recvBuffer = b''
            #print('bad image packet. (%d)' % error)

    def Start(self):
        self.recvBuffer = b''
        self.udp.Start()
        self.thread2 = threading.Thread(target = self.Send, args=())
        self.thread2.setDaemon(True)
        self.thread2.start()  #打开收数据的线程
    def Close(self):
        self.udp.Close()

    def Send(self):
        while True:
            self.udp.Send(b'\x42\x76', self.cameraAddr)
            time.sleep(2)


if __name__ == '__main__':
    camera = WifiCamera()
    camera.Start()
    cv2.namedWindow('img')
    while True:
        if not camera.imgQueue.empty():
            img = camera.imgQueue.get()
            cv2.imshow('img', img)
            cv2.waitKey(10)
        else:
            cv2.waitKey(10)

