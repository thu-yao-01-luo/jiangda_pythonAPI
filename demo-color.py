from flight_controller import FlightController
from wifi_camera import WifiCamera
import numpy as np
import cv2

# 示例：
# 根据相机中心区域的颜色是否为红色，控制蜂鸣器是否鸣叫

def AverageColor_HSV(img):
    # 计算HSV颜色空间下img的平均颜色
    # H:色相（0~180）  S:饱和度（0~255）  V:明度（0~255）
    img_H, img_S, img_V = cv2.split(img.astype(np.float64))
    img_H_rad = img_H*2/180*np.pi
    img_H_x = np.cos(img_H_rad).mean()
    img_H_y = np.sin(img_H_rad).mean()
    mean_H = np.arctan2(img_H_y, img_H_x)/np.pi*180/2
    if mean_H < 0:
        mean_H += 180
    mean_S = img_S.mean()
    mean_V = img_V.mean()
    return [mean_H, mean_S, mean_V]

# 打开遥控
controller = FlightController()
controller.Start()

# 打开相机
camera = WifiCamera()
camera.maxQSize = 5 # 设置缓存不超过5帧，避免图像处理不及时导致数据堆积
camera.Start()

cv2.namedWindow('img')

buzzingFlag = False # 蜂鸣器是否鸣叫

while True:
    # 接收到的图像在队列camera.imgQueue中
    if not camera.imgQueue.empty():
        # 从队列中取出图像
        img = camera.imgQueue.get()
        # 感兴趣区域大小
        ROI_size = 10
        imgSize = img.shape
        imgCenter = [int(imgSize[0]/2), int(imgSize[1]/2)]
        # 从图像中心取感兴趣的区域(ROI)
        ROI_BGR = img[(imgCenter[0]-ROI_size):(imgCenter[0]+ROI_size),(imgCenter[1]-ROI_size):(imgCenter[1]+ROI_size)]

        # 将ROI转换到HSV色彩空间
        ROI_HSV = cv2.cvtColor(ROI_BGR, cv2.COLOR_BGR2HSV)
        mean_H, mean_S, mean_V = AverageColor_HSV(ROI_HSV)
        print(mean_H, mean_S, mean_V)
        #(mean_H > 170 or mean_H < 10) 红色
        if (mean_H > 120 or mean_H < 60) and mean_S > 60 and mean_V > 130: # 图像中心为红色
            if buzzingFlag == False:
                # 蜂鸣器鸣叫（这里使用的是丝印为O的GPIO_O）
                controller.SetGPIO(FlightController.GPIO_O, FlightController.HIGH)
                buzzingFlag = True
        else: # 图像中心不是红色
            if buzzingFlag == True:
                # 蜂鸣器关闭
                controller.SetGPIO(FlightController.GPIO_O, FlightController.LOW)
                buzzingFlag = False

        cv2.rectangle(img,
                      ((imgCenter[1]-ROI_size), (imgCenter[0]-ROI_size)),
                      ((imgCenter[1]+ROI_size), (imgCenter[0]+ROI_size)),
                      [0,0,255], 2)
        cv2.imshow('img', img)
        cv2.waitKey(10)
    else:
        cv2.waitKey(10)
