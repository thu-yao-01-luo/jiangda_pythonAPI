import cv2
import numpy as np
from ultralytics import YOLO
from ultralytics.utils import ASSETS
from ultralytics.models.yolo.detect import DetectionPredictor
from globals import debug, mode, control_signal, gpio

from wifi_camera import WifiCamera

# load a model
model_path = "yolov8n.pt"
model = YOLO(model_path)  # load a pretrained model (recommended for training)

# predictor args
args = dict(model=model_path, source=ASSETS)
args['save'] = False
args['show'] = True
# verbose=True would print a lot in your terminal
args['verbose'] = False

# set a predictor
predictor = DetectionPredictor(overrides=args)

def follow_mode():
    global debug
    global mode
    global control_signal
    global gpio

    thr_x = 0.10
    thr_y = 0.10
    thr_too_wide = 0.65
    thr_too_narrow = 0.4
    center_x = 0.5
    center_y = 0.6
    follow_debug = False
    thr_boundary = 0.05
    dof_follow = 6 # or 4

    print("Follow mode activated.")
    if follow_debug:
        img_path = "./example.jpg"
        img = cv2.imread(img_path)
        results = model(source=img) # get results list
        predictor(source=img, model=model_path, stream=False,) # show the result in cv2 windows
        result = results[0]
        boxes = result.boxes

        is_person = (boxes.cls == 0)
        is_bicycle = (boxes.cls == 1)
        is_car = (boxes.cls == 2)
        
        print(results) # https://docs.ultralytics.com/modes/predict/#working-with-results
        print(result.names)
        print(result.orig_shape)
        print(result)
        print(boxes) # https://docs.ultralytics.com/modes/predict/#boxes

        xyxy = boxes.xyxy[is_person]
        xyxyn = boxes.xyxyn[is_person]
        xywh = boxes.xywh[is_person]
        xywhn = boxes.xywhn[is_person]

        print(xyxy) # not normalized, 4 corner points
        print(xyxyn) # normalized 
        print(xywh) # center x, y and width and height
        print(xywhn) # normalized

        if is_person.sum() > 1 or is_bicycle.sum() > 1 or is_car.sum() > 0: # more than 1 person or 1 bike or car appears
            print("alert and set gpio!")
            gpio = True
        else:
            print("no alert")
            gpio = False
            
        while mode == "follow":
            cv2.waitKey(10)
        return
    else:
        # 打开相机
        camera = WifiCamera()
        camera.maxQSize = 5 # 设置缓存不超过5帧，避免图像处理不及时导致数据堆积
        camera.Start()

        cv2.namedWindow('img')
        while mode == "follow":
            # 接收到的图像在队列camera.imgQueue中
            if not camera.imgQueue.empty():
                # 从队列中取出图像并显示
                img = camera.imgQueue.get()
                img = np.flip(img, 0)
                img = np.flip(img, 1)
                results = model(source=img) # get results list
                result = results[0]
                boxes = result.boxes
                # print(results) # https://docs.ultralytics.com/modes/predict/#working-with-results
                # print(results.names)
                # print(result)
                # print(boxes) # https://docs.ultralytics.com/modes/predict/#boxes
                is_person = (boxes.cls == 0)
                is_bicycle = (boxes.cls == 1)
                is_car = (boxes.cls == 2)
                if is_person.sum() > 1 or is_bicycle.sum() > 1 or is_car.sum() > 0: # more than 1 person or 1 bike or car appears
                    # print("alert and set gpio!")
                    gpio = True
                    control_signal = "control"
                elif is_person.sum() == 1:
                    # xyxy = boxes.xyxy[is_person] # https://docs.ultralytics.com/modes/predict/#boxes
                    # xyxyn = boxes.xyxyn[is_person]
                    # xywh = boxes.xywh[is_person]
                    # print("xywhn", boxes.xywhn)
                    # print("is_person", is_person, len(is_person))
                    # print("results", boxes.xywhn[is_person])

                    if len(is_person) > 1:
                        xywhn = boxes.xywhn[is_person] # 2 dim
                    else:
                        xywhn = boxes.xywhn # 2 dim

                    # print("before xywhn[0]", xywhn)
                    xywhn = xywhn[0] # 1 dim

                    # print("after xywhn[0]", xywhn)

                    # print("results:", results)  # https://docs.ultralytics.com/modes/predict/#working-with-results
                    # print("names:", result.names)
                    # print("shape: ", result.orig_shape)
                    # print("result:", result)
                    # print("boxes:", boxes)
                    gpio = False
                    x = xywhn[0]
                    y = xywhn[1]
                    w = xywhn[2]
                    h = xywhn[3]

                    # TODO: Error
                    #             "D:\user\desktop\jiangda\jiangda\uav\PythonAPI\jiangda-uav.py", line
                    #             316, in follow_mode
                    #             x = xywhn[0]
                    # IndexError: index 0 is out  of  bounds  for dimension 0 with size 0

                    priority = 0
                    x_bias = abs(x - center_x)
                    y_bias = abs(y - center_y)
                    w_bias = max(w - thr_too_wide, thr_too_narrow - w)
                    h_bias = 1 - h
                    y_high = y - h / 2
                    y_low = y + h / 2
                    x_high = x + w / 2
           
                    x_low = x - w / 2

                    if (w_bias > x_bias) and (w_bias > y_bias) or ((x_low < thr_boundary) and (x_high > 1 - thr_boundary)):
                        priority = 3
                    elif (x_bias > y_bias) and (x_bias > w_bias) or (x_low < thr_boundary) or (x_high > 1 - thr_boundary):
                        priority = 1
                    elif (y_bias > x_bias) and (y_bias > w_bias):
                        priority = 2
                    else:
                        priority = 0

                    # if abs(x - 0.5) < abs(y - 0.5):
                    if priority == 2 and dof_follow == 6:
                        if y < center_y - thr_y:
                            # control_signal = "backward"
                            control_signal = "up"
                        elif y > center_y + thr_y:
                            # control_signal = "forward"
                            control_signal = "down"
                        else:
                            control_signal = "control"
                    # else:
                    elif priority == 1:
                        if x < center_x - thr_x or x_low < thr_boundary:
                            control_signal = "left"
                        elif x > center_x + thr_x or x_high > 1 - thr_boundary:
                            control_signal = "right"
                        else:
                            control_signal = "control"
                    elif priority == 3 and dof_follow == 6:
                        if w > thr_too_wide or (x_low < thr_boundary) and (x_high > 1 - thr_boundary):
                            control_signal = "backward"
                        elif w < thr_too_narrow:
                            control_signal = "forward"
                        else:
                            print("Something goes wrong!")
                            control_signal = "control"
                    elif priority == 3 and dof_follow == 4 or priority == 2 and dof_follow == 4:
                        if w > thr_too_wide or (x_low < thr_boundary) and (x_high > 1 - thr_boundary) or (y_high < thr_boundary) or (abs(y - center_y) > thr_y):
                            control_signal = "backward"
                        elif w < thr_too_narrow:
                            control_signal = "forward"
                        else:
                            control_signal = "control"
                    else:
                        control_signal = "control"


                predictor(source=img, model=model_path, stream=False,) # show the result in cv2 windows
            else:
                cv2.waitKey(10)
        camera.Close()
        return