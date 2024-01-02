from flight_controller import FlightController
####### no flight_controller in mac ###########
import time
import cv2
from threading import Thread

from pynput.keyboard import Key, Listener, KeyCode

from wifi_camera import WifiCamera
from ultralytics import YOLO
from ultralytics.utils import ASSETS
from ultralytics.models.yolo.detect import DetectionPredictor

# from handcontrol import mp, gesture_recognition, mp_hands, mp_drawing_styles, mp_drawing
import mediapipe as mp
import numpy as np
from mediapipe.framework.formats import landmark_pb2

# load a model
model_path = "yolov8n.pt"
model = YOLO(model_path)  # load a pretrained model (recommended for training)
# predictor args
args = dict(model='yolov8n.pt', source=ASSETS)
args['save'] = False
args['show'] = True
# args['verbose'] = False
# set a predictor
predictor = DetectionPredictor(overrides=args)

# Use the model
mode = "follow" # "hand" or "keyboard" or "quit"
control_signal = "control" # "forward" "backward" "left" "right" Optional: "up" "down"
gpio = False
debug = False # if debug, no control, print control signal instead
# debug = True
dof_follow = 6

mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_hands = mp.solutions.hands

controller = FlightController()
controller.Start()
        
def vec_cos(vec1, vec2):
    return np.dot(vec1, vec2) / (np.linalg.norm(vec1) * np.linalg.norm(vec2))

def is_line(vec_list, threshold = 0.8,):
    if vec_list is not None:
        for i in range(len(vec_list) - 1):
            # if np.cos(vec_list[i], vec_list[i+1]) < threshold:
            if vec_cos(vec1=vec_list[i], vec2=vec_list[i+1]) < threshold:
                return False
        return True 
    raise NotImplementedError

def gesture_recognition(hand_landmarks):
    lm = hand_landmarks.landmark  # Assuming hand_landmarks is the list of landmarks
    thr = 0.1
    xy_ratio_thr = 1.2
    yx_ratio_thr = 2
    eps = 1e-3
    cos_thr = 0.8

    feature_pt = np.array([(lm[8].x + lm[12].x) / 2, (lm[8].y + lm[12].y) / 2])
    wrist_pt = np.array([lm[0].x, lm[0].y])
    feature_vec = feature_pt - wrist_pt
    xy_feature_ratio = np.abs(feature_vec[0]) / (np.abs(feature_vec[1]) + eps)
    yx_feature_ratio = np.abs(feature_vec[1]) / (np.abs(feature_vec[0]) + eps)
    
    # Calculate vectors
    # vec01 = np.array([lm[1].x - lm[0].x, lm[1].y - lm[0].y])

    # vec02 = np.array([lm[2].x - lm[0].x, lm[2].y - lm[0].y])
    vec23 = np.array([lm[3].x - lm[0].x, lm[3].y - lm[0].y])
    vec34 = np.array([lm[4].x - lm[0].x, lm[4].y - lm[0].y])
    # thumb = [vec01, vec02, vec03, vec04]
    # thumb = [vec02, vec23, vec34]
    thumb = [vec23, vec34]

    # vec05 = np.array([lm[5].x - lm[0].x, lm[5].y - lm[0].y])
    vec56 = np.array([lm[6].x - lm[5].x, lm[6].y - lm[5].y])
    vec68 = np.array([lm[8].x - lm[6].x, lm[8].y - lm[6].y])
    # pointer = [vec05, vec56, vec68]
    pointer = [vec56, vec68]

    # vec09 = np.array([lm[9].x - lm[0].x, lm[9].y - lm[0].y])
    vec9_10 = np.array([lm[10].x - lm[9].x, lm[10].y - lm[9].y])
    vec10_12 = np.array([lm[12].x - lm[10].x, lm[12].y - lm[10].y])

    # middle = [vec09, vec9_10, vec10_12]
    middle = [vec9_10, vec10_12]

    # vec0_13 = np.array([lm[13].x - lm[0].x, lm[13].y - lm[0].y])
    vec13_14 = np.array([lm[14].x - lm[13].x, lm[14].y - lm[13].y])
    vec14_16 = np.array([lm[16].x - lm[14].x, lm[16].y - lm[14].y])

    # ring = [vec0_13, vec13_14, vec14_16]
    ring = [vec13_14, vec14_16]
    
    # vec0_17 = np.array([lm[17].x - lm[0].x, lm[17].y - lm[0].y])
    vec17_18 = np.array([lm[18].x - lm[17].x, lm[18].y - lm[17].y])
    vec18_20 = np.array([lm[20].x - lm[18].x, lm[20].y - lm[18].y])

    # pinky = [vec0_17, vec17_18, vec18_20]
    pinky = [vec17_18, vec18_20]
    
    thumb_is_line = is_line(thumb, cos_thr)
    pointer_is_line = is_line(pointer, cos_thr)
    middle_is_line = is_line(middle, cos_thr)
    ring_is_line = is_line(ring, cos_thr)
    pinky_is_line = is_line(pinky, cos_thr)

    if ring_is_line and pinky_is_line:
        if thumb_is_line and pointer_is_line and middle_is_line:
            return "forward"
    else:
        if pointer_is_line and middle_is_line:
            if xy_feature_ratio > xy_ratio_thr: 
                # left or right
                return "left" if feature_vec[0] > 0 else "right"
            if yx_feature_ratio > yx_ratio_thr:
                # up or down 
                return "up" if feature_vec[1] < 0 else "down"

        elif not pointer_is_line and not middle_is_line:
            return "backward"
    return "control"

def reminder():
    global gpio
    global controller
    # 具体电平由接线决定fgbr
    while True:
        if gpio:
            controller.SetGPIO(FlightController.GPIO_S, FlightController.HIGH)
            controller.SetGPIO(FlightController.GPIO_K, FlightController.HIGH)
            controller.SetGPIO(FlightController.GPIO_I, FlightController.HIGH)
            controller.SetGPIO(FlightController.GPIO_O, FlightController.LOW)
            time.sleep(1)
            controller.SetGPIO(FlightController.GPIO_S, FlightController.LOW)
            controller.SetGPIO(FlightController.GPIO_K, FlightController.HIGH)
            controller.SetGPIO(FlightController.GPIO_I, FlightController.HIGH)
            controller.SetGPIO(FlightController.GPIO_O, FlightController.HIGH)
        else:
            controller.SetGPIO(FlightController.GPIO_S, FlightController.LOW)
            controller.SetGPIO(FlightController.GPIO_K, FlightController.LOW)
            controller.SetGPIO(FlightController.GPIO_I, FlightController.HIGH)
            controller.SetGPIO(FlightController.GPIO_O, FlightController.HIGH)

def control():
    global control_signal 
    global debug
    global gpio
    global controller

    init_height = 90 # the initial flight height
    height = init_height # flight height for up or down
    init_wait = 5 # initial control waiting time
    wait = 1 # control waiting time
    wait_long = 10
    step_size = 30
    height_size = 10
    height_lower_bound = 40

    if debug:
        while True:
            if control_signal == "control":
                print("control executed!")            
                time.sleep(2)
            elif control_signal == "forward":
                print("go forward!")
                control_signal = "control"
                time.sleep(2)
            elif control_signal == "backward":
                print("go backward")
                control_signal = "control"
                time.sleep(2)
            elif control_signal == "left":
                print("go left")
                control_signal = "control"
                time.sleep(2)
            elif control_signal == "right":
                print("go right")
                control_signal = "control"
                time.sleep(2)
            elif control_signal == "up":
                print("go up")
                control_signal = "control"
                time.sleep(2)
            elif control_signal == "down":
                print("go down")
                control_signal = "control"
                time.sleep(2)
            else: 
                raise NotImplementedError
            if gpio:
                print("warning!")            
                time.sleep(2)
    else:
        # time gap
        time.sleep(1)
        # get preparation
        controller.Unlock() # 解锁飞控
        controller.TakeOff(1700, 80, offset_x = -20, offset_y = 45)  # 起飞，设置起飞油门，x、y操作偏置
        controller.SetHeight(init_height)    # 设置定高高度
        controller.Control(init_wait)       # 停留5s

        while True:
            if mode == "follow":
                wait = 1
                step_size = 15
                height_size = 5
                controller.SetHeight(height)
                controller.Control(wait)
            else:
                wait = 0.5
                step_size = 30
                height_size = 10


            if control_signal == "control":
                # print("controlling!")
                controller.Control(wait)
            elif control_signal == "forward":
                print("forward move")
                controller.MoveXY(dx = step_size, dy = 0)
                control_signal = "control"
            elif control_signal == "backward":
                print("backward move")
                controller.MoveXY(dx = -step_size, dy = 0)
                control_signal = "control"
            elif control_signal == "left":
                print("left move")
                controller.MoveXY(dx = 0, dy = step_size)
                control_signal = "control"
            elif control_signal == "right":
                print("right move")
                controller.MoveXY(dx = 0, dy = -step_size)
                control_signal = "control"
            elif control_signal == "up": # Not sure
                print("up move")
                height += height_size
                controller.SetHeight(height)
                control_signal = "control"
            elif control_signal == "down": # Not sure
                print("down move")
                height -= height_size
                if height > height_lower_bound:
                    controller.SetHeight(height)
                    control_signal = "control"
                else:
                    height += height_size
                    print("too low to go down!")
                    controller.Control(wait)
                    control_signal = "control"
            elif control_signal == "quit":
                print("close controller")
                controller.Land()           # 降落
                controller.Lock()           # 锁定飞控
                return
            elif control_signal == "shift":
                print("waiting for shifting mode")
                controller.Control(wait_long)
                control_signal = "control"
            else:
                print("control error!")
                controller.Land()           # 降落
                controller.Lock()           # 锁定飞控
                raise NotImplementedError
            if mode == "follow":
                controller.Control(wait)

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
       
def hand_mode(init_height=100, init_wait=5):
    global control_signal
    global mode
    if mode == "hand":
        print("Hand-Control mode activated.")

        cap = cv2.VideoCapture(0)
        with mp_hands.Hands(
                model_complexity=0,
                min_detection_confidence=0.5,
                min_tracking_confidence=0.5) as hands:
            while cap.isOpened() and mode == "hand": # check current mode!
                success, image = cap.read()
                if not success:
                    print("Ignoring empty camera frame.")
                    # If loading a video, use 'break' instead of 'continue'.
                    continue
                # To improve performance, optionally mark the image as not writeable to
                # pass by reference.
                image.flags.writeable = False
                image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                results = hands.process(image)

                image.flags.writeable = True
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
                if results.multi_hand_landmarks:
                    for hand_landmarks in results.multi_hand_landmarks:
                        gesture = gesture_recognition(hand_landmarks=hand_landmarks)
                        if gesture != "other":
                            print(f"capture {gesture}")
                            control_signal = gesture
                        mp_drawing.draw_landmarks(
                            image,
                            hand_landmarks,
                            mp_hands.HAND_CONNECTIONS,
                            mp_drawing_styles.get_default_hand_landmarks_style(),
                            mp_drawing_styles.get_default_hand_connections_style())
                        time.sleep(1) 
                cv2.imshow('MediaPipe Hands', cv2.flip(image, 1))
                if cv2.waitKey(20) & 0xFF == 27:
                    break
        cap.release()
    else:
        print("hand end!")
        return

def keyboard_mode(init_height=100, init_wait=5):
    print("Keyboard-Control mode activated.")

    def on_press(key):
        return

    def on_release(key):
        global control_signal
        global mode
        global gpio

        if key == Key.esc:
            # Stop listener
            mode = "quit"
            control_signal = "quit"
            return False
        if isinstance(key, KeyCode):
            if key.char == "f":
                mode = "follow"
                control_signal = "shift"
            elif key.char == "h":
                mode = "hand"
                control_signal = "shift"
            elif key.char == "k":
                mode = "keyboard"
                control_signal = "shift"
            if key.char == "j":
                gpio = True
            elif key.char == "k":
                gpio = False
            if mode == "keyboard":
                if key.char == 'a':
                    control_signal = "left"
                if key.char == 's':
                    control_signal = "backward"
                if key.char == 'd':
                    control_signal = "right"
                if key.char == 'w':
                    control_signal = "forward"
                if key.char == 'o':
                    control_signal = "up"
                if key.char == 'p':
                    control_signal = "down"
    # Collect events until released
    # with Listener(
    #         on_press=on_press,
    #         on_release=on_release) as listener:
    #     listener.join()
    listener = Listener(
            on_press=on_press,
            on_release=on_release)
    listener.start()
    if mode == "quit" :
        listener.join()


if __name__ == "__main__":
    mode = input("input the mode follow / hand / keyboard: ")
    t = Thread(target=control)
    t.start()
    warns = Thread(target=reminder)
    warns.start()
    keyboard_mode()
    while mode != "quit":
        if mode == "follow":
            follow_mode()
        elif mode == "hand":
            hand_mode()
        elif mode == "keyboard":
            continue
        else:
            raise NotImplementedError
    t.join()
    warns.join()
    print("The program exits!")