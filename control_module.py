from globals import debug, gpio, control_signal, controller, mode
import time

def control():
    global control_signal 
    global debug
    global gpio
    global controller
    global mode

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
        controller.Start()
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