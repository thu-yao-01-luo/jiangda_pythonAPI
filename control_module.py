# from globals import debug, contro_signal, mode
# if not debug:
#     from globals import controller
from globals import debug
import globals
import time

def control():
    # global contro_signal 
    # global debug
    # global mode

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
            if globals.control_signal == "control":
                print("control executed!")            
                time.sleep(2)
            elif globals.control_signal == "forward":
                print("go forward!")
                globals.control_signal = "control"
                time.sleep(2)
            elif globals.control_signal == "backward":
                print("go backward")
                globals.control_signal = "control"
                time.sleep(2)
            elif globals.control_signal == "left":
                print("go left")
                globals.control_signal = "control"
                time.sleep(2)
            elif globals.control_signal == "right":
                print("go right")
                globals.control_signal = "control"
                time.sleep(2)
            elif globals.control_signal == "up":
                print("go up")
                globals.control_signal = "control"
                time.sleep(2)
            elif globals.control_signal == "down":
                print("go down")
                globals.control_signal = "control"
                time.sleep(2)
            elif globals.control_signal == "shift":
                print("waiting for shifting mode")
                globals.control_signal = "control"
            else: 
                raise NotImplementedError
            if globals.mode == "quit":
                print("close controller")
                return
    else:
        # global controller
        globals.controller.Start()
        # time gap
        time.sleep(1)
        # get preparation
        globals.controller.Unlock() # 解锁飞控
        globals.controller.TakeOff(1700, 80, offset_x = -20, offset_y = 45)  # 起飞，设置起飞油门，x、y操作偏置
        globals.controller.SetHeight(init_height)    # 设置定高高度
        globals.controller.Control(init_wait)       # 停留5s

        while True:
            if globals.mode == "follow":
                wait = 1
                step_size = 15
                height_size = 5
                globals.controller.SetHeight(height)
                globals.controller.Control(wait)
            else:
                wait = 0.5
                step_size = 30
                height_size = 10


            if globals.control_signal == "control":
                # print("controlling!")
                globals.controller.Control(wait)
            elif globals.control_signal == "forward":
                print("forward move")
                globals.controller.MoveXY(dx = step_size, dy = 0)
                globals.control_signal = "control"
            elif globals.control_signal == "backward":
                print("backward move")
                globals.controller.MoveXY(dx = -step_size, dy = 0)
                globals.control_signal = "control"
            elif globals.control_signal == "left":
                print("left move")
                globals.controller.MoveXY(dx = 0, dy = step_size)
                globals.control_signal = "control"
            elif globals.control_signal == "right":
                print("right move")
                globals.controller.MoveXY(dx = 0, dy = -step_size)
                globals.control_signal = "control"
            elif globals.control_signal == "up": # Not sure
                print("up move")
                height += height_size
                globals.controller.SetHeight(height)
                globals.control_signal = "control"
            elif globals.control_signal == "down": # Not sure
                print("down move")
                height -= height_size
                if height > height_lower_bound:
                    globals.controller.SetHeight(height)
                    globals.control_signal = "control"
                else:
                    height += height_size
                    print("too low to go down!")
                    globals.controller.Control(wait)
                    globals.control_signal = "control"
            elif globals.control_signal == "shift":
                print("waiting for shifting mode")
                globals.controller.Control(wait_long)
                globals.control_signal = "control"
            else:
                print("control error!")
                globals.controller.Land()           # 降落
                globals.controller.Lock()           # 锁定飞控
                raise NotImplementedError
            if globals.mode == "follow":
                globals.controller.Control(wait)
            elif globals.mode == "quit":
                print("close globals.controller")
                globals.controller.Land()           # 降落
                globals.controller.Lock()           # 锁定飞控
                return