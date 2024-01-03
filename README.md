# JiangDa(疆大) UAV PythonAPI #

This repo is implementation for JiangDa UAV in our course project of [Experience of Manufacturing Engineering](https://www.icenter.tsinghua.edu.cn/info/1035/2255.htm). 

## Install
The latest codes are tested on Windows 11, Python 3.9:
```shell
pip install -r requirements.txt
```

## Functions

```shell
python demo-jiangda.py
```
to test the functionalities including keyboard control, handpose control, detection and following. 

### Keyboard Mode

Press and release the key, one could enter specific mode or specific command. The mapping from key to mode / command 
is defined by `CONTROL_SIGNAL`, `KEYBOARD_COMMANDS`, `KEYBOARD_GPIOS`, `KEYBOARD_MODES` in `globals.py`. Customization 
on your own command mapping is welcomed! 

| Key | mode |
|--|--|
| k | keyboard mode |
| f | follow mode |
| h | hand mode |
| q | quit mode |

| Key | signal |
|--|--|
| w | forward |
| s | backward |
| a | left |
| d | right |
| o | up |
| p | down |
| j | open alarm |
| l | close alarm | 

### Handpose Mode

![handpose](https://github.com/thu-yao-01-luo/jiangda_pythonAPI/assets/87383739/95f52740-9d17-44b4-a16b-4ea6d20f39fd)

The handposes from left to right are `forward`, `backward`, `left`, `right`, `up`, `down`. The commands are defined by 
`gesture_recognition` in `utils.py`. Customization on your own command mapping is welcomed! 

### Follow Mode 

#### Alarm 

If there are more than 2 people or 1 bicycle or 1 car observed, the controller would set alarm open. **Customization of `alarm_module.py` is needed based on your GPIO!** The UAV would keep position when alarming. 

#### Follow

If there is only one person observed, the UAV would follow the movement of that person. Please check `follow_module.py` for more details. Pretrained YOLOv8 is adopted for detection.

### Quit Mode

Exit the main program. UAV will land and controller is closed. 

### FlightController Class
| Method | function |
|--|--|
| Start | start the flight controller |
| Close | close the flight controller |
| SetGPIO | set the voltage of GPIO in the main chip |
| SetHeight | set the flight height for the UAV |
| MoveXY | move UAV with dx and dy, dx > 0 means forward, right-handed system |
| Control | keep position |
| Lock | lock the controller, used after Land(), before Close() |
| Unlock | lock the controller, used after Start(), before TakeOff() |
| TakeOff | start the engine and keep the UAV at given height |
| Land | slow the engine and land the UAV over land |

### WifiCamera
| Method | function |
|--|--|
| Start | start the wifi camera on the UAV |
| Close | close the wifi camera on the UAV |

| Member | property |
|--|--|
| imgQueue | queue to receive the image from camera |
| maxQSize | queue size to cache the images |


## Reference By
[ultralytics with YOLOv8](https://github.com/ultralytics/ultralytics)<br>
[handpose detection](https://mediapipe.readthedocs.io/en/latest/solutions/hands.html)<br>
[keyboard monitoring with pynput](https://pynput.readthedocs.io/en/latest/keyboard.html)

## Acknowledge
[Zhiheng Zheng](https://github.com/zzh-thu) for `alarm_module.py` part, [Lunwei Zhang]() and [Zhiyuan Chen]() for `flight_controller.py` and `wifi_camera.py`. [Honzhi Zang](https://github.com/zanghz21) for handpose figure postprocessing.
