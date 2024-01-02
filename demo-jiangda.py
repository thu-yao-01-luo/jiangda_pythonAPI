import cv2
from threading import Thread
# from globals import mode
import globals

from alarm_module import reminder
from control_module import control
from follow_module import follow_mode
from hand_module import hand_mode
from keyboard import keyboard_mode

def main():
    globals.mode = input("input the mode follow / hand / keyboard: ")
    t = Thread(target=control)
    t.start()
    warns = Thread(target=reminder)
    warns.start()
    keyboard_mode()

    while globals.mode != "quit":
        if globals.mode == "follow":
            follow_mode()
        elif globals.mode == "hand":
            hand_mode()
        elif globals.mode == "keyboard":
            continue
        else:
            raise NotImplementedError

    t.join()
    warns.join()
    print("The program exits!")

if __name__ == "__main__":
    main()