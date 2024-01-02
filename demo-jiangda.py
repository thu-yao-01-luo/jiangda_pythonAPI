import cv2
from threading import Thread

from alarm import reminder
from control_module import control
from follow_module import follow_mode
from hand_module import hand_mode
from keyboard import keyboard_mode

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