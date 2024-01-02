from globals import control_signal, mode
import mediapipe as mp
import numpy as np
import time
import cv2
from utils import gesture_recognition

mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_hands = mp.solutions.hands

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
