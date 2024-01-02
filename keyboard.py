from pynput.keyboard import Key, Listener, KeyCode
from globals import control_signal, mode, gpio

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

    listener = Listener(
            on_press=on_press,
            on_release=on_release)
    listener.start()
    if mode == "quit" :
        listener.join()