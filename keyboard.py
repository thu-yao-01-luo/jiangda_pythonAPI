from pynput.keyboard import Key, Listener, KeyCode
from globals import KEYBOARD_MODES, KEYBOARD_COMMANDS, KEYBOARD_GPIOS
import globals

def keyboard_mode():
    print("Keyboard-Control mode activated.")

    def on_press(key):
        return

    def on_release(key):
        # global control_signal
        # global mode
        # global gpio

        ######### KEYBOARD_MODES["Esc"]="quit" #########
        if key == Key.esc and "Esc" in KEYBOARD_MODES.keys():
            # Stop listener
            globals.mode = KEYBOARD_MODES["Esc"] 
            assert globals.mode == "quit"
            return False
        #############################################

        if isinstance(key, KeyCode):
            if key.char in KEYBOARD_MODES.keys():
                globals.mode = KEYBOARD_MODES[key.char]
                if globals.mode == "quit":
                    return False
            elif key.char in KEYBOARD_COMMANDS.keys() and globals.mode == "keyboard":
                globals.control_signal = KEYBOARD_COMMANDS[key.char] 
            elif key.char in KEYBOARD_GPIOS.keys():
                globals.gpio = KEYBOARD_GPIOS[key.char]
                
    listener = Listener(
            on_press=on_press,
            on_release=on_release)
    listener.start()
    if globals.mode == "quit" :
        listener.join()