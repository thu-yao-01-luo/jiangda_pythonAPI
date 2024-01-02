dof_follow = 6

# debug = False # if debug, no control, print control signal instead
debug = True

# Use the model
mode = "follow" # "hand" or "keyboard" or "quit"
control_signal = "control" # "forward" "backward" "left" "right" Optional: "up" "down"
gpio = False

if not debug:
    from flight_controller import FlightController
    controller = FlightController()

CONTROL_SIGNALS = [
    "forward",
    "backward",
    "left",
    "right",
    "up",
    "down", 
    "quit",
    "shift",
]

KEYBOARD_COMMANDS = {
    "w": "forward", 
    "s": "backward",
    "a": "left",
    "d": "right",
    "o": "up", 
    "p": "down",
}

KEYBOARD_GPIOS={
    "j": True,
    "l": False,
}

KEYBOARD_MODES = {
    "f": "follow",
    "h": "hand",
    "k": "keyboard",
    "q": "quit",
}

assert len(KEYBOARD_COMMANDS.keys() & KEYBOARD_MODES.keys()) == 0, "intersection between modes and commands!" 
assert len(KEYBOARD_COMMANDS.keys() & KEYBOARD_GPIOS.keys()) == 0, "intersection between alarms and commands!" 
assert len(KEYBOARD_MODES.keys() & KEYBOARD_GPIOS.keys()) == 0, "intersection between alarms and modes!" 