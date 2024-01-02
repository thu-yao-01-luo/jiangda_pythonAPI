from flight_controller import FlightController

# Use the model
mode = "follow" # "hand" or "keyboard" or "quit"
control_signal = "control" # "forward" "backward" "left" "right" Optional: "up" "down"
gpio = False
dof_follow = 6

debug = False # if debug, no control, print control signal instead
# debug = True

controller = FlightController()