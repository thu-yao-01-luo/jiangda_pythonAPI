from globals import debug
if not debug:
    from flight_controller import FlightController
# from globals import debug, FlightController
import globals
from time import time

def reminder():
    """
    zzh: 具体电平由接线决定 fgbr 
    """
    # global gpio
    if debug:
        while True:
            if globals.gpio:             
                print("warning!")            
                time.sleep(2)
    else:
        # global controller

        while True:
            if globals.gpio:
                globals.controller.SetGPIO(FlightController.GPIO_S, FlightController.HIGH)
                globals.controller.SetGPIO(FlightController.GPIO_K, FlightController.HIGH)
                globals.controller.SetGPIO(FlightController.GPIO_I, FlightController.HIGH)
                globals.controller.SetGPIO(FlightController.GPIO_O, FlightController.LOW)
                time.sleep(1)
                globals.controller.SetGPIO(FlightController.GPIO_S, FlightController.LOW)
                globals.controller.SetGPIO(FlightController.GPIO_K, FlightController.HIGH)
                globals.controller.SetGPIO(FlightController.GPIO_I, FlightController.HIGH)
                globals.controller.SetGPIO(FlightController.GPIO_O, FlightController.HIGH)
            else:
                globals.controller.SetGPIO(FlightController.GPIO_S, FlightController.LOW)
                globals.controller.SetGPIO(FlightController.GPIO_K, FlightController.LOW)
                globals.controller.SetGPIO(FlightController.GPIO_I, FlightController.HIGH)
                globals.controller.SetGPIO(FlightController.GPIO_O, FlightController.HIGH)