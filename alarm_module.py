from globals import gpio, controller
from flight_controller import FlightController
from time import time

def reminder():
    """
    zzh: 具体电平由接线决定 fgbr 
    """
    global gpio
    global controller
    while True:
        if gpio:
            controller.SetGPIO(FlightController.GPIO_S, FlightController.HIGH)
            controller.SetGPIO(FlightController.GPIO_K, FlightController.HIGH)
            controller.SetGPIO(FlightController.GPIO_I, FlightController.HIGH)
            controller.SetGPIO(FlightController.GPIO_O, FlightController.LOW)
            time.sleep(1)
            controller.SetGPIO(FlightController.GPIO_S, FlightController.LOW)
            controller.SetGPIO(FlightController.GPIO_K, FlightController.HIGH)
            controller.SetGPIO(FlightController.GPIO_I, FlightController.HIGH)
            controller.SetGPIO(FlightController.GPIO_O, FlightController.HIGH)
        else:
            controller.SetGPIO(FlightController.GPIO_S, FlightController.LOW)
            controller.SetGPIO(FlightController.GPIO_K, FlightController.LOW)
            controller.SetGPIO(FlightController.GPIO_I, FlightController.HIGH)
            controller.SetGPIO(FlightController.GPIO_O, FlightController.HIGH)
