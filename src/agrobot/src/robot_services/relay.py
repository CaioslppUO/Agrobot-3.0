#!/usr/bin/env python3

import robot_utils.testing as testing,rospy,rosparam,time
from agrobot.srv import relay
from typing import Final

if(testing.is_test_running()):
    from test_robot_utils import log_dependency as logs
else:
    from robot_utils import logs

# Variáveis de controle de importação.
gpio_imported: bool = False

try:
    import RPi.GPIO as GPIO
    GPIO.setmode(GPIO.BOARD)
    GPIO.setwarnings(False)
    gpio_imported = True
except Exception as e:
    logs.do_log_warning("Could not import RPi.GPIO. This will be ignored and no real signal will be send to the relay. " + str(e),"relay.py")
    pass

# Nó do relé.
rospy.init_node("relay")

## Envia o sinal para o relé.
def send_signal(data: relay) -> None:
    try:
        pinout: Final = int(rosparam.get_param("module_pinout"))
        if(gpio_imported):
            if(int(data.signal) == 1):
                GPIO.setmode(GPIO.BOARD)
                GPIO.setwarnings(False)
                GPIO.setup(pinout, GPIO.OUT)
                GPIO.output(pinout, GPIO.HIGH)
                time.sleep(0.2)
                logs.do_log_info("The signal 1 was sent to the relay module.","relay.py")
            elif(int(data.signal) == 0):
                GPIO.setmode(GPIO.BOARD)
                GPIO.setwarnings(False)
                GPIO.setup(pinout, GPIO.OUT)
                GPIO.output(pinout, GPIO.HIGH)
                time.sleep(0.2)
                logs.do_log_info("The signal 0 was sent to the relay module.","relay.py")
            else:
                GPIO.setmode(GPIO.BOARD)
                GPIO.setwarnings(False)
                GPIO.setup(pinout, GPIO.OUT)
                GPIO.output(pinout, GPIO.HIGH)
                time.sleep(0.2)
                logs.do_log_warning("The signal " + str(data.signal) + " is not valid, 0 was sent to the relay module.","relay.py")
        else:
            logs.do_log_warning("No signal was sent to the relay. GPIO module could not be imported.","relay.py")
        return "The signal was sent to the module."
    except:
        return "Could not send the signal to the module."


## Escuta o chamado dos serviços.
def log_server():
    rospy.Service("relay", relay, send_signal)

## Executa as rotinas do serviço.
if __name__ == "__main__":
    log_server()
    rospy.spin()