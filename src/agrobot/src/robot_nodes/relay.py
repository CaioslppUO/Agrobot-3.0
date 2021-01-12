#!/usr/bin/env python3

"""
@package relay
Controla a comunicação com os relés que controlam módulos extras, através do GPIO.
"""

import robot_utils.testing as testing,rospy,time
from agrobot.msg import power_control

# Injeção de dependência.
if(testing.is_test_running()):
    from test_robot_utils import services_dependency as services
else:
    from robot_utils import services

# Variáveis de controle de importação.
gpio_imported: bool = False

try:
    import RPi.GPIO as GPIO
    GPIO.setmode(GPIO.BOARD)
    GPIO.setwarnings(False)
    gpio_imported = True
except Exception as e:
    services.do_log_warning("Could not import RPi.GPIO. This will be ignored and no real signal will be send to the relay. " + str(e),"relay.py")
    pass

# Nó do relé.
rospy.init_node("relay")

## Envia o sinal para o relé que liga ou desliga o módulo extra..
def power_control_callback(data: power_control) -> str:
    pinout: int = int(services.get_parameter("module_pinout"))        
    try:
        if(gpio_imported):
            if(pinout != -1):
                if(int(data.signal_relay_power) == 1):
                    GPIO.setmode(GPIO.BOARD)
                    GPIO.setwarnings(False)
                    GPIO.setup(pinout, GPIO.OUT)
                    GPIO.output(pinout, GPIO.HIGH)
                    time.sleep(0.2)
                    services.do_log_info("The signal 1 was sent to the relay module.","relay.py")
                elif(int(data.signal_relay_power) == 0):
                    GPIO.setmode(GPIO.BOARD)
                    GPIO.setwarnings(False)
                    GPIO.setup(pinout, GPIO.OUT)
                    GPIO.output(pinout, GPIO.HIGH)
                    time.sleep(0.2)
                    services.do_log_info("The signal 0 was sent to the relay module.","relay.py")
                else:
                    GPIO.setmode(GPIO.BOARD)
                    GPIO.setwarnings(False)
                    GPIO.setup(pinout, GPIO.OUT)
                    GPIO.output(pinout, GPIO.HIGH)
                    time.sleep(0.2)
                    services.do_log_warning("The signal " + str(data.signal_relay_power) + " is not valid, 0 was sent to the relay module.","relay.py")
            else:
                services.do_log_warning("The rosparam module_pinout parameter wasn't set. The relay service will not work properly. " + str(e),"relay.py")
        else:
            services.do_log_warning("No signal was sent to the relay. GPIO module could not be imported.","relay.py")
        return "The signal was sent to the module."
    except:
        return "Could not send the signal to the module."


## Escuta comandos recebidos que devem ser enviados para o relé.
def listen_relay() -> None:
    rospy.Subscriber("relay", power_control, power_control_callback)

if __name__ == "__main__":
    listen_relay()
    rospy.spin()