#!/usr/bin/env python3

"""
@package control_robot
Controla a comunicação com o Vesc através dos pinos pwm para controlar o robô.
"""

import rospy
from agrobot.msg import complete_command
from robot_utils import testing

# Injeção de dependência.
if(testing.is_test_running()):
    from test_robot_utils import services_dependency as services
else:
    from robot_utils import services

# Nó do controle do robô.
rospy.init_node("control_robot", anonymous=True)

pin_gpio_pwm = int(services.get_parameter('gpio_pwm'))
frequency_pwm = int(services.get_parameter('frequency'))

try:
	import RPi.GPIO as GPIO
	# #Configuração dos GPIO
	GPIO.setmode(GPIO.BOARD)
	GPIO.setup(pin_gpio_pwm, GPIO.OUT)

	# #Configure the pwm objects and initialize its value
	pwm = GPIO.PWM(pin_gpio_pwm,frequency_pwm)#pino/frequência
	pwm.start(50)
except Exception as e:
    services.do_log_warning("Could not import RPi.GPIO. This will be ignored and no real command will be send to the control_robot_pwm. " + str(e),"control_mini_robot.py")
    pass

## Envia a string de comando pela porta UART.
def send_command(command: int) -> None:
    try:
        pwm.ChangeDutyCycle(command)
    except Exception as e:
        services.do_log_error("Could not send command through PWM. " + str(e),"control_robot.py")

def convert_command_pwm(speed: int) -> int:
    if (speed >= 0):
        return int((speed / 100 * 48) + 50)
    return int(50-((-speed/100 *47)+1))

## Envia o comando de controle recebido para o robô.
def control_robot_callback(data: complete_command) -> int:
    speed: int = int(data.move.linear.x)
    steer: int = - int(data.move.angular.z) # O negativo desconverte do padrão.
    limit: int = int(data.limit.speed_limit/100)
    power_signal: int = int(data.relay.signal_relay_power)
    command: int = convert_command_pwm(speed*abs(limit))
    send_command(command)
    return command

## Escuta o topico control_robot e processa os comandos recebidos.
def listen_control_robot() -> None:
    rospy.Subscriber("control_robot",complete_command, control_robot_callback)

if __name__ == "__main__":
    listen_control_robot()
    rospy.spin()