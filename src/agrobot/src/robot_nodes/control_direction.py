#!/usr/bin/env python3

"""
@package control_direction
Controla a direção do robô, suavizando os movimentos de virar.
"""

import rospy
import RPi.GPIO as GPIO
from agrobot.msg import complete_command
from robot_utils import testing
import time

# Injeção de dependência.
if(testing.is_test_running()):
    from test_robot_utils import services_dependency as services
else:
    from robot_utils import services

GPIO.setmode(GPIO.BOARD)

# Nó control_direction.
rospy.init_node('control_direction', anonymous=True)

# Variáveis de configuração.
motor_1: int = 16
motor_2: int = 19

GPIO.setup(motor_1, GPIO.OUT)
GPIO.setup(motor_2, GPIO.OUT)

def turn_right() -> None:
    GPIO.output(motor_1, GPIO.LOW)
    GPIO.output(motor_2, GPIO.HIGH)
    time.sleep(0.1)
    GPIO.output(motor_1, GPIO.LOW)
    GPIO.output(motor_2, GPIO.LOW)

def turn_left() -> None:
    GPIO.output(motor_1, GPIO.HIGH)
    GPIO.output(motor_2, GPIO.LOW)
    time.sleep(0.1)
    GPIO.output(motor_1, GPIO.LOW)
    GPIO.output(motor_2, GPIO.LOW)

def turn_all_right() -> None:
    GPIO.output(motor_1, GPIO.LOW)
    GPIO.output(motor_2, GPIO.HIGH)
    time.sleep(0.2)
    GPIO.output(motor_1, GPIO.LOW)
    GPIO.output(motor_2, GPIO.LOW)
    pass

def turn_all_left() -> None:
    GPIO.output(motor_1, GPIO.HIGH)
    GPIO.output(motor_2, GPIO.LOW)
    time.sleep(0.2)
    GPIO.output(motor_1, GPIO.LOW)
    GPIO.output(motor_2, GPIO.LOW)

def turn_ahead() -> None:
    pass

def control_direction(factor: int) -> None:
    """Chama a função que vira o robô de acordo com o fator."""
    if(factor == -1):
        turn_left()
    elif(factor == 1):
        turn_right()
    elif(factor == -2):
        turn_all_left()
    elif(factor == 2):
        turn_all_right()
    else:
        turn_ahead()

def get_steer_factor(steer: int) -> int:
    """Retorna o fator de viragem do robô. -1 caso esteja entre -50 e 0. 1 caso
        entre 0 e 50. -2 caso entre -100 e -50. 2 caso entre 50 e 100."""
    if(steer > -50 and steer < -10):
        return -1
    elif(steer > 10 and steer < 50):
        return 1
    elif(steer > -100 and steer < -50):
        return -2
    elif(steer > 50 and steer < 100):
        return 2
    return 0

def control_robot_callback(data: complete_command) -> str:
    """Envia o comando de controle recebido para os motores que controlam a 
        direção do robô."""
    steer: int = - int(data.move.angular.z)
    factor: int = get_steer_factor(steer)
    control_direction(factor)
    pass

def listen_control_robot() -> None:
    """Escuta o tópico control_robot e processa os comandos recebidos."""
    rospy.Subscriber('control_robot',complete_command,control_robot_callback)

if __name__ == "__main__":
    try:
        listen_control_robot()
        rospy.spin()
    except Exception as e:
        services.do_log_error(
            'Could not send command through GPIO. {0}'.format(str(e)),
            'control_direction.py')