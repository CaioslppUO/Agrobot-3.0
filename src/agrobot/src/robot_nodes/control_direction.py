#!/usr/bin/env python3

"""
@package control_direction
Controla a direção do robô, suavizando os movimentos de virar.
"""

import rospy
import RPi.GPIO as GPIO
from agrobot.msg import complete_command
from robot_utils import testing
from std_msgs.msg import String
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
motor_1: int = 16 # Raspberry pin 16
motor_2: int = 19 # Raspberrry pin 19
encoder: int = 0
straight_angle: tuple = (-5,-4,-3,-2,-1,1,2,3,4,5) # Posição em que o robô não está virando.
max_turn_angle: dict = {'left':-300,'right':300}

GPIO.setup(motor_1, GPIO.OUT)
GPIO.setup(motor_2, GPIO.OUT)

def turn_right() -> None:
    GPIO.output(motor_1, GPIO.LOW)
    GPIO.output(motor_2, GPIO.HIGH)
    time.sleep(0.05)
    GPIO.output(motor_1, GPIO.LOW)
    GPIO.output(motor_2, GPIO.LOW)

def turn_left() -> None:
    GPIO.output(motor_1, GPIO.HIGH)
    GPIO.output(motor_2, GPIO.LOW)
    time.sleep(0.05)
    GPIO.output(motor_1, GPIO.LOW)
    GPIO.output(motor_2, GPIO.LOW)

def straight() -> None:
    GPIO.output(motor_1, GPIO.LOW)
    GPIO.output(motor_2, GPIO.LOW)

def control_direction(steer: int, encoder: int) -> None:
    """Chama a função que vira o robô de acordo com o fator."""
    # Robô indo reto.
    if(steer in straight_angle):
        if(encoder < int(straight_angle[0])): # Corrigindo a direção.
            turn_right()
        elif(encoder > int(straight_angle[-1])): # Corrigindo a direção.
            turn_left()
        else:
            straight()
    elif(steer < int(straight_angle[0])): # Virando para a esquerda.
        if(encoder > max_turn_angle['left']):
            turn_left()
    else: # Virando para a direita.
        if(encoder < max_turn_angle['right']):
            turn_right()

def control_robot_callback(data: complete_command) -> str:
    """Envia o comando de controle recebido para os motores que controlam a 
        direção do robô."""
    steer: int = - int(data.move.angular.z)
    factor: int = 10
    control_direction(factor)
    pass

def encoder_callback(data: String) -> None:
    """Armazena o valor lido do tópico encoder."""
    global encoder 
    encoder = int(data.data)

def listen_encoder() -> None:
    """Esuta o tóico encoder e processa o valor lido."""
    rospy.Subscriver('encoder',String,encoder_callback)

def listen_control_robot() -> None:
    """Escuta o tópico control_robot e processa os comandos recebidos."""
    rospy.Subscriber('control_robot',complete_command,control_robot_callback)

if __name__ == "__main__":
    try:
        listen_encoder()
        listen_control_robot()
        rospy.spin()
    except Exception as e:
        services.do_log_error(
            'Could not send command through GPIO. {0}'.format(str(e)),
            'control_direction.py')