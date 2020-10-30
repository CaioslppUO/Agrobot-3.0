#!/usr/bin/env python3

import sys,rospy
from agrobot.srv import control_mini_robot
from typing import Final
from robot_utils import testing

if(testing.is_test_running()):
    from test_robot_utils import services_dependency as services
else:
    from robot_utils import services

## Nó control_robot.
rospy.init_node('control_mini_robot', anonymous=True)

# Variáveis de controle de importação.
gpio_imported: bool = False
robot_model_is_mini_robot: bool = False

# Variáveis de GPIO.
left_wheel_1 = int(services.get_parameter('gpio_left_wheel_1'))
left_wheel_2 = int(services.get_parameter('gpio_left_wheel_2'))
right_wheel_1 = int(services.get_parameter('gpio_right_wheel_1'))
right_wheel_2 = int(services.get_parameter('gpio_right_wheel_2'))

if(left_wheel_1 != -1 and left_wheel_2 != -1 and right_wheel_1 != -1 and right_wheel_2 != -1):
    robot_model_is_mini_robot = True
else:
    services.do_log_warning("Robot model is not mini_robot. control_mini_robot will not work properly. ","control_mini_robot.py")

left_wheel_1_pwm = None
left_wheel_2_pwm = None
right_wheel_1_pwm = None
right_wheel_2_pwm = None

# Variáveis de controle
movement: str = "forward"
direction: str = "none"
limit: int = 100

try:
    import RPi.GPIO as GPIO
    GPIO.setmode(GPIO.BOARD)
    GPIO.setwarnings(False)

    # Configurando o GPIO
    GPIO.setmode(GPIO.BOARD)
    GPIO.setwarnings(False)
    GPIO.setup(left_wheel_1, GPIO.OUT)
    GPIO.setup(left_wheel_2, GPIO.OUT)
    GPIO.setup(right_wheel_1, GPIO.OUT)
    GPIO.setup(right_wheel_2, GPIO.OUT)

    # Configurando o PWM
    left_wheel_1_pwm = GPIO.PWM(left_wheel_1, 100)
    left_wheel_2_pwm = GPIO.PWM(left_wheel_2, 100)
    right_wheel_1_pwm = GPIO.PWM(right_wheel_1, 100)
    right_wheel_2_pwm = GPIO.PWM(right_wheel_2, 100)

    left_wheel_1_pwm.start(0)
    left_wheel_2_pwm.start(0)
    right_wheel_1_pwm.start(0)
    right_wheel_2_pwm.start(0)

    gpio_imported = True
except Exception as e:
    services.do_log_warning("Could not import RPi.GPIO. This will be ignored and no real command will be send to the mini_robot. " + str(e),"control_mini_robot.py")
    pass

## Movimenta o robô para frente.
def go_forward():
    if(gpio_imported):
        left_wheel_1_pwm.ChangeDutyCycle(0)
        left_wheel_2_pwm.ChangeDutyCycle(0)
        right_wheel_1_pwm.ChangeDutyCycle(limit)
        right_wheel_2_pwm.ChangeDutyCycle(limit)

## Movimenta o robô para trás.
def go_back() -> None:
    if(gpio_imported):
        left_wheel_1_pwm.ChangeDutyCycle(limit)
        left_wheel_2_pwm.ChangeDutyCycle(limit)
        right_wheel_1_pwm.ChangeDutyCycle(0)
        right_wheel_2_pwm.ChangeDutyCycle(0)

## Movimenta o robô para frente e esquerda.
def turn_left_forward() -> None:
    if(gpio_imported):
        left_wheel_1_pwm.ChangeDutyCycle(0)
        left_wheel_2_pwm.ChangeDutyCycle(0)
        right_wheel_1_pwm.ChangeDutyCycle(limit)
        right_wheel_2_pwm.ChangeDutyCycle(0)

## Movimenta o robô para frente e direita.
def turn_right_forward() -> None:
    if(gpio_imported):
        left_wheel_1_pwm.ChangeDutyCycle(0)
        left_wheel_2_pwm.ChangeDutyCycle(0)
        right_wheel_1_pwm.ChangeDutyCycle(0)
        right_wheel_2_pwm.ChangeDutyCycle(limit)

## Movimenta o robô para trás e esquerda.
def turn_left_back() -> None:
    if(gpio_imported):
        left_wheel_1_pwm.ChangeDutyCycle(limit)
        left_wheel_2_pwm.ChangeDutyCycle(0)
        right_wheel_1_pwm.ChangeDutyCycle(0)
        right_wheel_2_pwm.ChangeDutyCycle(0)

## Movimenta o robô para trás e direita.
def turn_right_back() -> None:
    if(gpio_imported):
        left_wheel_1_pwm.ChangeDutyCycle(0)
        left_wheel_2_pwm.ChangeDutyCycle(limit)
        right_wheel_1_pwm.ChangeDutyCycle(0)
        right_wheel_2_pwm.ChangeDutyCycle(0)

## Para o robô.
def stop() -> None:
    if(gpio_imported):
        left_wheel_1_pwm.ChangeDutyCycle(0)
        left_wheel_2_pwm.ChangeDutyCycle(0)
        right_wheel_1_pwm.ChangeDutyCycle(0)
        right_wheel_2_pwm.ChangeDutyCycle(0)

## Movimenta o robô para a esquerda.
def turn_all_left() -> None:
    if(gpio_imported):
        left_wheel_1_pwm.ChangeDutyCycle(0)
        left_wheel_2_pwm.ChangeDutyCycle(limit)
        right_wheel_1_pwm.ChangeDutyCycle(limit)
        right_wheel_2_pwm.ChangeDutyCycle(0)

## Movimenta o robô para a direita.
def turn_all_right() -> None:
    if(gpio_imported):
        left_wheel_1_pwm.ChangeDutyCycle(limit)
        left_wheel_2_pwm.ChangeDutyCycle(0)
        right_wheel_1_pwm.ChangeDutyCycle(0)
        right_wheel_2_pwm.ChangeDutyCycle(limit)

## Seta os valores das variáveis recebidas.
def set_values(speed,steer,lim) -> None:
    global movement,direction,limit
    if(speed < -35):
        movement = "reverse"
    elif(speed > 35):
        movement = "forward"
    else:
        movement = "none"

    if(steer < -35):
        direction = "left"
    elif(steer > 35):
        direction = "right"
    else:
        direction = "none"

    limit = lim

## Movimenta o robô com base nos valores recebidos pelo serviço.
def move(data: control_mini_robot) -> str:
    set_values(data.speed,data.steer,data.limit)
    if(robot_model_is_mini_robot):
        if(movement == "forward" and direction == "none"):
            go_forward()
        elif(movement == "forward" and direction == "left"):
            turn_left_forward()
        elif(movement == "forward" and direction == "right"):
            turn_right_forward()
        elif(movement == "none" and direction == "left"):
            turn_all_left()
        elif(movement == "none" and direction == "right"):
            turn_all_right()
        elif(movement == "reverse" and direction == "none"):
            go_back()
        elif(movement == "reverse" and direction == "left"):
            turn_left_back()
        elif(movement == "reverse" and direction == "right"):
            turn_right_back()
        else:
            stop()
    return str(movement)+";"+str(direction)+";"+str(limit) 

## Escuta o chamado do serviço.
def control_mini_robot_server() -> None:
    rospy.Service("control_mini_robot",control_mini_robot,move)

## Executa as rotinas do serviço.
if __name__ == "__main__":
    control_mini_robot_server()
    rospy.spin()