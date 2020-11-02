#!/usr/bin/env python3

import rospy
from agrobot.msg import complete_command,power_control,speed_limit
from robot_utils import services

## Nó command_center.
rospy.init_node("command_center", anonymous=True)

# Variáveis de controle de lag.
last_signal_sent_to_module: int = 0

## Envia o comando de movimento para o mini robô.
def send_command_to_mini_robot(command: complete_command) -> None:
    speed: int = int(command.move.linear.x)
    steer: int = int(command.move.linear.y)
    limit: int = int(command.limit.speed_limit)
    services.send_command_to_mini_robot(speed,steer,limit)

## Envia o comando de movimento para o robô.
def send_command_to_robot(command: complete_command) -> None:
    speed: int = int(command.move.linear.x)
    steer: int = int(command.move.linear.y)
    limit: int = int(command.limit.speed_limit)
    services.send_command_to_robot(speed,steer,limit,0)

## Envia o sinal para ligar/desligar o robô.
def send_signal_to_power_relay(command: power_control) -> None:
    signal: int = int(command.signal_relay_power)
    if(signal != 0):
        services.send_command_to_robot(0,0,0,signal)

## Envia o sinal para ligar/desligar o modulo do relé.
def send_signal_to_module_relay(command: power_control) -> None:
    global last_signal_sent_to_module
    signal: int = int(command.signal_relay_power)
    if(signal != last_signal_sent_to_module):
        last_signal_sent_to_module = signal
        services.send_signal_to_relay_module(signal)
    
## Trata o recebimento de um novo comando.
def priority_decider_callback(command: complete_command) -> None:
    robot_model = services.get_parameter("robot_model")
    if(robot_model != -1):
        send_signal_to_power_relay(command.relay)
        send_signal_to_module_relay(command.relay)
        if(robot_model == "mini_robot"):
            send_command_to_mini_robot(command)
        else:
            send_command_to_robot(command)
    else:
        services.do_log_error("Could not get robot_model.","command_center.py")

## Escuta o tópico priority_decider e gerencia os comandos recebidos.
def listen_priority_decider() -> None:
    rospy.Subscriber("/priority_decider",complete_command,priority_decider_callback)

## Executa as rotinas do command_center.
if __name__ == "__main__":
    try:
        listen_priority_decider()
        rospy.spin()
    except Exception as e:
        services.do_log_error("Could not run command_center.py. " + str(e),"command_center.py")