#!/usr/bin/env python3

import rospy
from agrobot.msg import complete_command,power_control
from robot_utils import services
from typing import Final

## Nó command_center.
rospy.init_node("command_center", anonymous=True)

# Variáveis de controle de lag.
last_module_signal_sent: int = 0

# Variáveis de controle de publicação.
pub_relay: Final = rospy.Publisher("/relay", power_control, queue_size=10)
pub_control_robot: Final = rospy.Publisher("/control_robot", complete_command, queue_size=10)
pub_control_mini_robot: Final = rospy.Publisher("/control_mini_robot", complete_command, queue_size=10)

## Envia o comando de movimento para o mini robô.
def send_command_to_mini_robot(command: complete_command) -> None:
    pub_control_mini_robot.publish(command)

## Envia o comando de movimento para o robô.
def send_command_to_robot(command: complete_command) -> None:
    pub_control_robot.publish(command)

## Envia o sinal para ligar/desligar o modulo do relé.
def send_signal_to_module_relay(command: power_control) -> None:
    global last_module_signal_sent
    if(command.signal_relay_module != last_module_signal_sent):
        last_module_signal_sent = command.signal_relay_module
        pub_relay.publish(command)
    
## Trata o recebimento de um novo comando.
def priority_decider_callback(command: complete_command) -> None:
    robot_model = services.get_parameter("robot_model")
    if(robot_model != -1):
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