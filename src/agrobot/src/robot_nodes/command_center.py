#!/usr/bin/env python3

import rospy
from agrobot.msg import complete_command,power_control
from robot_utils import services

## Nó command_center.
rospy.init_node("command_center", anonymous=True)

# Variáveis de controle de lag.
last_module_signal_sent: int = 0

# Variáveis de controle de publicação.
pub_relay: rospy.Publisher = rospy.Publisher("/relay", power_control, queue_size=10)
pub_control_robot: rospy.Publisher = rospy.Publisher("/control_robot", complete_command, queue_size=10)

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
    send_signal_to_module_relay(command.relay)
    send_command_to_robot(command)

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