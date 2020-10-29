#!/usr/bin/env python3

import rospy,rosparam
from geometry_msgs.msg import Twist
from typing import Final
from robot_utils import testing

if(testing.is_test_running()):
    from test_robot_utils import log_dependency as logs, services_dependency as services, nodes_dependency as nodes, params_dependency as params
else:
    from robot_utils import services,nodes,params,logs

## Definição do nó.
rospy.init_node("priority_decider",anonymous=True)

# Definição das prioridades e constantes.
APP_PRIORITY: Final = 1000
LIDAR_PRIORITY: Final = 999
GUARANTEED_COMMANDS: Final = 50

# Definição dos tópicos de publicação.
pub_priority_decider = rospy.Publisher("priority_decider", Twist, queue_size=10)

## Variável de controle de prioridade e quantidade de comandos.
current_priority: int = 0
remaining_commands: int = 0
current_command: Twist = None

## Coloca as constantes de prioridade no rosparam.
def publish_priorities_in_rosparam() -> None:
    try:
        rosparam.set_param("APP_PRIORITY",APP_PRIORITY)
        rosparam.set_param("LIDAR_PRIORITY",LIDAR_PRIORITY)
        rosparam.set_param("GUARANTEED_COMMANDS",GUARANTEED_COMMANDS)
    except Exception as e:
        logs.do_log_error("Could not publish priority variables to rosparam. " + str(e),"priority_decider.py")

## Publica o comando escolhido com base na prioridade.
def publish_selected_command(command: Twist) -> None:
    global current_command
    try:
        if(command != None):
            pub_priority_decider.publish(command)
            current_command = None
        else:
            pass
            logs.do_log_error("Could not publish command to topic priority_decider. The command is None.","priority_decider.py")
    except Exception as e:
        pass
        logs.do_log_error("Could not publish command to topic priority_decider. " + str(e),"priority_decider.py")

## Trata o recebimento de um novo comando.
def callback(command: Twist, priority: int) -> None:
    global current_priority,remaining_commands,current_command
    if(priority >= current_priority or (remaining_commands == 0 and priority < current_priority)):
        current_priority = priority
        remaining_commands = GUARANTEED_COMMANDS
        current_command = command
        publish_selected_command(current_command)
    else:
        remaining_commands = remaining_commands - 1

## Se inscreve em um tópico e chama a função de callback.
def listen(topic,priority) -> None:
    try:
        rospy.Subscriber(topic,Twist,callback,priority)
    except Exception as e:
        pass
        logs.do_log_error("Could not subscribe to topic (" + topic + "). " + str(e),"priority_decider.py")

## Adiciona os listenners e continua a escutar em loop.
def add_listeners_and_listen():
    global current_command
    topics_to_listen: dict = {'web_server':APP_PRIORITY, 'control_lidar_PRIORITY':LIDAR_PRIORITY}
    for key in topics_to_listen:
        listen(key,topics_to_listen[key])
    rospy.spin()

## Executa as rotinas de decisão ed prioridade.
if __name__ == "__main__":
    try:
        if(services.wait_for_services_availability() and nodes.wait_for_nodes_availability() and params.wait_for_param_availability([''])):
            logs.do_log_info("PRIORITY_DECIDER.PY STARTED.","priority_decider.py")
            publish_priorities_in_rosparam()
            add_listeners_and_listen()
        else:
            logs.do_log_error("Time limit reached when waiting for used services,nodes or parameters to respond.","priority_decider.py")
    except:
        pass
        logs.do_log_error("Could not run priority_decider.py","priority_decider.py")