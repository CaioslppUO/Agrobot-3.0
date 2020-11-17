#!/usr/bin/env python3

import rospy,os,pathlib,json,requests
from robot_utils import services
from agrobot.msg import complete_command
from shutil import which

# Variáveis de diretórios.
current_directory: str = str(pathlib.Path(__file__).parent.absolute()) + "/"

## Nó get_robot_commands.
rospy.init_node('get_robot_commands', anonymous=True)

def convert_bool_to_int(data: bool) -> int:
    if(data):
        return 1
    return 0
## Separa e configura o tipo de cada comando recebido.
def setup_command(command) -> complete_command:
    cpt_command = complete_command()
    try:
        cpt_command.move.linear.x = int(command["speed"])
        cpt_command.move.linear.y = int(command["steer"])
        cpt_command.limit.speed_limit = int(command["limit"])
        cpt_command.relay.signal_relay_power = convert_bool_to_int(bool(command["power"]))
        cpt_command.relay.signal_relay_module = convert_bool_to_int(bool(command["module"]))

    except Exception as e:
        services.do_log_error("Could not load json. " + str(e), "get_robot_commands.py")
    services.check_complete_control_command(cpt_command)
    return cpt_command

## Publica a mensagem no tópico /get_robot_commands.
def publish_msg(msg: complete_command) -> None:
    try:
        pub = rospy.Publisher("/get_robot_commands", complete_command, queue_size=10)
        pub.publish(msg)
    except Exception as e:
        services.do_log_error("Could not publish msg to /get_robot_commands. " + str(e),"get_robot_commands.py")

## Retorna o ipv4 do computador.
def get_ipv4() -> str:
    ip: str = ""
    if(which("ifconfig") is not None):
        try:
            os.system("ifconfig | grep 'inet *.*.*.*' > " + current_directory + "ipv4.tmp")
            with open(current_directory+"ipv4.tmp","r") as file:
                line = file.readlines()
                file.close()
                line = line[0]
                line = line.split("inet ")[1].split(" ")
                ip = line[0]
            if(os.path.exists(current_directory+"ipv4.tmp")):
                os.system("rm " + current_directory+"ipv4.tmp")
        except Exception as e:
            services.do_log_error("Could not get ipv4. " + str(e),"get_robot_commands.py")
    else:
        services.do_log_error("Could not find ifconfig tool. Please install the package net-tools.","get_robot_commands.py")
    return str(ip)

## Classe que gerencia o servidor http.
def Get_robot_commands(ip: str):
    try:
        data = json.loads(requests.get(ip).content.decode('utf-8'))
        publish_msg(setup_command(data))
    except Exception as e:
        services.do_log_error("Could not run get_robot_commands.py. " + str(e),"get_robot_commands.py")

## Execução das rotinas do get_robot_commands.
if __name__ == '__main__':
    try:
        ip: str = str("http://" + get_ipv4() +":3000/control")
        if(ip != ""):
            while not rospy.is_shutdown():
                Get_robot_commands(ip)
        else:
            services.do_log_error("Could not get ipv4.","get_robot_commands.py")
    except rospy.ROSInterruptException:
        services.do_log_warning("The roscore was interrupted.","get_robot_commands.py")