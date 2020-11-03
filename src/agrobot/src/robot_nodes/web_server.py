#!/usr/bin/env python3

import rospy,os,pathlib,json,requests
from robot_utils import services
from agrobot.msg import complete_command,power_control,speed_limit

# Variáveis de diretórios.
current_directory: str = str(pathlib.Path(__file__).parent.absolute()) + "/"

## Nó web_server.
rospy.init_node('web_server', anonymous=True)

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
        services.do_log_error("Could not load json. " + str(e), "web_server.py")
    services.check_complete_control_command(cpt_command)
    return cpt_command

## Publica a mensagem no tópico /web_server.
def publish_msg(msg: complete_command) -> None:
    try:
        pub = rospy.Publisher("/web_server", complete_command, queue_size=10)
        pub.publish(msg)
    except Exception as e:
        services.do_log_error("Could not publish msg to /web_server. " + str(e),"web_server.py")

## Retorna o ipv4 do computador.
def get_ipv4() -> str:
    ip: str = ""
    try:
        os.system("ifconfig | grep '192.' > " + current_directory + "ipv4.tmp")
        with open(current_directory+"ipv4.tmp","r") as file:
            line = file.readlines()
            file.close()
            line = line[0]
            line = line.split("inet ")[1].split(" ")
            ip = line[0]
        if(os.path.exists(current_directory+"ipv4.tmp")):
            os.system("rm " + current_directory+"ipv4.tmp")
    except Exception as e:
        services.do_log_error("Could not get ipv4. " + str(e),"web_server.py")
    return str(ip)

## Classe que gerencia o servidor http.
def Web_server(ip: str):
    try:
        data = json.loads(requests.get(ip).content.decode('utf-8'))
        publish_msg(setup_command(data))
    except Exception as e:
        services.do_log_error("Could not run web_server.py. " + str(e),"web_server.py")

## Execução das rotinas do web_server.
if __name__ == '__main__':
    try:
        ip: str = str("http://" + get_ipv4() +":3000/control")
        while not rospy.is_shutdown():
            Web_server(ip)
    except rospy.ROSInterruptException:
        services.do_log_warning("The roscore was interrupted.","web_server.py")