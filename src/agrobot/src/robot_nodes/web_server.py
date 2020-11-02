#!/usr/bin/env python3

import rospy,os,pathlib
from http.server import BaseHTTPRequestHandler, HTTPServer
from threading import Thread
from robot_utils import services
from agrobot.msg import complete_command,power_control,speed_limit
from typing import Final

# Variáveis de diretórios.
current_directory: Final = str(pathlib.Path(__file__).parent.absolute()) + "/"

## Nó web_server.
rospy.init_node('web_server', anonymous=True)

## Separa e configura o tipo de cada comando recebido.
def setup_command(command: str) -> complete_command:
    commands: list = command.split("*")
    spd_limit = speed_limit()
    pwr_control = power_control()
    cpt_command = complete_command()
    for cm in commands:
        aux: list = cm.split("$")
        try:
            if(aux[0] == "speed"):
                cpt_command.move.linear.x = int(aux[1])
            elif(aux[0] == "steer"):
                cpt_command.move.linear.y = int(aux[1])
            elif(aux[0] == "limit"):
                spd_limit.speed_limit = int(aux[1])
            elif(aux[0] == "powerA"):
                pwr_control.signal_relay_power = int(aux[1])
            elif(aux[0] == "pulverize"):
                pwr_control.signal_relay_module = int(aux[1])
        except:
            pass
    cpt_command.limit = spd_limit
    cpt_command.relay = pwr_control
    services.check_complete_control_command(cpt_command)
    return cpt_command

## Publica a mensagem no tópico /web_server.
def publish_msg(msg: complete_command) -> None:
    try:
        pub = rospy.Publisher("/web_server", complete_command, queue_size=10)
        pub.publish(msg)
    except Exception as e:
        services.do_log_error("Could not publish msg to /web_server. " + str(e),"web_server.py")

## Classe que gerencia os requests feitos no servidor http.
class RequestHandler_httpd(BaseHTTPRequestHandler):
    ## Trata o GET feito pelo app de controle manual.
    def do_GET(self):
        web_server_request = None
        msg = None
        web_server_request = self.requestline
        web_server_request = web_server_request[5 : int(len(web_server_request)-9)]
        msg = str(web_server_request) # Mensagem recebida do app.
        publish_msg(setup_command(msg))
        return

## Retorna o ipv4 do computador.
def get_ipv4() -> str:
    ip: str = ""
    try:
        os.system("ifconfig | grep '192.' > " + current_directory + "ipv4.tmp")
        with open(current_directory+"ipv4.tmp","r") as file:
            line = file.readlines()
            file.close()
            line = line[0]
            line = line.rstrip('\n')
            line = line.split(" ")
            ip = line[1]
        if(os.path.exists(current_directory+"ipv4.tmp")):
            os.system("rm " + current_directory+"ipv4.tmp")
    except Exception as e:
        services.do_log_error("Could not get ipv4. " + str(e),"web_server.py")
    return ip

## Classe que gerencia o servidor http.
class Web_server():
    ## Inicializa as variáveis e o servidor.
    def __init__(self):
        try:
            self.server_address_httpd = (get_ipv4(),8080)
            httpd = HTTPServer(self.server_address_httpd, RequestHandler_httpd)
            self.server_thread = Thread(target=httpd.serve_forever)
            self.server_thread.daemon = True # O servidor é fechado ao finalizar o programa.
            self.server_thread.start()
        except Exception as e:
            services.do_log_error("Could not run web_server.py. " + str(e),"web_server.py")

## Execução das rotinas do web_server.
if __name__ == '__main__':
    try:
        web_server = Web_server()
        while not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException:
        services.do_log_warning("The roscore was interrupted.","web_server.py")
        pass