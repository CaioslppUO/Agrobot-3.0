#!/usr/bin/env python3

import rospy,os,pathlib
from agrobot.srv import log_error,log_info,log_warning
from datetime import datetime

# Variáveis de caminho.
log_dir: str = str(pathlib.Path(__file__).parent.absolute()) + "/../../log/"

# Nó de logs.
rospy.init_node("log")

## Cria a pasta de logs caso não exista.
def create_log_folder():
    try:
        if(not os.path.exists(log_dir)):
            os.mkdir(log_dir)
    except:
        pass

## Escreve no arquivo de logs.
def write_log(log_type,msg):
    try:
        with open(log_dir+"log.txt","a") as file:
            current_time = datetime.now().strftime("%H:%M:%S")
            file.write("(" + current_time + ") " + log_type + " " + msg+"\n")
            file.close()
        return "Succesffuly logged the message."
    except:
        return "Error Trying to log the message."

## Trata o recebimento de erro.
def handle_log_error(data):
    return write_log("[ERROR]",data.log_msg)

## Trata o recebimento de info.
def handle_log_info(data):
    return write_log("[INFO]",data.log_msg)

## Trata o recebimento de warning.
def handle_log_warning(data):
    return write_log("[WARNING]",data.log_msg)

## Escuta o chamado dos serviços.
def log_server():
    rospy.Service("log_error", log_error, handle_log_error)
    rospy.Service("log_info", log_info, handle_log_info)
    rospy.Service("log_warning", log_warning, handle_log_warning)

## Executa as rotinas de log.
if __name__ == "__main__":
    log_server()
    rospy.spin()