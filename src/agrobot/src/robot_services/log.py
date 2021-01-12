#!/usr/bin/env python3

## Faz o log de toda a aplicação.

import rospy,os,pathlib
from agrobot.srv import log_error,log_info,log_warning
from datetime import datetime

# Variáveis de diretório.
log_dir: str = str(pathlib.Path(__file__).parent.absolute()) + "/../../log/"

# Nó de logs.
rospy.init_node("log")

## Cria a pasta de logs caso não exista.
def create_log_folder() -> None:
    try:
        if(not os.path.exists(log_dir)):
            os.mkdir(log_dir)
    except Exception as e:
        print("Error trying to create log_dir. " + str(e))

## Escreve no log que uma nova execução começou.
def write_new_execution_in_log() -> str:
    try:
        with open(log_dir+"log.txt","a") as file:
            current_time = datetime.now().strftime("%H:%M:%S")
            file.write("\n\n===================================== NEW EXECUTION (" + str(current_time) + ") =====================================\n")
            file.close()
        return "Successfully logged the message."
    except Exception as e:
        return "Error Trying to log the message. " + str(e)

## Escreve no arquivo de logs.
def write_log(log_type,msg) -> None:
    try:
        with open(log_dir+"log.txt","a") as file:
            current_time = datetime.now().strftime("%H:%M:%S")
            file.write("(" + current_time + ") <" + msg.file + "> " + log_type + " " + msg.log_msg+"\n")
            file.close()
        return "Successfully logged the message."
    except Exception as e:
        return "Error Trying to log the message. " + str(e)

## Trata o recebimento de error.
def handle_log_error(data) -> None:
    return write_log("[ERROR]",data)

## Trata o recebimento de info.
def handle_log_info(data) -> None:
    return write_log("[INFO]",data)

## Trata o recebimento de warning.
def handle_log_warning(data) -> None:
    return write_log("[WARNING]",data)

## Escuta o chamado dos serviços.
def log_server() -> None:
    rospy.Service("log_error", log_error, handle_log_error)
    rospy.Service("log_info", log_info, handle_log_info)
    rospy.Service("log_warning", log_warning, handle_log_warning)

if __name__ == "__main__":
    create_log_folder()
    write_new_execution_in_log()
    log_server()
    rospy.spin()