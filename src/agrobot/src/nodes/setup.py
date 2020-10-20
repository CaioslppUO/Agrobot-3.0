#!/usr/bin/env python3

import rospy,rosservice,rosparam,pathlib,json

# Variáveis de diretório.
version: str = ""
current_file: str = "setup.py"
project_dir: str = str(pathlib.Path(__file__).parent.absolute()) + "/../../"

## Faz log de erro.
def do_log_error(msg: str):
    rosservice.call_service("/log_error",[msg,current_file])

## Faz log de info.
def do_log_info(msg: str):
    rosservice.call_service("/log_info",[msg,current_file])

## Faz log de warning.
def do_log_warning(msg: str):
    rosservice.call_service("/log_warning",[msg,current_file])

## Lê a versão atual do programa do arquivo info.json na pasta raiz do agrobot.
def get_version():
    global version
    json_object = None
    try:
        with open(project_dir+"info.json","r") as file:
            json_object = json.load(file) 
            file.close()
            version = json_object['version']
            if(version != ""):
                rosparam.set_param("version",version)
                do_log_info("Version read successfully from info.json.")
            else:
                rosparam.set_param("version","-1")
                do_log_error("Could not read info.json properly.")
    except:
        do_log_error("Could not read info.json.")

## Executa as rotinas de setup.
if __name__ == "__main__":
    get_version()
