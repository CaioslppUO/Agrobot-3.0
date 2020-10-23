#!/usr/bin/env python3

import rosservice,rosparam,pathlib,json,time

# Variáveis de diretório.
current_file: str = "setup.py"
project_dir: str = str(pathlib.Path(__file__).parent.absolute()) + "/../../"

# Parâmetros para serem armazenados.
version: str = ""
number_of_hoverboard_boards: int = -1
extra_module_gpio_pinout: int = -1
robot_model: str = ""

## Verifica a disponibilidade dos serviços utilizados nesse nó.
def verify_services_availability():
    log = rosservice.get_service_type("/log_error")
    while(log == None):
        log = rosservice.get_service_type("/log_error")

## Faz log de erro.
def do_log_error(msg: str):
    rosservice.call_service("/log_error",[msg,current_file])

## Faz log de info.
def do_log_info(msg: str):
    rosservice.call_service("/log_info",[msg,current_file])

## Faz log de warning.
def do_log_warning(msg: str):
    rosservice.call_service("/log_warning",[msg,current_file])

## Lê e guarda a versão atual do programa do arquivo info.json na pasta raiz do agrobot.
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

## Lê e guarda qual modelo de robô será carregado.
def get_selected_robot_model():
    global robot_model
    try:
        with open(project_dir+"src/config/setup/setup.json","r") as file:
            json_object = json.load(file)
            file.close()
            robot_model = json_object['robot_model']
            if(robot_model != ""):
                rosparam.set_param("robot_model",robot_model)
                do_log_info("Robot model read successfully from setup.json")
            else:
                rosparam.set_param("robot_model","No model selected")
                do_log_error("Could not read setup.json properly.")
    except:
        do_log_error("Could not read setup.json")

## Lê e carrega o modelo de robô selecionado.
def get_robot_model():
    global number_of_hoverboard_boards,extra_module_gpio_pinout
    get_selected_robot_model()
    try:
        with open(project_dir+"src/config/robot_models/"+str(rosparam.get_param("robot_model"))+".json","r") as file:
            json_object = json.load(file)
            file.close()
            number_of_hoverboard_boards = json_object['number_of_hoverboard_boards']
            extra_module_gpio_pinout = json_object['extra_module_gpio_pinout']
            if(number_of_hoverboard_boards != -1 and extra_module_gpio_pinout != -1):
                rosparam.set_param("hoverboard_boards",str(number_of_hoverboard_boards))
                rosparam.set_param("module_pinout",str(extra_module_gpio_pinout))
                do_log_info("Robot model loaded successfully from " + str(rosparam.get_param("robot_model")) + ".json")
            else:
                rosparam.set_param("hoverboard_boards","0")
                rosparam.set_param("module_pinout","-1")
                do_log_error("Could not load robot model (" + str(rosparam.get_param("robot_model")) + ") properly. Check for invalid values.")
    except Exception as e:
        do_log_error("Could not load robot_model (" + str(rosparam.get_param("robot_model")) + "). " + str(e))

## Executa as rotinas de setup.
if __name__ == "__main__":
    verify_services_availability()
    get_version()
    get_robot_model()