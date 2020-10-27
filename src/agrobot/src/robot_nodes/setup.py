#!/usr/bin/env python3

import rosparam,pathlib,json

if(rosparam.get_param("testing") != "True"):
    from robot_utils import logs,services
else:
    from test_robot_utils import logs_dependency,services_dependency

# Variáveis de diretório.
current_file: str = "setup.py"
robot_config_dir: str = "robot_config"
project_dir: str = str(pathlib.Path(__file__).parent.absolute()) + "/../../"

# Parâmetros para serem armazenados.
version: str = ""
number_of_hoverboard_boards: int = -1
extra_module_gpio_pinout: int = -1
robot_model: str = ""

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
                logs.do_log_info("Version read successfully from info.json.",current_file)
            else:
                rosparam.set_param("version","-1")
                logs.do_log_error("Could not read info.json properly.",current_file)
    except Exception as e:
        logs.do_log_error("Could not read info.json."+str(e),current_file)

## Lê e guarda qual modelo de robô será carregado.
def get_selected_robot_model():
    global robot_model
    try:
        with open(project_dir+"src/" + robot_config_dir + "/setup/setup.json","r") as file:
            json_object = json.load(file)
            file.close()
            robot_model = json_object['robot_model']
            if(robot_model != ""):
                rosparam.set_param("robot_model",robot_model)
                logs.do_log_info("Robot model read successfully from setup.json",current_file)
            else:
                rosparam.set_param("robot_model","No model selected")
                logs.do_log_error("Could not read setup.json properly.",current_file)
    except Exception as e:
        logs.do_log_error("Could not read setup.json"+str(e),current_file)

## Lê e carrega o modelo de robô selecionado.
def get_robot_model():
    global number_of_hoverboard_boards,extra_module_gpio_pinout
    get_selected_robot_model()
    try:
        with open(project_dir+"src/" + robot_config_dir + "/robot_models/"+str(rosparam.get_param("robot_model"))+".json","r") as file:
            json_object = json.load(file)
            file.close()
            number_of_hoverboard_boards = json_object['number_of_hoverboard_boards']
            extra_module_gpio_pinout = json_object['extra_module_gpio_pinout']
            if(number_of_hoverboard_boards != -1 and extra_module_gpio_pinout != -1):
                rosparam.set_param("hoverboard_boards",str(number_of_hoverboard_boards))
                rosparam.set_param("module_pinout",str(extra_module_gpio_pinout))
                logs.do_log_info("Robot model loaded successfully from " + str(rosparam.get_param("robot_model")) + ".json",current_file)
            else:
                rosparam.set_param("hoverboard_boards","0")
                rosparam.set_param("module_pinout","-1")
                logs.do_log_error("Could not load robot model (" + str(rosparam.get_param("robot_model")) + ") properly. Check for invalid values.",current_file)
    except Exception as e:
        logs.do_log_error("Could not load robot_model (" + str(rosparam.get_param("robot_model")) + "). " + str(e),current_file)

## Executa as rotinas de setup.
if __name__ == "__main__":
    if(services.wait_for_services_availability()):
        get_version()
        get_robot_model()
    else:
        logs.do_log_error("Time limit reached when waiting for used services to respond.","setup.py")