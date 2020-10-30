#!/usr/bin/env python3

import rosparam,pathlib,json,robot_utils.testing as testing
from typing import Final

if(testing.is_test_running()):
    from test_robot_utils import services_dependency as services
else:
    from robot_utils import services

# Variáveis de diretório.
current_file: Final = "setup.py"
robot_config_dir: Final = "robot_config"
project_dir: Final = str(pathlib.Path(__file__).parent.absolute()) + "/../../"

# Parâmetros para serem armazenados.
version: str = ""
number_of_hoverboard_boards: int = -1
extra_module_gpio_pinout: int = -1
robot_model: str = ""
gpio_left_wheel_1: int = -1
gpio_left_wheel_2: int = -1
gpio_right_wheel_1: int = -1
gpio_right_wheel_2: int = -1

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
                services.do_log_info("Version read successfully from info.json.",current_file)
            else:
                rosparam.set_param("version","-1")
                services.do_log_error("Could not read info.json properly.",current_file)
    except Exception as e:
        services.do_log_error("Could not read info.json. "+str(e),current_file)

## Lê e guarda qual modelo de robô será carregado.
def get_selected_robot_model() -> str:
    global robot_model
    try:
        with open(project_dir+"src/" + robot_config_dir + "/setup/setup.json","r") as file:
            json_object = json.load(file)
            file.close()
            robot_model = json_object['robot_model']
            if(robot_model != ""):
                rosparam.set_param("robot_model",robot_model)
                services.do_log_info("Robot model read successfully from setup.json",current_file)
            else:
                rosparam.set_param("robot_model","No model selected")
                services.do_log_error("Could not read setup.json properly.",current_file)
    except Exception as e:
        services.do_log_error("Could not read setup.json. "+str(e),current_file)
    return robot_model

## Lê e carrega o modelo de robô selecionado.
def get_robot_model():
    robot_model = get_selected_robot_model()
    if(robot_model == "corona_killer" or robot_model == "agrobot"):
        global number_of_hoverboard_boards,extra_module_gpio_pinout
        try:
            with open(project_dir+"src/" + robot_config_dir + "/robot_models/"+str(rosparam.get_param("robot_model"))+".json","r") as file:
                json_object = json.load(file)
                file.close()
                number_of_hoverboard_boards = json_object['number_of_hoverboard_boards']
                extra_module_gpio_pinout = json_object['extra_module_gpio_pinout']
                if(number_of_hoverboard_boards != -1 and extra_module_gpio_pinout != -1):
                    rosparam.set_param("hoverboard_boards",str(number_of_hoverboard_boards))
                    rosparam.set_param("module_pinout",str(extra_module_gpio_pinout))
                    services.do_log_info("Robot model loaded successfully from " + str(rosparam.get_param("robot_model")) + ".json",current_file)
                else:
                    rosparam.set_param("hoverboard_boards","0")
                    rosparam.set_param("module_pinout","-1")
                    services.do_log_error("Could not load robot model (" + str(rosparam.get_param("robot_model")) + ") properly. Check for invalid values.",current_file)
        except Exception as e:
            services.do_log_error("Could not load robot_model (" + str(rosparam.get_param("robot_model")) + "). " + str(e),current_file)
    elif(robot_model == "mini_robot"):
        global gpio_left_wheel_1,gpio_left_wheel_2,gpio_right_wheel_1,gpio_right_wheel_2
        try:
            with open(project_dir+"src/" + robot_config_dir + "/robot_models/"+str(rosparam.get_param("robot_model"))+".json","r") as file:
                json_object = json.load(file)
                file.close()
                gpio_left_wheel_1 = json_object['gpio_left_wheel_1']
                gpio_left_wheel_2 = json_object['gpio_left_wheel_2']
                gpio_right_wheel_1 = json_object['gpio_right_wheel_1']
                gpio_right_wheel_2 = json_object['gpio_right_wheel_2']
                if(gpio_left_wheel_1 != -1 and gpio_left_wheel_2 != -1 and gpio_right_wheel_1 != -1 and gpio_right_wheel_2 != -1):
                    rosparam.set_param("gpio_left_wheel_1",str(gpio_left_wheel_1))
                    rosparam.set_param("gpio_left_wheel_2",str(gpio_left_wheel_2))
                    rosparam.set_param("gpio_right_wheel_1",str(gpio_right_wheel_1))
                    rosparam.set_param("gpio_right_wheel_2",str(gpio_right_wheel_2))
                    services.do_log_info("Robot model loaded successfully from " + str(rosparam.get_param("robot_model")) + ".json",current_file)
                else:
                    rosparam.set_param("gpio_left_wheel_1","-1")
                    rosparam.set_param("gpio_left_wheel_2","-1")
                    rosparam.set_param("gpio_right_wheel_1","-1")
                    rosparam.set_param("gpio_right_wheel_2","-1")
                    services.do_log_error("Could not load robot model (" + str(rosparam.get_param("robot_model")) + ") properly. Check for invalid values.",current_file)
        except Exception as e:
            services.do_log_error("Could not load robot_model (" + str(rosparam.get_param("robot_model")) + "). " + str(e),current_file)        

## Executa as rotinas de setup.
if __name__ == "__main__":
    if(services.wait_for_services_availability()):
        get_version()
        get_robot_model()
    else:
        services.do_log_error("Time limit reached when waiting for used services to respond.","setup.py")