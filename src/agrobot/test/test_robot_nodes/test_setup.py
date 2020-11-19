#!/usr/bin/env python3

import rosparam,time,os

# Constantes utilizadas para pintar o texto.
blue: str = '\033[94m'
green: str = '\033[92m'
red: str = '\033[91m'
yellow: str = '\033[93m'
end: str = '\033[0m'

## Recebe um texto e o retorna com uma cor específica.
def set_color(color: str,text: str) -> str:
    return color + text + end

# Variáveis de controle de tentativas.
limit_attempts: int = 100

# Variáveis para o resultado dos testes.
version = set_color(red,"NO")
robot_model = set_color(red,"NO")
hoverboard_boards = set_color(red,"NO")
module_pinout = set_color(red,"NO")
mini_robot_gpio = set_color(red,"NO")

## Testa se a versão está definida.
def test_version() ->None:
    global version
    count: int = 0
    parameter: str = ""
    while(parameter == "" and count < limit_attempts):
        try:
            parameter = rosparam.get_param("version")
        except:
            pass
        count = count + 1
    if(parameter != ""):
        version = set_color(green,"OK")

## Testa se o modelo do robô está definido.
def test_robot_model() -> str:
    global robot_model
    count: int = 0
    parameter: str = ""
    while(parameter == "" and count < limit_attempts):
        try:
            parameter = rosparam.get_param("robot_model")
        except:
            pass
        count = count + 1
    if(parameter != ""):
        robot_model = set_color(green,"OK")
    return parameter

## Testa se a quantidade de placas de hoverboard que serão utilizadas está definida.
def test_hoverboard_boards() ->None:
    global hoverboard_boards
    count: int = 0
    parameter: str = ""
    while(parameter == "" and count < limit_attempts):
        try:
            parameter = rosparam.get_param("hoverboard_boards")
        except:
            pass
        count = count + 1
    if(parameter != ""):
        hoverboard_boards = set_color(green,"OK")

## Testa se o pinout para o módulo extra está definido.
def test_module_pinout() -> None:
    global module_pinout
    count: int = 0
    parameter: str = ""
    while(parameter == "" and count < limit_attempts):
        try:
            parameter = rosparam.get_param("module_pinout")
        except:
            pass
        count = count + 1
    if(parameter != ""):
        module_pinout = set_color(green,"OK")

## Testa se os GPIO para o mini_robot foram carregados corretamente.
def test_gpio_for_mini_robot() -> None:
    global mini_robot_gpio
    count: int = 0
    parameter_gpio_left_wheel_1: str = ""
    parameter_gpio_left_wheel_2: str = ""
    parameter_gpio_right_wheel_1: str = ""
    parameter_gpio_right_wheel_2: str = ""
    while((parameter_gpio_left_wheel_1 == "" or parameter_gpio_left_wheel_2 == "" or parameter_gpio_right_wheel_1 == "" or parameter_gpio_right_wheel_2 == "") and count < limit_attempts):
        try:
            parameter_gpio_left_wheel_1 = rosparam.get_param("gpio_left_wheel_1")
            parameter_gpio_left_wheel_2 = rosparam.get_param("gpio_left_wheel_2")
            parameter_gpio_right_wheel_1 = rosparam.get_param("gpio_right_wheel_1")
            parameter_gpio_right_wheel_2 = rosparam.get_param("gpio_right_wheel_2")
        except:
            pass
    if(int(parameter_gpio_left_wheel_1) != -1 and parameter_gpio_left_wheel_1 != "" and 
        int(parameter_gpio_left_wheel_2) != -1 and parameter_gpio_left_wheel_2 != "" and
        int(parameter_gpio_right_wheel_1) != -1 and parameter_gpio_right_wheel_1 != "" and
        int(parameter_gpio_right_wheel_2) != -1 and parameter_gpio_right_wheel_2 != ""):
        mini_robot_gpio = set_color(green,"OK")

## Auxiliar para o cálculo do sucesso instalação.
def calc_installation_aux(variable_to_check: str) -> int:
    if(variable_to_check == set_color(green,"OK")):
        return 1
    return 0

## Calcula a procentagem que deu certo da desinstalação.
def all_tests_ok():
    count = 0
    total = 2
    count += calc_installation_aux(version)
    count += calc_installation_aux(robot_model)
    if(robot_model == set_color(green,"OK")):
        model = rosparam.get_param("robot_model")
        if(model == "mini_robot"):
            total = total + 1
            count += calc_installation_aux(mini_robot_gpio)
        else:
            total = total + 2
            count += calc_installation_aux(hoverboard_boards)
            count += calc_installation_aux(module_pinout)
    return count == total, count

## Executa todos os testes.
def run_test(model: str) -> None:
    try:
        test_version()
        if(model == "mini_robot"):
            test_gpio_for_mini_robot()
        else:
            test_hoverboard_boards()
            test_module_pinout()
    except Exception as e:
        print("Error trying to run test_setup. "+str(e))

## Imprime na tela o resultado dos testes.
def tests_results(model: str) -> None:
    time.sleep(0.5)
    os.system("clear")
    print("===========Test Result============")
    print("  Version --------------------- " + version)
    print("  Robot Model ----------------- " + robot_model)
    if(model == "mini_robot"):
        print("  Mini Robot Pinout ----------- " + mini_robot_gpio)
    else:
        print("  Module Piout ---------------- " + hoverboard_boards)
        print("  Hoverboard Boards ----------- " + module_pinout)
    print("----------------------------------")
    print("  Result ---------------------- ",end="")
    all_ok,count = all_tests_ok()
    if(all_ok):
        print(set_color(green,"OK"))
    elif(count == 2):
        print(set_color(yellow,"~~"))
    else:
        print(set_color(red,"NO"))
    print("==================================")
    os.system("pkill ros")
    
## Executa as rotinas de teste do setup.py.
if __name__ == "__main__":
    model = test_robot_model()
    run_test(model)
    tests_results(model)
