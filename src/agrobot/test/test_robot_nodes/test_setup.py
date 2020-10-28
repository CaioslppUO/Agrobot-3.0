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
def test_robot_model() ->None:
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
def test_module_pinout() ->None:
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

## Auxiliar para o cálculo do sucesso instalação.
def calc_installation_aux(variable_to_check: str) -> int:
    if(variable_to_check == set_color(green,"OK")):
        return 1
    return 0

## Calcula a procentagem que deu certo da desinstalação.
def all_tests_ok() -> bool:
    count = 0
    total = 4
    count += calc_installation_aux(version)
    count += calc_installation_aux(robot_model)
    count += calc_installation_aux(hoverboard_boards)
    count += calc_installation_aux(module_pinout)
    return count == total

## Executa todos os testes.
def run_test() -> None:
    try:
        test_hoverboard_boards()
        test_module_pinout()
        test_robot_model()
        test_version()
    except Exception as e:
        print("Error trying to run test_setup. "+str(e))

## Imprime na tela o resultado dos testes.
def tests_results() -> None:
    time.sleep(0.5)
    os.system("clear")
    print("===========Test Result============")
    print("  Version --------------------- " + version)
    print("  Robot Model ----------------- " + robot_model)
    print("  Model Piout ----------------- " + hoverboard_boards)
    print("  Hoverboard Boards ----------- " + module_pinout)
    print("---------------------------------")
    print("  Result ---------------------- ",end="")
    if(all_tests_ok()):
        print(set_color(green,"OK"))
    else:
        print(set_color(red,"NO"))
    print("==================================")
    os.system("pkill ros")
    
## Executa as rotinas de teste do setup.py.
if __name__ == "__main__":
    run_test()
    tests_results()
