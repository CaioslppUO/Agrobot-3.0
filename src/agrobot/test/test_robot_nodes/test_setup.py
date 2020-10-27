#!/usr/bin/env python3

import rosparam,os
# Constantes utilizadas para pintar o texto.
blue: str = '\033[94m'
green: str = '\033[92m'
red: str = '\033[91m'
yellow: str = '\033[93m'
end: str = '\033[0m'

## Recebe um texto e o retorna com uma cor específica.
def set_color(color: str,text: str) -> str:
    return color + text + end

# Variáveis para o resultado dos testes.
version = set_color(red,"NO")
robot_model = set_color(red,"NO")
hoverboard_boards = set_color(red,"NO")
module_pinout = set_color(red,"NO")

def test_version() ->None:
    global version
    if(rosparam.get_param("/version") != ""):
        version = set_color(green,"YES")

def test_robot_model() ->None:
    global robot_model
    if(rosparam.get_param("/robot_model") != ""):
        robot_model= set_color(green,"YES")

def test_hoverboard_boards() ->None:
    global hoverboard_boards
    if(rosparam.get_param("/hoverboard_boards") != -1):
        hoverboard_boards = set_color(green,"YES")

def test_module_pinout() ->None:
    global module_pinout
    if(rosparam.get_param("/module_pinout") != -1):
        module_pinout = set_color(green,"YES")

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
    if(count == 0):
        return False
    return count == total

def run_test() -> None:
    try:
        test_hoverboard_boards()
        test_module_pinout()
        test_robot_model()
        test_version()
    except Exception as e:
        print("Erro ao executar os testes"+str(e))


## Imprime na tela o resultado dos testes.
def tests_results() -> None:
    # os.system("clear")
    print("Version  -------------------- " + version)
    print("Robot Model  ---------------- " + robot_model)
    print("Model Piout  ---------------- " + hoverboard_boards)
    print("Hoverboard Boards  ---------- " + module_pinout)
    print("----------------------------------------")
    print("Test Result ----------------- ",end="")
    if(all_tests_ok()):
        print(set_color(green,"OK"))
    else:
        print(set_color(red,"NO"))

    
if __name__ == "__main__":
    run_test()
    tests_results()
