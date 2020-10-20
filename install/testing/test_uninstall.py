#!/usr/bin/env python3

import os

# Caminhos para as pastas.
user: str = os.getlogin()
home: str = "/home/" + user + "/"
catkin_ws_dir: str = home + "catkin_ws/"

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
agrobot_folder_not_exists = set_color(red,"NO")

## Testa se a pasta agrobot existe.
def test_agrobot_folder_exists() -> None:
    global agrobot_folder_not_exists
    try:
        if(not os.path.exists(catkin_ws_dir+"src/agrobot")):
            agrobot_folder_not_exists = set_color(green,"OK")
    except:
        pass

## Imprime na tela o resultado dos testes.
def tests_results() -> None:
    os.system("clear")
    if(agrobot_folder_not_exists == set_color(green,"OK")):
        print(set_color(green,"Successfully Uninstalled."))
    else:
        print(set_color(red,"Could not Uninstall properly. Check log files under install/logs/log.txt for more details."))

## Executa as rotinas de teste.
if __name__ == "__main__":
    test_agrobot_folder_exists()
    tests_results()