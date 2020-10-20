#!/usr/bin/env python3

import os,pathlib
from datetime import datetime

# Caminhos para as pastas.
user: str = os.getlogin()
home: str = "/home/" + user + "/"
catkin_ws_dir: str = home + "catkin_ws/"
current_dir: str = str(pathlib.Path(__file__).parent.absolute()) + "/"

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

## Escreve mensagens de log no arquivo de logs.
def do_log(msg: str) -> None:
    if(not os.path.exists(current_dir+"logs/")):
        os.mkdir(current_dir+"logs/")
    try:
        with open(current_dir+"logs/log.txt","a") as file:
            current_time = datetime.now().strftime("%H:%M:%S")
            file.write("(" + current_time + ") " + msg+"\n")
            file.close()
    except:
        print("[ERROR] Could not log msg properly.")

## Testa se a pasta agrobot existe.
def test_agrobot_folder_exists() -> None:
    global agrobot_folder_not_exists
    try:
        if(not os.path.exists(catkin_ws_dir+"src/agrobot")):
            agrobot_folder_not_exists = set_color(green,"OK")
    except:
        pass

## Calcula a procentagem que deu certo da desinstalação.
def calc_uninstallation_percent() -> float:
    count = 0
    total = 1
    if(agrobot_folder_not_exists == set_color(green,"OK")):
        count = count + 1
    else:
         do_log("<test_uninstall.py> [ERROR] Could not exclude catkin_ws/src/agrobot/")
    if(count == 0):
        return 0.0
    return (count*100) / total

## Imprime na tela o resultado dos testes.
def tests_results() -> None:
    os.system("clear")
    uninstallattion_result = calc_uninstallation_percent()
    if(uninstallattion_result == 100.0):
        print(set_color(green,"Successfully Uninstalled."))
    else:
        print(set_color(red,"Could not Uninstall properly. Check log files under " + current_dir + "../logs/log.txt for more details."))

## Executa as rotinas de teste.
if __name__ == "__main__":
    test_agrobot_folder_exists()
    tests_results()