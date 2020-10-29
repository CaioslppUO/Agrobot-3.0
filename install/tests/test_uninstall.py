#!/usr/bin/env python3

import os,pathlib
from datetime import datetime
from typing import Final

# Caminhos para as pastas.
user: Final = os.getlogin()
home: Final = "/home/" + user + "/"
catkin_ws_dir: Final = home + "catkin_ws/"
current_dir: Final = str(pathlib.Path(__file__).parent.absolute()) + "/"

# Variáveis de controle de bug. Utilizadas para saber se as funções rodaram corretamente ou não. Impedem a execução de funções com dependência.
uninstalled: bool = False

# Constantes utilizadas para pintar o texto.
blue: Final = '\033[94m'
green: Final = '\033[92m'
red: Final = '\033[91m'
yellow: Final = '\033[93m'
end: Final = '\033[0m'

## Recebe um texto e o retorna com uma cor específica.
def set_color(color: str,text: str) -> str:
    return color + text + end

# Variáveis para o resultado dos testes.
agrobot_folder_not_exists = set_color(red,"NO")
sym_links_removed = set_color(red,"NO")

## Escreve mensagens de log no arquivo de logs.
def do_log(msg: str) -> None:
    if(not os.path.exists(current_dir+"../logs/")):
        os.mkdir(current_dir+"../logs/")
    try:
        with open(current_dir+"../logs/log.txt","a") as file:
            current_time = datetime.now().strftime("%H:%M:%S")
            file.write("(" + current_time + ") " + msg+"\n")
            file.close()
    except Exception as e:
        print("[ERROR] Could not log msg properly. "+str(e))

## Testa se a pasta agrobot existe.
def test_agrobot_folder_exists() -> bool:
    global agrobot_folder_not_exists
    try:
        if(not os.path.exists(catkin_ws_dir+"src/agrobot")):
            agrobot_folder_not_exists = set_color(green,"OK")
        return True
    except:
        return False

## Testa se os links simbólicos criados para o código fonte no python path foram removidos.
def test_sym_links_removed() -> bool:
    global sym_links_removed
    
    def get_python_version() -> str:
        try:
            python_version = "-1"
            command = "python3 --version > " + current_dir+"python_version.tmp"
            os.system(command)
            with open(current_dir+"python_version.tmp","r") as file:
                for line in file.readlines():
                    line = line.rstrip('\n')
                    line = line.split(" ")
                    line = line[1].split(".")
                    line = line[0] + "." + line[1]
                    python_version = line
                file.close()
            if(os.path.exists(current_dir+"python_version.tmp")):
                os.system("rm " + current_dir+"python_version.tmp")
            return python_version
        except Exception as e:
            do_log("<test_install.py> [ERROR] Could not get python 3 version. "+str(e))

    try:
        paths_to_check_uninstall = ["robot_utils","test_robot_utils"]
        python_version = get_python_version()
        sym_links_removed = set_color(green,"OK")
        for path in paths_to_check_uninstall:
            if(os.path.exists("/usr/lib/python"+python_version+"/site-packages/"+path)):
                sym_links_removed = set_color(red,"NO")
                break
        return True
    except Exception as e:
        sym_links_removed = set_color(red,"NO")
        do_log("<test_uninstall.py> [ERROR] Some of the symlinks could not be checked. "+str(e))
        return False

## Auxiliar para o cálculo do sucesso da desinstalação.
def calc_uninstallation_aux(variable_to_check: str, log_msg: str) -> int:
    if(variable_to_check == set_color(green,"OK")):
        return 1
    do_log(log_msg)
    return 0

## Calcula a procentagem que deu certo da desinstalação.
def calc_uninstallation_percent() -> float:
    count = 0
    total = 2
    # Precisa passar no teste (Entra para o total).
    count += calc_uninstallation_aux(agrobot_folder_not_exists,"<test_uninstall.py> [ERROR] Could not exclude catkin_ws/src/agrobot/")
    count += calc_uninstallation_aux(sym_links_removed,"<test_uninstall.py> [ERROR] Could not remove symlinks.")
    if(count == 0):
        return 0.0
    return (count*100) / total

## Roda os testes de desinstalação.
def run_uninstall_tests():
    global uninstalled
    uninstalled = test_agrobot_folder_exists() and test_sym_links_removed()

## Imprime na tela o resultado dos testes.
def tests_results() -> None:
    uninstallattion_result = calc_uninstallation_percent()
    if(uninstallattion_result == 100.0):
        print(set_color(green,"Successfully Uninstalled."))
    else:
        print(set_color(red,"Could not Uninstall properly. Check log files under " + current_dir + "../logs/log.txt for more details."))

## Executa as rotinas de teste.
if __name__ == "__main__":
    run_uninstall_tests()
    tests_results()