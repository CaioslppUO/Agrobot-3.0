#!/usr/bin/env python3

# Script que realiza os testes para descobrir se a instalação foi bem sucedida.

import os,pathlib
from datetime import datetime
from shutil import which

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
catkin_folder_exists = set_color(red,"NO")
files_copied = set_color(red,"NO")
compilation_done = set_color(red,"NO")
source_bashrc = set_color(red,"NO")
source_zshrc = set_color(red,"NO")

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

## Testa se a pasta de projetos do ROS existe.
def test_catkin_folder_exists() -> None:
    global catkin_folder_exists
    try:
        if(os.path.exists(catkin_ws_dir)):
            catkin_folder_exists = set_color(green,"OK")
    except:
        pass

## Testa se os arquivos do agrobot foram copiados.
def test_files_where_copied() -> None:
    global files_copied
    try:
        if(os.path.exists(catkin_ws_dir+"src/agrobot/")):
            files_copied = set_color(green,"OK")
    except:
        pass

## Testa se o código foi compilado com sucesso.
def test_compilation() -> None:
    global compilation_done
    aux1,aux2,aux3 = False,False,False
    try:
        if(os.path.exists(catkin_ws_dir+"devel")):
            aux1 = True
        if(os.path.exists(catkin_ws_dir+"build")):
            aux2 = True
        if(os.path.exists(catkin_ws_dir+"src/CMakeLists.txt")):
            aux3 = True
    except:
        pass

    if(aux1 == True and aux2 == True and aux3 == True):
        compilation_done = set_color(green,"OK")

## Testa se a função source foi utilizada para o bashrc.
def test_source_bashrc() -> None:
    global source_bashrc
    bashrc_path = home + ".bashrc"
    try:
        with open(bashrc_path,"r") as file:
            for line in file.readlines():
                line = line.rstrip('\n')
                if(line == "source " + catkin_ws_dir + "devel/setup.bash"):
                    source_bashrc = set_color(green,"OK")
            file.close()
    except:
        pass

## Testa se a função source foi utilizada para o zshrc.
def test_source_zshrc() -> None:
    global source_zshrc
    if(which("zsh") is not None):
        zshrc_path = home + ".zshrc"
        try:
            with open(zshrc_path,"r") as file:
                for line in file.readlines():
                    line = line.rstrip('\n')
                    if(line == "source " + catkin_ws_dir + "devel/setup.zsh"):
                        source_zshrc = set_color(green,"OK")
                file.close()
        except:
            pass
    else:
        source_zshrc = set_color(green,"OK")
        do_log("<test_install.py> [WARNING] Zsh was not found while testing but no errors will be produced.")

## Calcula a procentagem que deu certo da instalação.
def calc_installation_percent() -> float:
    count = 0
    total = 5
    if(catkin_folder_exists == set_color(green,"OK")):
        count = count + 1
    else:
        do_log("<test_install.py> [ERROR] Could not find catkin_ws folder.")
    if(files_copied == set_color(green,"OK")):
        count = count + 1
    else:
         do_log("<test_install.py> [ERROR] Could not copy files to catkin_ws/src/agrobot/")
    if(compilation_done == set_color(green,"OK")):
        count = count + 1
    else:
         do_log("<test_install.py> [ERROR] Could not compile the src files.")
    if(source_bashrc == set_color(green,"OK")):
        count = count + 1
    else:
         do_log("<test_install.py> [ERROR] Could not source .bashrc.")
    if(source_zshrc == set_color(green,"OK")):
        count = count + 1
    else:
         do_log("<test_install.py> [WARNING] Could not source .zshrc. It may be caused by missing zsh installation.")
    if(count == 0):
        return 0.0
    return (count*100) / total

## Imprime na tela o resultado dos testes.
def tests_results() -> None:
    installattion_result = calc_installation_percent()
    if(installattion_result == 100.0):
        print(set_color(green,"Successfully Installation."))
    else:
        print(set_color(red,"Could not Install properly. Check log files under " + current_dir + "../logs/log.txt for more details."))

## Executa as rotinas de teste.
if __name__ == "__main__":
    test_catkin_folder_exists()
    test_files_where_copied()
    test_compilation()
    test_source_bashrc()
    test_source_zshrc()
    tests_results()