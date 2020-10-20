#!/usr/bin/env python3

# Script que remove a instalação do código do agrobot e instala a versão atual.
# OBS: A pasta catkin_ws é mantida, juntamente com os sources no .bashrc e .zshrc.

import os,pathlib
from shutil import rmtree
from datetime import datetime

# Caminhos para as pastas.
user: str = os.getlogin()
home: str = "/home/" + user + "/"
current_dir: str = str(pathlib.Path(__file__).parent.absolute()) + "/"
catkin_ws_dir: str = home + "catkin_ws/"

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
        print("<uninstall.py> [ERROR] Could not log msg properly.")

## Tenta remover a pasta do agrobot.
def remove_agrobot_folder() -> None:
    try:
        rmtree(catkin_ws_dir+"src/agrobot", ignore_errors=True)
    except:
        do_log("<uninstall.py> [WARNING] Could not remove catkin_ws/src/agrobot/.")

## Remove o source do .bashrc para a pasta catkin_ws, caso exista.
def remove_bashrc_source() -> None:
    bashrc_path = home + ".bashrc"
    text_to_copy = ""
    try:
        with open(bashrc_path,"r") as file:
            for line in file.readlines():
                line = line.rstrip('\n')
                if(line != "source " + catkin_ws_dir + "devel/setup.bash"):
                    text_to_copy += line + "\n"
            file.close()
        try:
            with open(bashrc_path,"w") as file:
                file.write(text_to_copy)
                file.close()
        except:
            do_log("<uninstall.py> [ERROR] Could not write to .bashrc file.")
    except:
        do_log("<uninstall.py> [ERROR] Could not read from .bashrc file.")

## Remove o source do .zshrc para a pasta catkin_ws, caso exista.
def remove_zshrc_source() -> None:
    zshrc_path = home + ".zshrc"
    text_to_copy = ""
    try:
        with open(zshrc_path,"r") as file:
            for line in file.readlines():
                line = line.rstrip('\n')
                if(line != "source " + catkin_ws_dir + "devel/setup.zsh"):
                    text_to_copy += line + "\n"
            file.close()
        try:
            with open(zshrc_path,"w") as file:
                file.write(text_to_copy)
                file.close()
        except:
            do_log("<uninstall.py> [ERROR] Could not write to .zshrc file.")
    except:
        do_log("<uninstall.py> [WARNING] Could not read from .zshrc file. Maybe file doesn't exists because zsh is not installed.")

## Tenta recompilar a pasta de projetos do ROS. Caso a pasta catkin_ws não exista, tenta remover o source do .bashrc e .zshrc.
def recompile_catkin_ws_dir() -> None:
    try:
        if(os.path.exists(catkin_ws_dir)):
            os.system("cd " + catkin_ws_dir + " && catkin_make")
        else:
            remove_bashrc_source()
            remove_zshrc_source()
            do_log("<uninstall.py> [WARNING] Since catkin_ws folder wasn't found, source from .bashrc and .zshrc where removed.")
    except:
        do_log("<uninstall.py> [ERROR] Could not compile catkin_ws folder.")

## Testa se a instalação ocorreu conforme o esperado e imprime o resultado na tela.
def test_uninstallattion() -> None:
    try:
        if(os.path.exists(current_dir+"testing/test_uninstall.py")):
            os.system(current_dir+"testing/./test_uninstall.py")
        else:
            do_log("<uninstall.py> [ERROR] Could not run uninstallation tests. Code = (0)")
    except:
        do_log("<uninstall.py> [ERROR] Could not run uninstallation tests. Code = (1)")

## Executa as rotinas de remoção do código do agrobot.
if __name__ == "__main__":
    do_log("---------UNINSTALL---------")
    remove_agrobot_folder()
    recompile_catkin_ws_dir()
    test_uninstallattion()