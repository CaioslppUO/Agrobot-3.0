#!/usr/bin/env python3

# Script que remove a instalação do código do agrobot e instala a versão atual.
# OBS: A pasta catkin_ws é mantida, juntamente com os sources no .bashrc e .zshrc.

import os,pathlib,pwd
from utils.general import do_log

# Caminhos para as pastas.
user: str = pwd.getpwuid(os.getuid())[0]
home: str = "/home/" + user + "/"
current_dir: str = str(pathlib.Path(__file__).parent.absolute()) + "/"
catkin_ws_directory: str = home + "catkin_ws/"

# Variáveis de controle de bug. Utilizadas para saber se as funções rodaram corretamente ou não. Impedem a execução de funções com dependência.
uninstalled: bool = False

## Tenta remover a pasta do agrobot.
def remove_agrobot_folder() -> None:
    try:
        if(os.path.exists(catkin_ws_directory)):
            os.system("sudo rm -r " + catkin_ws_directory)
    except Exception as e:
        do_log("<uninstall.py> [WARNING] Could not remove catkin_ws/src/agrobot/. "+str(e))

## Remove o source do .bashrc para a pasta catkin_ws, caso exista.
def remove_bashrc_source() -> None:
    bashrc_path: str = home + ".bashrc"
    text_to_copy: str = ""
    try:
        with open(bashrc_path,"r") as file:
            for line in file.readlines():
                line = line.rstrip('\n')
                if(line != "source " + catkin_ws_directory + "devel/setup.bash"):
                    text_to_copy += line + "\n"
            file.close()
        try:
            with open(bashrc_path,"w") as file:
                file.write(text_to_copy)
                file.close()
        except Exception as e:
            do_log("<uninstall.py> [ERROR] Could not write to .bashrc file. "+str(e))
    except Exception as e:
        do_log("<uninstall.py> [ERROR] Could not read from .bashrc file. "+str(e))

## Remove o source do .zshrc para a pasta catkin_ws, caso exista.
def remove_zshrc_source() -> None:
    zshrc_path: str = home + ".zshrc"
    text_to_copy: str = ""
    try:
        with open(zshrc_path,"r") as file:
            for line in file.readlines():
                line = line.rstrip('\n')
                if(line != "source " + catkin_ws_directory + "devel/setup.zsh"):
                    text_to_copy += line + "\n"
            file.close()
        try:
            with open(zshrc_path,"w") as file:
                file.write(text_to_copy)
                file.close()
        except Exception as e:
            do_log("<uninstall.py> [ERROR] Could not write to .zshrc file. "+str(e))
    except Exception as e:
        do_log("<uninstall.py> [WARNING] Could not read from .zshrc file. Maybe file doesn't exists because zsh is not installed. "+str(e))

## Remove os sym links para os códigos do robô colocados no python path.
def uninstall_robot_utils() -> bool:
    try:
        if(os.path.exists(catkin_ws_directory + "src/agrobot/src/.dist/")):
            os.system("cd " + catkin_ws_directory + "src/agrobot/src/.dist/ && yes | python3 -m pip uninstall robot_utils-0.0.1-py3-none-any.whl")
        try:
            from robot_utils import services
            return False
        except:
            return True
    except Exception as e:
        do_log("<uninstall.py> [ERROR] Could not remove robot_utils. "+str(e))
        return False

## Executa as rotinas de desinstalação.
def uninstall():
    global uninstalled
    uninstalled = uninstall_robot_utils()
    remove_agrobot_folder()
    uninstalled = uninstalled

## Testa se a instalação ocorreu conforme o esperado e imprime o resultado na tela.
def test_uninstallattion() -> None:
    try:
        if(os.path.exists(current_dir+"tests/test_uninstall.py")):
            os.system(current_dir+"tests/./test_uninstall.py")
        else:
            do_log("<uninstall.py> [ERROR] Could not run uninstallation tests. Code = (0)")
    except Exception as e:
        do_log("<uninstall.py> [ERROR] Could not run uninstallation tests. Code = (1). "+str(e))

## Executa as rotinas de remoção do código do agrobot.
if __name__ == "__main__":
    do_log(">>>-------------------------------->>>START UNINSTALLATION<<<--------------------------------<<<\n")
    uninstall()
    test_uninstallattion()
    do_log("<<<================================<<<FINISHED UNINSTALLATION>>>================================>>>")