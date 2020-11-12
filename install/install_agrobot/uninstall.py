#!/usr/bin/env python3

# Script que remove a instalação do código do agrobot.

import os,pathlib,pwd
from utils.general import do_log, exists

# Caminhos para as pastas.
user: str = str(pwd.getpwuid(os.getuid())[0])
home: str = "/home/" + user + "/"
current_directory: str = str(pathlib.Path(__file__).parent.absolute()) + "/"
catkin_ws_directory: str = home + "catkin_ws/"

# Variáveis de controle de bug. 
# Utilizadas para saber se as funções rodaram corretamente ou não. Impedem a execução de funções com dependência.
uninstalled_successfully: bool = False

## Remove a pasta catkin_ws.
def remove_catkin_ws_directory() -> bool:
    try:
        if(exists(catkin_ws_directory)):
            os.system("sudo rm -r " + catkin_ws_directory)
        return True
    except Exception as e:
        do_log("<uninstall.py> [WARNING] Could not remove catkin_ws. " + str(e))
        return False

## Remove o source do .bashrc para a pasta catkin_ws.
def remove_bashrc_source() -> None:
    bashrc_file: str = home + ".bashrc"
    setup_file: str = catkin_ws_directory + "devel/setup.bash"
    text_to_copy: str = ""
    if(exists(bashrc_file)):
        try:
            with open(bashrc_file,"r") as file:
                for line in file.readlines():
                    line = line.rstrip('\n')
                    if(line != "source " + setup_file):
                        text_to_copy += line + "\n"
                file.close()
            try:
                with open(bashrc_file,"w") as file:
                    file.write(text_to_copy)
                    file.close()
            except Exception as e:
                do_log("<uninstall.py> [ERROR] Could not write to .bashrc file. " + str(e))
        except Exception as e:
            do_log("<uninstall.py> [ERROR] Could not read from .bashrc file. " + str(e))

## Remove o source do .zshrc para a pasta catkin_ws.
def remove_zshrc_source() -> None:
    zshrc_file: str = home + ".zshrc"
    setup_file: str =  catkin_ws_directory + "devel/setup.zsh"
    text_to_copy: str = ""
    if(exists(zshrc_file)):
        try:
            with open(zshrc_file,"r") as file:
                for line in file.readlines():
                    line = line.rstrip('\n')
                    if(line != "source " + setup_file):
                        text_to_copy += line + "\n"
                file.close()
            try:
                with open(zshrc_file,"w") as file:
                    file.write(text_to_copy)
                    file.close()
            except Exception as e:
                do_log("<uninstall.py> [ERROR] Could not write to .zshrc file. " + str(e))
        except Exception as e:
            do_log("<uninstall.py> [WARNING] Could not read from .zshrc file. Maybe file doesn't exists because zsh is not installed. " + str(e))

## Remove a biblioteca robot_utils.
def uninstall_robot_utils() -> bool:
    dist_directory: str = catkin_ws_directory + "src/agrobot/src/.dist/"
    try:
        if(exists(dist_directory)):
            os.system("cd " + dist_directory + " && python3 -m pip uninstall -y robot_utils-0.0.1-py3-none-any.whl") # Desinstalação utilizando o pip.
        try:
            from robot_utils import services # Caso consiga importar a desinstalação deu errado.
            return False
        except:
            return True
    except Exception as e:
        do_log("<uninstall.py> [ERROR] Could not remove robot_utils. " + str(e))
        return False

## Executa as rotinas de desinstalação.
def uninstall():
    global uninstalled_successfully
    uninstalled_successfully = uninstall_robot_utils() and remove_catkin_ws_directory()

## Testa se a instalação ocorreu conforme o esperado e imprime o resultado na tela.
def test_uninstallattion() -> None:
    test_uninstall_file: str = current_directory+"tests/test_uninstall.py"
    try:
        if(exists(test_uninstall_file)):
            os.system(current_directory+"tests/./test_uninstall.py")
        else:
            do_log("<uninstall.py> [ERROR] Could not run uninstallation tests. Test file was not found.")
    except Exception as e:
        do_log("<uninstall.py> [ERROR] Could not run uninstallation tests. " + str(e))

## Executa as rotinas de remoção do código do agrobot.
if __name__ == "__main__":
    do_log(">>>-------------------------------->>>START UNINSTALLATION<<<--------------------------------<<<\n")
    uninstall()
    test_uninstallattion()
    do_log("<<<================================<<<FINISHED UNINSTALLATION>>>================================>>>")