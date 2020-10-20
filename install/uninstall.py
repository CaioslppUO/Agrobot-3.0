#!/usr/bin/env python3

# Script que remove a instalação do código do agrobot e instala a versão atual.
# OBS: A pasta catkin_ws é mantida, juntamente com os sources no .bashrc e .zshrc.

import os,pathlib
from shutil import rmtree

# Caminhos para as pastas.
user: str = os.getlogin()
home: str = "/home/" + user + "/"
project_dir: str = str(pathlib.Path(__file__).parent.absolute()) + "/../src/agrobot/"
catkin_ws_dir: str = home + "catkin_ws/"

## Tenta remover a pasta do agrobot.
def remove_agrobot_folder() -> None:
    try:
        rmtree(catkin_ws_dir+"src/agrobot", ignore_errors=True)
    except:
        pass

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
            pass
    except:
        pass

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
            pass
    except:
        pass

## Tenta recompilar a pasta de projetos do ROS. Caso a pasta catkin_ws não exista, tenta remover o source do .bashrc e .zshrc.
def recompile_catkin_ws_dir() -> None:
    try:
        if(os.path.exists(catkin_ws_dir)):
            os.system("cd " + catkin_ws_dir + " && catkin_make")
        else:
            remove_bashrc_source()
            remove_zshrc_source()
    except:
        pass

## Testa se a instalação ocorreu conforme o esperado e imprime o resultado na tela.
def test_uninstallattion() -> None:
    try:
        if(os.path.exists(project_dir+"../../install/testing/test_uninstall.py")):
            os.system(project_dir+"../../install/testing/./test_uninstall.py")
        else:
            print("[ERROR] Could not run uninstalattion tests. Code = (0)")
    except:
        print("[ERROR] Could not run uninstalattion tests. Code = (1)")

## Executa as rotinas de remoção do código do agrobot.
if __name__ == "__main__":
    remove_agrobot_folder()
    recompile_catkin_ws_dir()
    test_uninstallattion()