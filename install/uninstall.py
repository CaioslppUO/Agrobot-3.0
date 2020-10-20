#!/usr/bin/env python3

# Script que remove a instalação do código do agrobot e instala a versão atual.
# OBS: A pasta catkin_ws é mantida, juntamente com os sources no .bashrc e .zshrc.

import os,pathlib

# Caminhos para as pastas.
user: str = os.getlogin()
home: str = "/home/" + user + "/"
catkin_ws_dir: str = home + "catkin_ws/"

## Tenta remover a pasta do agrobot.
def remove_agrobot_folder() -> None:
    try:
        os.system("sudo rm -r " + catkin_ws_dir+"src/agrobot/")
    except:
        pass

## Tenta recompilar a pasta de projetos do ROS.
def recompile_catkin_ws_dir() -> None:
    try:
        os.system("cd " + catkin_ws_dir + " && catkin_make")
    except:
        pass

## Executa as rotinas de remoção do código do agrobot.
if __name__ == "__main__":
    remove_agrobot_folder()
    recompile_catkin_ws_dir()