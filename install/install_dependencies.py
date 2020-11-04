#!/usr/bin/env python3

import os,pathlib

# Variáveis de diretório.
current_directory: str = str(pathlib.Path(__file__).parent.absolute()) + "/"


## Executa um comando no shell.
def exec(command: str) -> None:
    try:
        os.system(command)
    except Exception as e:
        print(str(e))

## Instala todas as dependências utilizadas.
def install() -> None:
    exec("sudo apt update")
    exec("curl -sL https://deb.nodesource.com/setup_12.x | sudo -E bash -")
    exec("yes | sudo apt install nodejs")
    exec("yes | sudo npm install -g yarn")
    exec("yes | sudo apt install python-pip")
    exec("yes | python3 -m pip install --user virtualenv")
    exec("mkdir -p /home/$USER/.envs/agrobot_env/ && virtualenv /home/$USER/.envs/agrobot_env/")
    exec("cd /home/$USER/.envs/agrobot_env/ && source /bin/activate && pip install " + current_directory + "requirements.env")

## Executa as rotinas de instalação.
if __name__ == "__main__":
    install()