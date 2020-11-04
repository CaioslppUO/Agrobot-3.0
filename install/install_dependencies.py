#!/usr/bin/env python3

import os

## Executa um comando no shell.
def exec(command: str) -> None:
    try:
        os.system(command)
    except Exception as e:
        print(str(e))

## Instala todas as dependências utilizadas.
def install() -> None:
    exec("sudo chmod 777 -R ~/.local/")
    exec("sudo apt update")
    exec("curl -sL https://deb.nodesource.com/setup_12.x | sudo -E bash -")
    exec("sudo apt install nodejs")
    exec("sudo npm install -g yarn")
    exec("sudo apt install python-pip")
    exec("sudo python -m pip install --upgrade pip")
    exec("sudo pip install empy")
    exec("sudo pip install pyserial")
    exec("sudo pip install pycryptodome")
    exec("sudo pip install pycryptodomex")
    exec("sudo python -m pip install --upgrade setuptools wheel")
    exec("sudo python -m pip install tqdm")
    exec("sudo python -m pip install --user --upgrade twine")

## Executa as rotinas de instalação.
if __name__ == "__main__":
    install()