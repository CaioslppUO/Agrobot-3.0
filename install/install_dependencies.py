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
    exec("yes | sudo apt install nodejs")
    exec("yes | sudo npm install -g yarn")
    exec("yes | sudo apt install python-pip")
    exec("yes | sudo python -m pip install --upgrade pip")
    exec("yes | sudo pip install empy")
    exec("yes | sudo pip install pyserial")
    exec("yes | sudo pip install pycryptodome")
    exec("yes | sudo pip install pycryptodomex")
    exec("yes | sudo python -m pip install --upgrade setuptools wheel")
    exec("yes | sudo python -m pip install tqdm")
    exec("yes | sudo python -m pip install --user --upgrade twine")

## Executa as rotinas de instalação.
if __name__ == "__main__":
    install()