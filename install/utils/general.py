#!/usr/bin/env python3

import os,pathlib,pwd
from datetime import datetime

# Caminhos para as pastas.
user: str = pwd.getpwuid(os.getuid())[0]
home: str = "/home/" + user + "/"
log_dir: str = str(pathlib.Path(__file__).parent.absolute()) + "/../logs/"
current_dir: str = str(pathlib.Path(__file__).parent.absolute()) + "/"

## Escreve mensagens de log no arquivo de logs.
def do_log(msg: str) -> None:
    if(not os.path.exists(log_dir)):
        os.mkdir(log_dir)
    try:
        with open(log_dir+"log.txt","a") as file:
            current_time = datetime.now().strftime("%H:%M:%S")
            file.write("(" + current_time + ") " + msg+"\n")
            file.close()
    except Exception as e:
        print("[ERROR] Could not log msg properly. "+str(e))

## Pega a versão do python instalada e a retorna.
def get_python_version() -> str:
        python_version = "-1"
        try:
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
        except Exception as e:
            do_log("<install.py> [ERROR] Could not get python 3 version. "+str(e))
        return python_version
