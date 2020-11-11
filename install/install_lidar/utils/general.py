#!/usr/bin/env python3

import os,pathlib,pwd
from datetime import datetime

# Caminhos para as pastas.
user: str = str(pwd.getpwuid(os.getuid())[0])
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