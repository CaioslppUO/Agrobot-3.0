#!/usr/bin/env python3

# Script que remove instalações antigas do código do agrobot e instala a versão atual.

import os,pathlib,json
from shutil import which
from shutil import rmtree
from datetime import datetime

# Caminhos para as pastas.
user: str = os.getlogin()
home: str = "/home/" + user + "/"
current_dir: str = str(pathlib.Path(__file__).parent.absolute()) + "/"
project_dir: str = current_dir +  "../src/agrobot/"
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
        print("[ERROR] Could not log msg properly.")

## Tenta remover o código antigo, caso já tenha sido instalado.
def uninstall_previous_versions() -> None:
    try:
        if(os.path.exists(current_dir+"uninstall.py")):
            os.system(current_dir+"./uninstall.py")
    except:
        do_log("<install.py> [ERROR] Could not find uninstall.py script.")

## Tenta deletar a compilação antiga da pasta do ROS.
def remove_previous_compilation() -> None:
    try:
        if(os.path.exists(catkin_ws_dir)):
            if(os.path.exists(catkin_ws_dir+"devel")):
                rmtree(catkin_ws_dir+"devel", ignore_errors=True)
            if(os.path.exists(catkin_ws_dir+"build")):
                rmtree(catkin_ws_dir+"build", ignore_errors=True)
            if(os.path.exists(catkin_ws_dir+"src/CMakeLists.txt")):
                os.system("rm " + catkin_ws_dir+"src/CMakeLists.txt")
    except:
        do_log("<install.py> [INFO] Could not find catkin_ws folder during remove_previous_compilation().")

## Tenta criar a pasta onde ficam todos os projetos ROS.
def create_catkin_folder() -> None:
    try:
        os.mkdir(home+"catkin_ws")
    except:
        do_log("<install.py> [INFO] Tried to create catkin_ws but it already exists.")
    try:
        os.mkdir(catkin_ws_dir+"src")
    except:
        do_log("<install.py> [INFO] Tried to create catkin_ws/src but it already exists.")

## Copia o código fonte do agrobot para a pasta dos projetos ROS.
def copy_src_to_catkin_ws() -> None:
    try:
        if(os.path.exists(catkin_ws_dir+"src")):
            os.system("cp -r " + project_dir + " " + catkin_ws_dir+"src/agrobot/")
    except:
        do_log("<install.py> [ERROR] Could not copy agrobot folder to catkin_ws/src/agrobot/.")

## Compila o novo código fonte copiado.
def compile_src() -> None:
    try:
        if(os.path.exists(catkin_ws_dir)):
            os.system("cd " + catkin_ws_dir + " && catkin_make")
    except:
        do_log("<install.py> [ERROR] Could not compile catkin_ws folder.")

## Utiliza o comando source no arquivo .bashrc.
def source_bashrc() -> None:
    bashrc_path = home + ".bashrc"
    already_sourced = False
    try:
        with open(bashrc_path,"r") as file:
            for line in file.readlines():
                line = line.rstrip('\n')
                if(line == "source " + catkin_ws_dir + "devel/setup.bash"):
                    already_sourced = True
            file.close()
        if(already_sourced == False):
            try:
                with open(bashrc_path,"a") as file:
                    file.write("source " + catkin_ws_dir + "devel/setup.bash\n")
                    file.close()
            except:
                do_log("<install.py> [ERROR] Could not write to .bashrc.")
        else:
            do_log("<install.py> [INFO] .bashrc already has source to catkin_ws/devel/setup.bashrc.")
    except:
        do_log("<install.py> [ERROR] Could not read .bashrc file.")

## Utiliza o comando source no arquivo .zshrc.
def source_zshrc() -> None:
    if(which("zsh") is not None):
        zshrc_path = home + ".zshrc"
        already_sourced = False
        try:
            with open(zshrc_path,"r") as file:
                for line in file.readlines():
                    line = line.rstrip('\n')
                    if(line == "source " + catkin_ws_dir + "devel/setup.zsh"):
                        already_sourced = True
                file.close()
            if(already_sourced == False):
                try:
                    with open(zshrc_path,"a") as file:
                        file.write("source " + catkin_ws_dir + "devel/setup.zsh\n")
                        file.close()
                except:
                    do_log("<install.py> [ERROR] Could not write to .zshrc.")
            else:
                do_log("<install.py> [INFO] .zshrc already has source to catkin_ws/devel/setup.zsh.")
        except:
            do_log("<install.py> [ERROR] Could not read .zshrc file.")
    else:
        do_log("<install.py> [WARNING] Could not find zsh installed.")

## Pega a versão atual do projeto do arquivo README.md no repositório.
def get_current_version() -> str:
    version = "NULL"
    try:
        with open(current_dir+"../README.md","r") as file:
            for line in file.readlines():
                line = line.rstrip('\n')
                line = line.split(":")
                if(line[0] == "# Versão"):
                    line = line[1].split(".")
                    version_l = int(line[0])
                    version_r = int(line[1])
                    version = str(version_l) + "." + str(version_r)
            file.close()
    except:
        do_log("<install.py> [ERROR] Could not read README.md while trying to get current version.")
    return version

## Atualiza o json dentro da pasta agrobot/src com a versão atual do código.
def update_code_version_inside_src() -> None:
    json_object = None
    try:
        with open(catkin_ws_dir+"src/agrobot/src/info.json","r") as file:
            json_object = json.load(file) 
            file.close()
        with open(catkin_ws_dir+"src/agrobot/src/info.json","w") as file:
            json_object['version'] = get_current_version()
            json.dump(json_object,file)
            file.close()
    except:
        do_log("<install.py> [ERROR] Could not read info.json while trying to set current version.")

## Testa se a instalação ocorreu conforme o esperado e imprime o resultado na tela.
def test_installation() -> None:
    try:
        if(os.path.exists(current_dir+"testing/test_install.py")):
            os.system(current_dir+"testing/./test_install.py")
        else:
            do_log("<install.py> [ERROR] Could not run instalattion tests. Code = (0)")
    except:
        do_log("<install.py> [ERROR] Could not run instalattion tests. Code = (1)")

## Executa as rotinas de instalação.
if __name__ == "__main__":
    uninstall_previous_versions()
    remove_previous_compilation()
    create_catkin_folder()
    copy_src_to_catkin_ws()
    compile_src()
    source_bashrc()
    source_zshrc()
    update_code_version_inside_src()
    test_installation()