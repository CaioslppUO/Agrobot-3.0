#!/usr/bin/env python3

# Script que remove instalações antigas do código do agrobot e instala a versão atual.

import os,pathlib,json
from shutil import which

# Caminhos para as pastas.
user: str = os.getlogin()
home: str = "/home/" + user + "/"
project_dir: str = str(pathlib.Path(__file__).parent.absolute()) + "/../src/agrobot/"
catkin_ws_dir: str = home + "catkin_ws/"

## Tenta remover o código antigo, caso já tenha sido instalado.
def uninstall_previous_versions() -> None:
    try:
        os.system(project_dir+"../../install/./uninstall.py")
    except:
        pass

## Tenta deletar a compilação antiga da pasta do ROS.
def remove_previous_compilation() -> None:
    try:
        os.system("sudo rm -r " + catkin_ws_dir + "devel")
        os.system("sudo rm -r " + catkin_ws_dir + "build")
        os.system("sudo rm " + catkin_ws_dir + "src/CMakeLists.txt")
    except:
        pass

## Tenta criar a pasta onde ficam todos os projetos ROS.
def create_catkin_folder() -> None:
    try:
        os.mkdir(home+"catkin_ws")
    except:
        pass

## Copia o código fonte do agrobot para a pasta dos projetos ROS.
def copy_src_to_catkin_ws() -> None:
    try:
        os.system("cp -r " + project_dir + " " + catkin_ws_dir+"src/")
    except:
        pass

## Compila o novo código fonte copiado.
def compile_src() -> None:
    try:
        os.system("cd " + catkin_ws_dir + " && catkin_make")
    except:
        pass

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
                pass
        else:
            pass
    except:
        pass

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
                    pass
            else:
                pass
        except:
            pass
    else:
        pass

## Pega a versão atual do projeto do arquivo README.md no repositório.
def get_current_version() -> str:
    version = "NULL"
    try:
        with open(project_dir+"../../README.md","r") as file:
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
        print("Error trying to read README.md")
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
        print("Error trying to read info.json")

## Executa as rotinas de instalação.
if __name__ == "__main__":
    remove_previous_compilation()
    create_catkin_folder()
    copy_src_to_catkin_ws()
    compile_src()
    source_bashrc()
    source_zshrc()
    update_code_version_inside_src()