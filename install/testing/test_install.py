#!/usr/bin/env python3

# Script que realiza os testes para descobrir se a instalação foi bem sucedida.

import os

# Caminhos para as pastas.
user: str = os.getlogin()
home: str = "/home/" + user + "/"
catkin_ws_dir: str = home + "catkin_ws/"

# Constantes utilizadas para pintar o texto.
blue: str = '\033[94m'
green: str = '\033[92m'
red: str = '\033[91m'
yellow: str = '\033[93m'
end: str = '\033[0m'

## Recebe um texto e o retorna com uma cor específica.
def set_color(color: str,text: str) -> str:
    return color + text + end

# Variáveis para o resultado dos testes.
catkin_folder_exists = set_color(red,"NO")
files_copied = set_color(red,"NO")
compilation_done = set_color(red,"NO")
source_bashrc = set_color(red,"NO")
source_zshrc = set_color(red,"NO")

## Testa se a pasta de projetos do ROS existe.
def test_catkin_folder_exists() -> None:
    global catkin_folder_exists
    try:
        if(os.path.exists(catkin_ws_dir)):
            catkin_folder_exists = set_color(green,"OK")
    except:
        pass

## Testa se os arquivos do agrobot foram copiados.
def test_files_where_copied() -> None:
    global files_copied
    try:
        if(os.path.exists(catkin_ws_dir+"src/agrobot/")):
            files_copied = set_color(green,"OK")
    except:
        pass

## Testa se o código foi compilado com sucesso.
def test_compilation() -> None:
    global compilation_done
    aux1,aux2,aux3 = False,False,False
    try:
        if(os.path.exists(catkin_ws_dir+"devel")):
            aux1 = True
        if(os.path.exists(catkin_ws_dir+"build")):
            aux2 = True
        if(os.path.exists(catkin_ws_dir+"src/CMakeLists.txt")):
            aux3 = True
    except:
        pass

    if(aux1 == True and aux2 == True and aux3 == True):
        compilation_done = set_color(green,"OK")

## Testa se a função source foi utilizada para o bashrc.
def test_source_bashrc() -> None:
    global source_bashrc
    bashrc_path = home + ".bashrc"
    try:
        with open(bashrc_path,"r") as file:
            for line in file.readlines():
                line = line.rstrip('\n')
                if(line == "source " + catkin_ws_dir + "devel/setup.bash"):
                    source_bashrc = set_color(green,"OK")
            file.close()
    except:
        pass

## Testa se a função source foi utilizada para o zshrc.
def test_source_zshrc() -> None:
    global source_zshrc
    zshrc_path = home + ".zshrc"
    try:
        with open(zshrc_path,"r") as file:
            for line in file.readlines():
                line = line.rstrip('\n')
                if(line == "source " + catkin_ws_dir + "devel/setup.zsh"):
                    source_zshrc = set_color(green,"OK")
            file.close()
    except:
        pass

def calc_installation_percent() -> float:
    count = 0
    total = 5
    if(catkin_folder_exists == set_color(green,"OK")):
        count = count + 1
    if(files_copied == set_color(green,"OK")):
        count = count + 1
    if(compilation_done == set_color(green,"OK")):
        count = count + 1
    if(source_bashrc == set_color(green,"OK")):
        count = count + 1
    if(source_zshrc == set_color(green,"OK")):
        count = count + 1
    if(count == 0):
        return 0.0
    return (count*100) / total

## Imprime na tela o resultado dos testes.
def tests_results() -> None:
    os.system("clear")
    installattion_result = calc_installation_percent()
    if(installattion_result == 100.0):
        print(set_color(green,"Successfully Installation."))
    else:
        print(set_color(red,"Could not Install properly. Check log files under install/logs/log.txt for more details."))

## Executa as rotinas de teste.
if __name__ == "__main__":
    test_catkin_folder_exists()
    test_files_where_copied()
    test_compilation()
    test_source_bashrc()
    test_source_zshrc()
    tests_results()