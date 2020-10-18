#!/usr/bin/env python3

import os,pathlib
from shutil import which

user: str = os.getlogin()
home: str = "/home/" + user + "/"
project_dir: str = str(pathlib.Path(__file__).parent.absolute()) + "/../src/agrobot/"
catkin_ws_dir: str = home + "catkin_ws/"

blue: str = '\033[94m'
green: str = '\033[92m'
red: str = '\033[91m'
yellow: str = '\033[93m'
end: str = '\033[0m'

def set_color(color: str,text: str):
    return color + text + end

catkin_folder_creation = set_color(red,"NO")
src_copied = set_color(red,"NO")
src_compiled = set_color(red,"NO")
bashrc_sourced = set_color(red,"NO")
zshrc_sourced = set_color(red,"NO")
error = [""]
warning = [""]
info = [""]

def create_catkin_folder():
    global catkin_folder_creation,error,warning,info
    try:
        os.mkdir(home+"catkin_ws")
        info.append("- catkin_ws folder created.")
    except:
        warning.append("- catkin_ws folder already exists.")
        catkin_folder_creation = set_color(green,"OK")
    catkin_folder_creation = set_color(green,"OK")

def copy_src_to_catkin_ws():
    global src_copied,error,warning,info
    try:
        os.system("cp -r " + project_dir + " " + catkin_ws_dir+"src/")
        info.append("- project folder copied to catkin_ws.")
        src_copied = set_color(green,"OK")
    except:
        error.append("- project folder could not be copied to catkin_ws.")
        src_copied = set_color(red,"NO")

def compile_src():
    global src_compiled,error,warning,info
    try:
        os.system("cd " + catkin_ws_dir + " && catkin_make")
        info.append("- src files where successfully compiled.")
        src_compiled = set_color(green,"OK")
    except:
        error.append("- src files could not be compiled.")
        src_compiled = set_color(red,"NO")

def source_bashrc():
    global bashrc_sourced,error,warning,info
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
                error.append("- Error (0) trying to source .bashrc.")
                bashrc_sourced = set_color(red,"NO")
        else:
            warning.append("- .bashrc is already sourced.")
            bashrc_sourced = set_color(green,"OK")
    except:
        error.append("- Error (1) trying to source .bashrc.")
        bashrc_sourced = set_color(red,"NO")

def source_zsh():
    global zshrc_sourced,error,warning,info
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
                    error.append("- Error (0) trying to source .zshrc.")
                    zshrc_sourced = set_color(red,"NO")
            else:
                warning.append("- .zshrc is already sourced.")
                zshrc_sourced = set_color(green,"OK")
        except:
            error.append("- Error (1) trying to source .bashrc.")
            zshrc_sourced = set_color(red,"NO")

    else:
        warning.append("* zsh is not installed. Skipping zsh sourcing.")
        zshrc_sourced = set_color(red,"NO")

def instalattion_resume():
    os.system("clear")
    print(set_color(blue,"-> Instalattion Resume\n"))
    print(" * Create catkin_ws folder: " + catkin_folder_creation)
    print(" * Copy project folder:     " + src_copied)
    print(" * Compile src files:       " + src_compiled)
    print(" * Source .bashrc:          " + bashrc_sourced)
    print(" * Source .zshrc:           " + zshrc_sourced)
    print(set_color(green,"\n---------------INFO---------------------"))
    for inf in info:
        print(" " + inf)
    print(set_color(yellow,"\n--------------WARNINGS------------------"))
    for war in warning:
        print(" " + war)
    print(set_color(red,"\n---------------ERRORS-------------------"))
    for err in error:
        print(" " + err)

if __name__ == "__main__":
    create_catkin_folder()
    copy_src_to_catkin_ws()
    compile_src()
    source_bashrc()
    source_zsh()
    instalattion_resume()