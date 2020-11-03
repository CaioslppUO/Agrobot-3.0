#!/usr/bin/env python3

import os,pathlib
from typing import Final
from shutil import which

# Caminhos para as pastas.
user: Final = os.getlogin()
home: Final = "/home/" + user + "/"
current_dir: Final = str(pathlib.Path(__file__).parent.absolute()) + "/"
catkin_ws_dir: Final = home + "catkin_ws/"

def install_ros() -> None:

    def setup_sources_and_keys() -> None:
        command: str = "sudo sh -c 'echo 'deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main' > /etc/apt/sources.list.d/ros-latest.list'"
        os.system(command)
        command = "sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654"
        os.system(command)

    def install() -> None:
        command: str = "sudo apt update"
        os.system(command)
        command = "sudo apt install ros-noetic-desktop"
        os.system(command)

    def source_bashrc() -> None:
        bashrc_path: str = home + ".bashrc"
        already_sourced: bool = False
        try:
            with open(bashrc_path,"r") as file:
                for line in file.readlines():
                    line = line.rstrip('\n')
                    if(line == "source /opt/ros/noetic/setup.bash"):
                        already_sourced = True
                file.close()
            if(already_sourced == False):
                try:
                    with open(bashrc_path,"a") as file:
                        file.write("source /opt/ros/noetic/setup.bash")
                        file.close()
                except Exception as e:
                    print(str(e))
        except Exception as e:
            print(str(e))

    def source_zshrc() -> None:
        if(which("zsh") is not None):
            zshrc_path: str = home + ".zshrc"
            already_sourced: bool = False
            try:
                with open(zshrc_path,"r") as file:
                    for line in file.readlines():
                        line = line.rstrip('\n')
                        if(line == "source /opt/ros/noetic/setup.zsh"):
                            already_sourced = True
                    file.close()
                if(already_sourced == False):
                    try:
                        with open(zshrc_path,"a") as file:
                            file.write("source /opt/ros/noetic/setup.zsh")
                            file.close()
                    except Exception as e:
                        print(str(e))
            except Exception as e:
                print(str(e))

    setup_sources_and_keys()
    install()
    source_bashrc()
    source_zshrc()

def install_i2c() -> None:
    command: str = "sudo apt-get install -y python-smbus i2c-tools"
    os.system(command)
    command = "sudo raspi-config"
    os.system(command)

def configure_usb_ports() -> None:
    command: str = "sudo usermod -a -G dialout " + user
    os.system(command)
    ########### Escrever no arquivo o nome das portas.

def install_ssh() -> None:
    pass

def upgrade_system() -> None:
    command: str = "sudo apt upgrade -y"
    os.system(command)

def configure_access_points() -> None:
    pass

if __name__ == "__main__":
    install_ros()
    install_i2c()