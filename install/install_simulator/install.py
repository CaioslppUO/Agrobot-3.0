#!/usr/bin/env python3
 
import os,pwd

# Caminhos para diretórios.
user: str = str(pwd.getpwuid(os.getuid())[0])
home: str = "/home/" + user + "/"
catkin_ws_directory: str = home + "catkin_ws/"

## Instala o simulador.
def install() -> None:
    install_directory: str = catkin_ws_directory + "src/"
    bashrc_file: str = home + ".bashrc"
    zshrc_file: str = home + ".zshrc"
    try:
        os.system("yay -S gazebo")
        os.system("yay -S ros-noetic-gazebo-ros")
        os.system("cd " + install_directory + " && git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git")
        os.system("cd " + install_directory + " && git clone https://github.com/ROBOTIS-GIT/turtlebot3.git")
        os.system("cd " + install_directory + " && git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git")
        if(not os.path.exists(catkin_ws_directory)):
            os.system("mkdir -p " + catkin_ws_directory+"src/")
        if(os.path.exists(bashrc_file)):
            with open(bashrc_file,"a") as file:
                file.write("export TURTLEBOT3_MODEL=burger")
                file.close()
        if(os.path.exists(zshrc_file)):
            with open(zshrc_file,"a") as file:
                file.write("export TURTLEBOT3_MODEL=burger")
                file.close()
        os.system("cd " + catkin_ws_directory + " && catkin_make")
    except Exception as e:
        print("Could not run install. " + str(e))

## Executa as rotinas para instalar o simulador.
if __name__ == "__main__":
    install()