#!/bin/bash

# Variáveis de controle
dependConfirmationFile="/home/$USER/.agrobot_dependencies_is_installed"
if test -f "$dependConfirmationFile"; then
    alreadyInstalled=$(cat $dependConfirmationFile)
else
    alreadyInstalled="false"
fi
linuxVersion=$(lsb_release -d)
GREEN='\033[0;32m'
END='\033[0m'

# Instalação
if [[ $alreadyInstalled != *"true"* ]]; then
    # Instalação do nodejs, npm e pip
    if [[ $linuxVersion != *"Manjaro Linux"* ]]; then
        curl -sL https://deb.nodesource.com/setup_12.x | sudo -E bash -
    	sudo apt install -y nodejs
    	sudo apt install -y python-pip
	    sudo apt install -y build-essential libssl-dev libffi-dev python-dev
    else
        yes | sudo pacman -S nodejs npm
	    yes | sudo pacman -S python-pip
    fi

    # Instalação do yarn
    yes | sudo npm install -g yarn

    # Instalação do virtualenv
    if [[ $USER == *"labiot"* ]]; then
        sudo apt install -y python3-venv
    else
        yes | python3 -m pip install virtualenv
    fi

    # Criação do virtualenv e dos sources
    echo $ROS_DISTRO > ros_distro.txt
    rosd=$(head -n 1 "ros_distro.txt")
    rm "ros_distro.txt"
    mkdir -p /home/$USER/.envs/agrobot_env/ && python3 -m venv /home/$USER/.envs/agrobot_env/
    source /home/$USER/.envs/agrobot_env/bin/activate && pip install --upgrade pip
    source /home/$USER/.envs/agrobot_env/bin/activate && cat requirements | while read PACKAGE; do pip install "$PACKAGE"; done
    if [[ $USER == *"labiot"* ]]; then
        pip install RPi.GPIO
    fi
    echo 'source /home/$USER/.envs/agrobot_env/bin/activate && source /opt/ros/'${rosd}'/setup.bash && source /home/$USER/catkin_ws/devel/setup.bash' >> /home/$USER/.bashrc
    echo 'source /home/$USER/.envs/agrobot_env/bin/activate && source /opt/ros/'${rosd}'/setup.zsh && source /home/$USER/catkin_ws/devel/setup.zsh' >> /home/$USER/.zshrc
    source /home/$USER/.bashrc

    # Pós-instalação
    terminal=$(ps -o 'cmd=' -p $(ps -o 'ppid=' -p $$))
    echo 'true' >> "/home/$USER/.agrobot_dependencies_is_installed"
    clear
    printf "${GREEN}installation done${END}"
    if [[ $terminal == *"bash"* ]]; then
        bash
    elif [[ $terminal == *"zsh"* ]]; then
        zsh
    fi
else
    echo "-> dependencies already installed." 
    echo "-> If wish to reinstall do:"
    echo "    * Remove sources from .bashrc and .zshrc" 
    echo "    * Delete file /home/$USER/.agrobot_dependencies_is_installed"
fi
