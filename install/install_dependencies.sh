#!/bin/bash

dependConfirmationPath="/home/$USER/.agrobot_dependencies_is_installed.txt"

alreadyInstalled=$(cat $dependConfirmationPath)

if [[ $alreadyInstalled != *"true"* ]]; then
    curl -sL https://deb.nodesource.com/setup_12.x | sudo -E bash -
    sudo apt install -y nodejs
    yes | sudo npm install -g yarn
    sudo apt install -y python-pip
    pip install --upgrade pip
    sudo apt install -y build-essential libssl-dev libffi-dev python-dev
    if [[ $USER == *"labiot"* ]]; then
        sudo apt install -y python3-venv
    else
        yes | python3 -m pip install virtualenv
    fi

    echo $ROS_DISTRO > ros_distro.txt
    rosd=$(head -n 1 "ros_distro.txt")

    mkdir -p /home/$USER/.envs/agrobot_env/ && python3 -m venv /home/$USER/.envs/agrobot_env/
    source /home/$USER/.envs/agrobot_env/bin/activate && pip install -r requirements.env
    if [[ $USER == *"labiot"* ]]; then
        pip install RPi.GPIO
    fi
    echo 'source /home/$USER/.envs/agrobot_env/bin/activate && source /opt/ros/'${rosd}'/setup.bash && source /home/$USER/catkin_ws/devel/setup.bash' >> /home/$USER/.bashrc
    echo 'source /home/$USER/.envs/agrobot_env/bin/activate && source /opt/ros/'${rosd}'/setup.zsh && source /home/$USER/catkin_ws/devel/setup.zsh' >> /home/$USER/.zshrc
    source /home/$USER/.bashrc

    terminal=$(ps -o 'cmd=' -p $(ps -o 'ppid=' -p $$))

    echo 'true' >> "/home/$USER/.agrobot_dependencies_is_installed.txt"
    clear
    echo 'installation done'
    if [[ $terminal == *"bash"* ]]; then
        bash
    elif [[ $terminal == *"zsh"* ]]; then
        zsh
    fi
else
    echo "dependencies already installed. If wish to reinstall, remove the sources from .bashrc,.zshrc and delete the file /home/$USER/.agrobot_dependencies_is_installed.txt"
fi