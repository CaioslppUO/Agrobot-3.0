#!/bin/bash

curl -sL https://deb.nodesource.com/setup_12.x | sudo -E bash -
sudo apt install -y nodejs
yes | sudo npm install -g yarn
yes | sudo apt install python-pip
if [[ $USER == *"labiot"* ]]; then
    sudo apt install -y python3-venv
else
    yes | python3 -m pip install --user virtualenv
fi
mkdir -p /home/$USER/.envs/agrobot_env/ && python3 -m venv /home/$USER/.envs/agrobot_env/
source /home/$USER/.envs/agrobot_env/bin/activate && pip install -r requirements.env
