#!/bin/bash

curl -sL https://deb.nodesource.com/setup_12.x | sudo -E bash -
sudo apt install -y nodejs
yes | sudo npm install -g yarn
sudo apt install -y python-pip
sudo apt install -y build-essential libssl-dev libffi-dev python-dev
if [[ $USER == *"labiot"* ]]; then
    sudo apt install -y python3-venv
else
    yes | python3 -m pip install --user virtualenv
fi

echo $ROS_DISTRO > ros_distro.txt
rosd=$(head -n 1 "ros_distro.txt")

mkdir -p /home/$USER/.envs/agrobot_env/ && python3 -m venv /home/$USER/.envs/agrobot_env/
source /home/$USER/.envs/agrobot_env/bin/activate && pip install -r requirements.env
echo 'source /home/$USER/.envs/agrobot_env/bin/activate && source /opt/ros/'${rosd}'/setup.bash && source /home/$USER/catkin_ws/devel/setup.bash' >> /home/$USER/.bashrc
echo 'source /home/$USER/.envs/agrobot_env/bin/activate && source /opt/ros/'${rosd}'/setup.zsh && source /home/$USER/catkin_ws/devel/setup.zsh' >> /home/$USER/.zshrc
source /home/$USER/.bashrc

wichsh="`ps -o pid,args| awk '$1=='"$$"'{print $2}'`"

if [[ $wichsh == *"bash"* ]]; then
    bash
elif [[ $wichsh == *"zsh"* ]]; then
    zsh
fi
