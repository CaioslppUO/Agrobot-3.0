#!/usr/bin/env python3

"""
@package test_priority.
Testes para o controle de prioridade do script priority_decider.
"""

# O teste de prioridade é realizado somente com o web_server e o control_lidar.

import os,rosparam,rospy
from std_msgs.msg import String

# Definição do nó.
rospy.init_node("test_priority.py",anonymous=True)

## Retorna um parâmetro do tipo inteiro do rosparam, ou -1 caso não exista.
def get_param(param_name: str) -> int:
    try:
        return int(rosparam.get_param(param_name))
    except:
        return -1

## Envia um comando de teste para o tópico web_server.
def send_to_web_server() -> None:
    try:
        os.system("rostopic pub -1 /web_server geometry_msgs/Twist -- '[-1.0, -1.0, -1.0]' '[-1.0, -1.0, -1.0]'")
    except:
        pass

## Envia 'n' + 2 comandos para o control_lidar.
def send_to_control_lidar() -> None:
    guarantee_commands: int = get_param("GUARANTEED_COMMANDS")
    count: int = 0
    while(count < guarantee_commands+2):
        try:
            os.system("rostopic pub -1 /control_lidar geometry_msgs/Twist -- '[-1.0, -1.0, -1.0]' '[-1.0, -1.0, -1.0]'")
        except:
            pass
        count = count + 1

## Envia o sinal para o test_priority_decider de que todos os testes acabaram.
def send_test_end_signal() -> None:
    try:
        os.system("rostopic pub -1 /test_priority_decider std_msgs/String 'OK'")
    except:
        pass

## Callback que executa os testes do test_priority.py
def test_talk_callback(msg: String) -> None:
    send_to_web_server()
    send_to_control_lidar()
    send_test_end_signal()

## Executa os testes quando receber o sinal do test_talk.py de que ele finalizou seus testes.
def listen_test_talk() -> None:
    rospy.Subscriber("/test_talk",String,test_talk_callback)
    rospy.spin()

if __name__ == "__main__":
    listen_test_talk()