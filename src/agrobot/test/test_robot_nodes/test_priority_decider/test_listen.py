#!/usr/bin/env python3

import rospy,os,pathlib
from geometry_msgs.msg import Twist

# Caminhos para as pastas.
current_directory: str = str(pathlib.Path(__file__).parent.absolute()) + "/"

## Definição do nó.
rospy.init_node("test_listen",anonymous=True)

# Variáveis de controle.
topics_to_test: list = ['web_server','control_lidar']

## Limpa o arquivo temporário.
def clean_tmp_file() -> None:
    try:
        with open(current_directory+"priority.tmp","w") as file:
            file.write("")
            file.close()
    except:
        pass

## Adiciona conteúdo ao arquivo temporário.
def add_to_tmp_file(content: str) -> None:
    try:
        if(os.path.exists(current_directory+"priority.tmp")):
            with open(current_directory+"priority.tmp","a") as file:
                file.write(content+"\n")
                file.close()
    except:
        pass

## Confirma que a foi possível subscrever ao tópico e escutar a mensagem.
def test_callback(msg: Twist, topic: str) -> None:
    try:
        if(msg != None):
            add_to_tmp_file(topic)
    except:
        pass    

## Adiciona os listeners para teste.
def add_listeners() -> None:
    for topic in topics_to_test:
        rospy.Subscriber("/"+topic,Twist,test_callback,topic)
    rospy.spin()

## Executa as rotinas de teste.
if __name__ == "__main__":
    clean_tmp_file()
    add_listeners()