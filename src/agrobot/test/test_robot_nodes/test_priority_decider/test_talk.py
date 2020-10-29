#!/usr/bin/env python3

import os
from typing import Final

# Variáveis de controle.
topics_to_test: Final = ['web_server','control_lidar']

## Testa se a publicação de comandos está funcionando.
def test_talk() -> None:
    try:
        for topic in topics_to_test:
            os.system("rostopic pub -1 /"+ topic + " geometry_msgs/Twist -- '[-1.0, -1.0, -1.0]' '[-1.0, -1.0, -1.0]'")
        os.system("rostopic pub -1 /test_talk std_msgs/String 'OK'") # Envia o OK para o test_priority.py começar.
    except Exception as e:
        print("Could not run <test_talk.py>. " + str(e))

## Executa as rotinas de teste.
if __name__ == "__main__":
    test_talk()