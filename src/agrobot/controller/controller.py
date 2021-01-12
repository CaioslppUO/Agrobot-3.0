#!/usr/bin/env python3

"""
@package controller
Controle manual do robô por meio de terminal linux.
"""

import socketio
from pynput.keyboard import Listener
from threading import Thread
import os
import time

## Dados enviados para o servidor de controle do robô.
data_output = {
    'limit': 0,
    'module': False,
    'autoMode': False,
    'power': False,
    'steer': 0,
    'speed': 0
}
client = socketio.Client()
ip = "192.168.16.105"
ip = input("Digite o ip do server.\nExemplo: 192.168.1.1\n")
client.connect("http://"+ip+":3000")
sleep: float = 0.3

## Trata a tecla apertada.
def on_press(key):
    print(type(key))
    if(key.char == 'w'):
        data_output["speed"] = 100
        time.sleep(sleep)
        data_output["speed"] = 0
    if(key.char == 's'):
        data_output["speed"] = -100
        time.sleep(sleep)
        data_output["speed"] = 0
    if(key.char == 'a'):
        data_output["steer"] = -100
        time.sleep(sleep)
        data_output["steer"] = 0
    if(key.char == 'd'):
        data_output["steer"] = 100
        time.sleep(sleep)
        data_output["steer"] = 0
    if(key.char == 'm'):
        data_output["module"] = not data_output["module"]
    if(key.char == 'p'):
        data_output["power"] = not data_output["power"]
    if(key.char == 'i'):
        data_output["limit"] = 100
        time.sleep(sleep)
    if(key.char == 'k'):
        data_output["limit"] = 0
        time.sleep(sleep)


thread_server = Thread(target=Listener(on_press=on_press).start)
thread_server.daemon = True
thread_server.start()

## Loop que executa as rotinas do controlador.
while True:
    try:
        print("Para controlar use:")
        print("    w   ")
        print(" a  s  d")
        print("b para ligar o motor")
        print("m para ligar o modulo")
        print("i para aumentar o limite")
        print("k para diminuir o limite")
        print("Os valores atuais são:")
        print("speed "+str(data_output["speed"]))
        print("steer "+str(data_output["steer"]))
        print("module "+str(data_output["module"]))
        print("power "+str(data_output["power"]))
        print("limit "+str(data_output["limit"]))
        client.emit('control_update', data_output)
        time.sleep(0.1)
        os.system("clear")
    except Exception as e:
        print(str(e))
        client.disconnect()
        break
