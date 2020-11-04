#!/usr/bin/env python3

import socketio
from pynput.keyboard import Listener
from threading import Thread
import os
import time


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


def on_press(key):
    print(type(key))
    if(key.char == 'w'):
        data_output["speed"] = data_output["speed"] + 1
    if(key.char == 's'):
        data_output["speed"] = data_output["speed"] - 1
    if(key.char == 'a'):
        data_output["steer"] = data_output["steer"] - 1
    if(key.char == 'd'):
        data_output["steer"] = data_output["steer"] + 1
    if(key.char == 'm'):
        data_output["module"] = not data_output["module"]
    if(key.char == 'p'):
        data_output["power"] = not data_output["power"]


thread_server = Thread(target=Listener(on_press=on_press).start)
thread_server.daemon = True
thread_server.start()

while True:
    try:
        print("Para controlar use:")
        print("    w   ")
        print(" a  s  d")
        print("b para ligar o motor")
        print("m para ligar o modulo")
        print("Os valores atuais são:")
        print("speed "+str(data_output["speed"]))
        print("steer "+str(data_output["steer"]))
        print("module "+str(data_output["module"]))
        print("power "+str(data_output["power"]))
        client.emit('control_update', data_output)
        time.sleep(0.1)
        os.system("clear")
    except Exception as e:
        print(str(e))
        client.disconnect()
        break
