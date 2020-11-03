#!/usr/bin/env python3

import os,pathlib

current_directory: str = str(pathlib.Path(__file__).parent.absolute()) + "/"

def start_server():
    os.system("cd " + current_directory+" && yarn start& ")

if __name__ == "__main__":
    try:
        start_server()
    except Exception as e:
        print(str(e))