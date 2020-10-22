#!/usr/bin/env python3

import os,pathlib

current_dir: str = str(pathlib.Path(__file__).parent.absolute()) + "/"
version_l = 0
version_r = 0

def get_current_version() -> None:
    global version_l,version_r
    try:
        with open(current_dir+"README.md","r") as file:
            for line in file.readlines():
                line = line.rstrip('\n')
                line = line.split(":")
                if(line[0] == "# Versão"):
                    line = line[1].split(".")
                    version_l = int(line[0])
                    version_r = int(line[1])
                    if(version_r == 9):
                        version_l = version_l + 1
                        version_r = 0
                    else:
                        version_r = version_r + 1
            file.close()
    except:
        print("Error trying to read README.md")

def set_next_version() -> None:
    text_to_copy = ""
    try:
        with open(current_dir+"README.md","r") as file:
            for line in file.readlines():
                line = line.rstrip('\n')
                aux = line.split(":")
                if(aux[0] != "# Versão"):
                    text_to_copy += line + "\n"
            file.close()
        with open(current_dir+"README.md","w") as file:
            file.write(text_to_copy)
            file.close()
        with open(current_dir+"README.md","a") as file:
            file.write("# Versão: " + str(version_l) + "." + str(version_r))
            file.close()
    except:
        print("Error trying to write README.md")

def git_pull():
    os.system("cd " + current_dir + " && git pull")

def git_add_commit_push():
    os.system("cd " + current_dir + " && git add .")
    os.system("cd " + current_dir + " && git commit")
    os.system("cd " + current_dir + " && git push")

if __name__ == "__main__":
    git_pull()
    get_current_version()
    set_next_version()
    git_add_commit_push()
