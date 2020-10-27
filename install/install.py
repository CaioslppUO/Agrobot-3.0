#!/usr/bin/env python3

# Script que remove instalações antigas do código do agrobot e instala a versão atual.

import os,pathlib,json
from shutil import which,rmtree
from utils.general import do_log,get_python_version

# Caminhos para as pastas.
user: str = os.getlogin()
home: str = "/home/" + user + "/"
current_dir: str = str(pathlib.Path(__file__).parent.absolute()) + "/"
project_dir: str = current_dir +  "../src/agrobot/"
catkin_ws_dir: str = home + "catkin_ws/"

## Tenta remover o código antigo, caso já tenha sido instalado.
def uninstall_previous_versions() -> None:
    try:
        if(os.path.exists(current_dir+"uninstall.py")):
            os.system(current_dir+"./uninstall.py")
    except Exception as e:
        do_log("<install.py> [ERROR] Could not find uninstall.py script."+str(e))

## Tenta deletar a compilação antiga da pasta do ROS.
def remove_previous_compilation() -> None:
    try:
        if(os.path.exists(catkin_ws_dir)):
            if(os.path.exists(catkin_ws_dir+"devel")):
                rmtree(catkin_ws_dir+"devel", ignore_errors=True)
            if(os.path.exists(catkin_ws_dir+"build")):
                rmtree(catkin_ws_dir+"build", ignore_errors=True)
            if(os.path.exists(catkin_ws_dir+"src/CMakeLists.txt")):
                os.system("rm " + catkin_ws_dir+"src/CMakeLists.txt")
    except Exception as e:
        do_log("<install.py> [INFO] Could not find catkin_ws folder during remove_previous_compilation()."+str(e))

## Tenta criar a pasta onde ficam todos os projetos ROS.
def create_catkin_folder() -> None:
    try:
        os.mkdir(home+"catkin_ws")
    except Exception as e:
        do_log("<install.py> [INFO] Tried to create catkin_ws but it already exists."+str(e))
    try:
        os.mkdir(catkin_ws_dir+"src")
    except Exception as e:
        do_log("<install.py> [INFO] Tried to create catkin_ws/src but it already exists."+str(e))

## Copia o código fonte do agrobot para a pasta dos projetos ROS.
def copy_src_to_catkin_ws() -> None:
    try:
        if(os.path.exists(catkin_ws_dir+"src")):
            os.system("cp -r " + project_dir + " " + catkin_ws_dir+"src/agrobot/")
    except Exception as e:
        do_log("<install.py> [ERROR] Could not copy agrobot folder to catkin_ws/src/agrobot/." +str(e))

## Compila o novo código fonte copiado.
def compile_src() -> None:
    try:
        if(os.path.exists(catkin_ws_dir)):
            os.system("cd " + catkin_ws_dir + " && catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.7m")
    except Exception as e:
        do_log("<install.py> [ERROR] Could not compile catkin_ws folder."+str(e))

## Utiliza o comando source no arquivo .bashrc.
def source_bashrc() -> None:
    bashrc_path: str = home + ".bashrc"
    already_sourced: bool = False
    try:
        with open(bashrc_path,"r") as file:
            for line in file.readlines():
                line = line.rstrip('\n')
                if(line == "source " + catkin_ws_dir + "devel/setup.bash"):
                    already_sourced = True
            file.close()
        if(already_sourced == False):
            try:
                with open(bashrc_path,"a") as file:
                    file.write("source " + catkin_ws_dir + "devel/setup.bash\n")
                    file.close()
            except Exception as e:
                do_log("<install.py> [ERROR] Could not write to .bashrc." +str(e))
        else:
            do_log("<install.py> [INFO] .bashrc already has source to catkin_ws/devel/setup.bash.")
    except Exception as e:
        do_log("<install.py> [ERROR] Could not read .bashrc file." +str(e))

## Utiliza o comando source no arquivo .zshrc.
def source_zshrc() -> None:
    if(which("zsh") is not None):
        zshrc_path: str = home + ".zshrc"
        already_sourced: bool = False
        try:
            with open(zshrc_path,"r") as file:
                for line in file.readlines():
                    line = line.rstrip('\n')
                    if(line == "source " + catkin_ws_dir + "devel/setup.zsh"):
                        already_sourced = True
                file.close()
            if(already_sourced == False):
                try:
                    with open(zshrc_path,"a") as file:
                        file.write("source " + catkin_ws_dir + "devel/setup.zsh\n")
                        file.close()
                except Exception as e:
                    do_log("<install.py> [ERROR] Could not write to .zshrc."+str(e))
            else:
                do_log("<install.py> [INFO] .zshrc already has source to catkin_ws/devel/setup.zsh.")
        except Exception as e:
            do_log("<install.py> [ERROR] Could not read .zshrc file."+str(e))
    else:
        do_log("<install.py> [WARNING] Could not find zsh installed.")

## Pega a versão atual do projeto do arquivo README.md no repositório.
def get_current_version() -> str:
    version: str = "NULL"
    try:
        with open(current_dir+"../README.md","r") as file:
            for line in file.readlines():
                line = line.rstrip('\n')
                line = line.split(":")
                if(line[0] == "# Versão"):
                    line = line[1].split(".")
                    version_l = int(line[0])
                    version_r = int(line[1])
                    version = str(version_l) + "." + str(version_r)
            file.close()
    except Exception as e:
        do_log("<install.py> [ERROR] Could not read README.md while trying to get current version."+str(e))
    return version

## Atualiza o json dentro da pasta agrobot/src com a versão atual do código.
def update_code_version_inside_src() -> None:
    json_object = None
    try:
        with open(catkin_ws_dir+"src/agrobot/info.json","r") as file:
            json_object = json.load(file) 
            file.close()
        with open(catkin_ws_dir+"src/agrobot/info.json","w") as file:
            json_object['version'] = get_current_version()
            json.dump(json_object,file)
            file.close()
    except Exception as e:
        do_log("<install.py> [ERROR] Could not read info.json while trying to set current version. " + str(e))

## Instala todos os módulos no python path.
def install_code_in_python_path() -> None:
    try:
        paths_to_copy: list = ["src/robot_nodes","src/robot_services",
            "src/robot_utils","test/test_robot_nodes",
            "test/test_robot_services","test/test_robot_utils"]
        python_version: str = get_python_version()
        for path in paths_to_copy:
            aux = path.split("/")
            if(not os.path.exists("/usr/lib/python" + python_version + "/site-packages/" + aux[1])):
                os.system("sudo ln -s " + catkin_ws_dir+"src/agrobot/"+ path + "/ /usr/lib/python" + python_version + "/site-packages/" + aux[1])
                print("Symlink created from " + catkin_ws_dir+"src/agrobot/" + path +"/ to " + "/usr/lib/python" + python_version + "/site-packages/" + aux[1] + "/")
    except Exception as e:
        do_log("<install.py> [ERROR] Could not create the symlink to robot code in python path."+str(e))

## Instala o script de inicialização do serviço.
def install_service_script() -> str:
    script_location: str = home + "bin/"
    try:
        if(not os.path.exists(script_location)):
            os.mkdir(script_location)
        if(os.path.exists(script_location+"start_robot.sh")):
            os.system("sudo rm " + script_location+"start_robot.sh")
        script: str = ""    
        script += "#!/bin/bash\n"
        script += "source /opt/ros/melodic/setup.bash && "
        script += "source " + catkin_ws_dir+"devel/setup.bash && "
        script += "roslaunch agrobot run.launch\n"
        try:
            with open(script_location+"start_robot.sh","w") as file:
                file.write(script)
                file.close()
                os.chmod(script_location+"start_robot.sh",0o777)
        except Exception as e:
            do_log("<install.py> [ERROR] Could not create service script on /usr/bin"+str(e))
    except Exception as e:
        do_log("<install.py> [ERROR] Could not create " + script_location + " folder"+str(e))
    return script_location

## Instala o serviço que roda o código.
def install_service() -> None:
    try:
        service_location: str = "/etc/systemd/system/"
        service: str = ""
        service += "[Unit]\n"
        service += "Description=Serviço que inicializa o agrobot.\n\n"
        service += "[Service]\n"
        service += "Type=simple\n"
        service += "ExecStart=/bin/bash " + install_service_script() + "start_robot.sh\n\n"
        service += "[Install]\n"
        service += "WantedBy=multi-user.target\n"
        try:
            with open(current_dir+"start_robot.service","w") as file:
                file.write(service)
                file.close()
                os.chmod(current_dir+"start_robot.service",0o644)
                os.system("sudo mv " + current_dir+"start_robot.service " + service_location)
                if(user == "labiot"):
                    os.system("sudo systemctl enable start_robot.service")
                    os.system("sudo systemctl start start_robot.service")
                    do_log("<install.py> [INFO] The service was installed, started and enabled.")
                else:
                    do_log("<install.py> [INFO] The service was installed but not started and enabled.")
        except Exception as e:
            do_log("<install.py> [ERROR] Could not write .service file."+str(e))
    except Exception as e:
        do_log("<install.py> [ERROR] Could not run install_service_script()."+str(e))

## Executa as rotinas que limpam instalações anteriores.
def clear_previous_install() -> None:
    uninstall_previous_versions()
    remove_previous_compilation()

## Executa as rotinas que instalam o código novo.
def install() -> None:
    create_catkin_folder()
    copy_src_to_catkin_ws()
    compile_src()
    update_code_version_inside_src()

## Executa as rotinas de configuração pós instalação.
def post_installation_configurations() -> None:
    source_bashrc()
    source_zshrc()
    install_code_in_python_path()
    install_service()

## Testa se a instalação ocorreu conforme o esperado e imprime o resultado na tela.
def test_installation() -> None:
    try:
        if(os.path.exists(current_dir+"tests/test_install.py")):
            os.system(current_dir+"tests/./test_install.py")
        else:
            do_log("<install.py> [ERROR] Could not run instalattion tests. Code = (0)")
    except Exception as e:
        do_log("<install.py> [ERROR] Could not run instalattion tests. Code = (1)"+str(e))

## Executa as rotinas de instalação.
if __name__ == "__main__":
    try:
        do_log(">>>-------------------------------->>>START INSTALLATION<<<--------------------------------<<<\n")
        clear_previous_install()
        install()
        post_installation_configurations()
        test_installation()
        do_log("<<<================================<<<FINISHED INSTALLATION>>>================================>>>\n")
    except Exception as e:
        do_log("<istall.py> [ERROR] Could not run install.py."+str(e))