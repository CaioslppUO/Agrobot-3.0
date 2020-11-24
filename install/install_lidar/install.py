#!/usr/bin/env python3

# Script que remove instalações antigas do código do lidar e instala a versão atual.

import os,pathlib,json,pwd,time
from shutil import which,rmtree
from utils.general import do_log

# Caminhos para as pastas.
user: str = str(pwd.getpwuid(os.getuid())[0])
home: str = "/home/" + user + "/"
current_directory: str = str(pathlib.Path(__file__).parent.absolute()) + "/"
project_directory: str = current_directory +  "../../src/lidar/"
catkin_ws_directory: str = home + "catkin_ws/"

# Variáveis de controle de bug. Utilizadas para saber se as funções rodaram corretamente ou não. Impedem a execução de funções com dependência.
previous_version_was_uninstalled: bool = False
installed_successfully: bool = False


## Tenta remover o código antigo, caso já tenha sido instalado.
def uninstall_previous_versions() -> bool:
    try:
        if(os.path.exists(current_directory+"uninstall.py")):
            os.system(current_directory+"./uninstall.py")
        return True
    except Exception as e:
        do_log("<install.py> [ERROR] Could not find uninstall.py script. "+str(e))
        return False

## Tenta deletar a compilação antiga da pasta do ROS.
def remove_previous_compilation() -> bool:
    try:
        if(os.path.exists(catkin_ws_directory)):
            if(os.path.exists(catkin_ws_directory+"devel")):
                rmtree(catkin_ws_directory+"devel", ignore_errors=True)
            if(os.path.exists(catkin_ws_directory+"build")):
                rmtree(catkin_ws_directory+"build", ignore_errors=True)
            if(os.path.exists(catkin_ws_directory+"src/CMakeLists.txt")):
                os.system("rm " + catkin_ws_directory+"src/CMakeLists.txt")
        return True
    except Exception as e:
        do_log("<install.py> [INFO] Could not find catkin_ws folder during remove_previous_compilation(). "+str(e))
        return False

## Tenta criar a pasta onde ficam todos os projetos ROS.
def create_catkin_folder() -> None:
    try:
        os.mkdir(catkin_ws_directory)
    except Exception as e:
        do_log("<install.py> [INFO] Tried to create catkin_ws but it already exists. "+str(e))
    try:
        os.mkdir(catkin_ws_directory+"src")
    except Exception as e:
        do_log("<install.py> [INFO] Tried to create catkin_ws/src but it already exists. "+str(e))

## Copia o código fonte do lidar para a pasta dos projetos ROS.
def copy_src_to_catkin_ws() -> bool:
    try:
        if(os.path.exists(catkin_ws_directory+"src")):
            os.system("cp -r " + project_directory + " " + catkin_ws_directory+"src/lidar/")
            os.system("cp -r " + project_directory + "../rplidar_ros " + catkin_ws_directory+"src/rplidar_ros/")
        return True
    except Exception as e:
        do_log("<install.py> [ERROR] Could not copy lidar folder to catkin_ws/src/lidar/. " +str(e))
        return False

## Compila o novo código fonte copiado.
def compile_src() -> bool:
    try:
        if(user == "labiot"):
            if(os.path.exists(catkin_ws_directory)):
                os.system("cd " + catkin_ws_directory + " && catkin_make -j1 ")
        else:
            if(os.path.exists(catkin_ws_directory)):
                os.system("cd " + catkin_ws_directory + " && catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m")
        return True
    except Exception as e:
        do_log("<install.py> [ERROR] Could not compile catkin_ws folder. "+str(e))
        return False

## Utiliza o comando source no arquivo .bashrc.
def source_bashrc() -> None:
    bashrc_path: str = home + ".bashrc"
    already_sourced: bool = False
    try:
        with open(bashrc_path,"r") as file:
            for line in file.readlines():
                line = line.rstrip('\n')
                if(line == "source " + catkin_ws_directory + "devel/setup.bash"):
                    already_sourced = True
            file.close()
        if(already_sourced == False):
            try:
                with open(bashrc_path,"a") as file:
                    file.write("source " + catkin_ws_directory + "devel/setup.bash\n")
                    file.close()
            except Exception as e:
                do_log("<install.py> [ERROR] Could not write to .bashrc. " +str(e))
        else:
            do_log("<install.py> [INFO] .bashrc already has source to catkin_ws/devel/setup.bash.")
    except Exception as e:
        do_log("<install.py> [ERROR] Could not read .bashrc file. " +str(e))

## Utiliza o comando source no arquivo .zshrc.
def source_zshrc() -> None:
    if(which("zsh") is not None):
        zshrc_path: str = home + ".zshrc"
        already_sourced: bool = False
        try:
            with open(zshrc_path,"r") as file:
                for line in file.readlines():
                    line = line.rstrip('\n')
                    if(line == "source " + catkin_ws_directory + "devel/setup.zsh"):
                        already_sourced = True
                file.close()
            if(already_sourced == False):
                try:
                    with open(zshrc_path,"a") as file:
                        file.write("source " + catkin_ws_directory + "devel/setup.zsh\n")
                        file.close()
                except Exception as e:
                    do_log("<install.py> [ERROR] Could not write to .zshrc. "+str(e))
            else:
                do_log("<install.py> [INFO] .zshrc already has source to catkin_ws/devel/setup.zsh.")
        except Exception as e:
            do_log("<install.py> [ERROR] Could not read .zshrc file. "+str(e))
    else:
        do_log("<install.py> [WARNING] Could not find zsh installed.")

## Pega a versão atual do projeto do arquivo README.md no repositório.
def get_current_version() -> str:
    version: str = "NULL"
    try:
        with open(current_directory+"../../README.md","r") as file:
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
        do_log("<install.py> [ERROR] Could not read README.md while trying to get current version. "+str(e))
    return version

## Atualiza o json dentro da pasta lidar/src com a versão atual do código.
def update_code_version_inside_src() -> bool:
    json_object = None
    try:
        with open(catkin_ws_directory+"src/lidar/info.json","r") as file:
            json_object = json.load(file) 
            file.close()
        with open(catkin_ws_directory+"src/lidar/info.json","w") as file:
            json_object['version'] = get_current_version()
            json.dump(json_object,file)
            file.close()
        return True
    except Exception as e:
        do_log("<install.py> [ERROR] Could not read info.json while trying to set current version. " + str(e))
        return False

## Instala todos os módulos no python path.
def install_robot_utils() -> None:
    try:
        os.system("cd " + catkin_ws_directory + "src/lidar/src && python3 install_utils.py sdist bdist_wheel")
        os.system("cd " + catkin_ws_directory + "src/lidar/src/dist/ && python3 -m pip install lidar_utils-0.0.1-py3-none-any.whl")
        os.system("cd " + catkin_ws_directory + "src/lidar/src/ && mv dist .dist && mv build .build && mv lidar_utils.egg-info .lidar_utils.egg-info")
    except Exception as e:
        do_log("<install.py> [ERROR] Could not install lidar_utils "+str(e))

## Instala o script de inicialização do serviço.
def install_service_script() -> str:
    script_location: str = home + "bin/"
    try:
        if(not os.path.exists(script_location)):
            os.mkdir(script_location)
        if(os.path.exists(script_location+"start_lidar.sh")):
            os.system("sudo rm " + script_location+"start_lidar.sh")
        script: str = ""    
        script += "#!/bin/bash\n"
        script += "source " + home + ".envs/agrobot_env/bin/activate && "
        script += "source /opt/ros/melodic/setup.bash && "
        script += "source " + catkin_ws_directory+"devel/setup.bash && "
        script += "export ROS_MASTER_URI=http://192.168.1.2:11311 && "
        script += "export ROS_IP=192.168.1.121 && "
        script += "roslaunch rplidar_ros rplidar.launch && "
        script += "roslaunch lidar run.launch\n"
        try:
            with open(script_location+"start_lidar.sh","w") as file:
                file.write(script)
                file.close()
                os.chmod(script_location+"start_lidar.sh",0o777)
        except Exception as e:
            do_log("<install.py> [ERROR] Could not create service script on /usr/bin. "+str(e))
    except Exception as e:
        do_log("<install.py> [ERROR] Could not create " + script_location + " folder. "+str(e))
    return script_location

## Instala o serviço que roda o código.
def install_service() -> None:
    try:
        service_location: str = "/etc/systemd/system/"
        service: str = ""
        service += "[Unit]\n"
        service += "Description=Serviço que inicializa o lidar.\n\n"
        service += "[Service]\n"
        service += "Type=simple\n"
        service += "ExecStart=/bin/bash " + install_service_script() + "start_lidar.sh\n\n"
        service += "[Install]\n"
        service += "WantedBy=multi-user.target\n"
        try:
            with open(current_directory+"start_lidar.service","w") as file:
                file.write(service)
                file.close()
                os.chmod(current_directory+"start_lidar.service",0o644)
                os.system("sudo mv " + current_directory+"start_lidar.service " + service_location)
                if(user == "labiot"):
                    os.system("sudo systemctl enable start_lidar.service")
                    do_log("<install.py> [INFO] The service was installed, started and enabled.")
                else:
                    do_log("<install.py> [INFO] The service was installed but not started and enabled.")
        except Exception as e:
            do_log("<install.py> [ERROR] Could not write .service file. "+str(e))
    except Exception as e:
        do_log("<install.py> [ERROR] Could not run install_service_script(). "+str(e))

## Executa as rotinas que limpam instalações anteriores.
def clear_previous_install() -> None:
    global previous_version_was_uninstalled
    previous_version_was_uninstalled = uninstall_previous_versions() and remove_previous_compilation()

## Executa as rotinas que instalam o código novo.
def install() -> None:
    global installed_successfully
    if(previous_version_was_uninstalled):
        create_catkin_folder()
        installed_successfully = copy_src_to_catkin_ws() and compile_src() and update_code_version_inside_src()
        time.sleep(3)
    else:
        do_log("<install.py> [ERROR] Could not install. Previous version of code was not uninstalled.")

## Executa as rotinas de configuração pós instalação.
def post_installation_configurations() -> None:
    if(installed_successfully):
        source_bashrc()
        source_zshrc()
        install_robot_utils()
        install_service()
        time.sleep(3)
    else:
        do_log("<install.py> [ERROR] Could not configure post installation. Code was not installed.")

## Testa se a instalação ocorreu conforme o esperado e imprime o resultado na tela.
def test_installation() -> None:
    try:
        if(os.path.exists(current_directory+"tests/test_install.py")):
            os.system(current_directory+"tests/./test_install.py")
        else:
            do_log("<install.py> [ERROR] Could not run instalattion tests. Code = (0)")
    except Exception as e:
        do_log("<install.py> [ERROR] Could not run instalattion tests. Code = (1). "+str(e))

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
        do_log("<install.py> [ERROR] Could not run install.py. "+str(e))
