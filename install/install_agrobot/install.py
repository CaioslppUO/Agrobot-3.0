#!/usr/bin/env python3

# Script que remove instalações antigas do código do agrobot e instala a versão atual.

import os,pathlib,json,pwd,time
from shutil import which,rmtree
from utils.general import do_log,exists

# Caminhos para diretórios.
user: str = str(pwd.getpwuid(os.getuid())[0])
home: str = "/home/" + user + "/"
current_directory: str = str(pathlib.Path(__file__).parent.absolute()) + "/"
agrobot_directory: str = current_directory +  "../../src/agrobot/"
catkin_ws_directory: str = home + "catkin_ws/"

# Variáveis de controle de bug. 
# Utilizadas para saber se as funções rodaram corretamente ou não. Impedem a execução de funções com dependência.
previous_version_was_uninstalled: bool = False
installed_successfully: bool = False

## Tenta fazer a instalação das dependências do node
def install_web_server() -> bool:
    web_server_folder: str = catkin_ws_directory + "src/agrobot/server/"
    if(exists(web_server_folder)):
        try:
            os.system("cd " + web_server_folder + " && yarn install")
            return True
        except Exception as e:
            do_log("<install.py> [ERROR] Cold not install yarn web server. " + str(e))
            return False
    else:
        do_log("<install.py> [ERROR] Could not install web_server. server folder was not found.")
        return False

## Tenta remover o código antigo, caso já tenha sido instalado.
def uninstall_previous_versions() -> bool:
    uninstall_script_file: str = current_directory+"uninstall.py"
    try:
        if(exists(uninstall_script_file)):
            os.system(current_directory+"./uninstall.py")
        return True
    except Exception as e:
        do_log("<install.py> [ERROR] Could not find uninstall.py script. " + str(e))
        return False

## Tenta deletar a compilação antiga da pasta do ROS.
def remove_previous_compilation() -> bool:
    devel_directory: str = catkin_ws_directory+"devel"
    build_directory: str = catkin_ws_directory+"build"
    cmakelists_file: str = catkin_ws_directory+"src/CMakeLists.txt"
    try:
        if(exists(catkin_ws_directory)):
            if(exists(devel_directory)):
                rmtree(devel_directory, ignore_errors=True)
            if(exists(build_directory)):
                rmtree(build_directory, ignore_errors=True)
            if(exists(cmakelists_file)):
                os.system("rm " + cmakelists_file)
        return True
    except Exception as e:
        do_log("<install.py> [INFO] Could not find catkin_ws folder during remove_previous_compilation(). " + str(e))
        return False

## Tenta criar a pasta onde ficam todos os projetos ROS.
def create_catkin_folder() -> None:
    try:
        os.system("mkdir -p " + catkin_ws_directory + "src")
    except Exception as e:
        do_log("<install.py> [INFO] Tried to create catkin_ws but it already exists. " + str(e))

## Copia o código fonte do agrobot para a pasta dos projetos ROS.
def copy_agrobot_to_catkin_ws() -> bool:
    src: str = agrobot_directory
    dst: str = catkin_ws_directory+"src/"
    try:
        if(exists(dst)):
            os.system("cp -r " + src + " " + dst)
        return True
    except Exception as e:
        do_log("<install.py> [ERROR] Could not copy agrobot folder to catkin_ws/src/agrobot/. " + str(e))
        return False

## Compila a pasta catkin_ws.
def compile_catkin_ws() -> bool:
    try:
        if(exists(catkin_ws_directory)):
            if(user == "labiot"):
                os.system("cd " + catkin_ws_directory + " && catkin_make ")
            else:
                os.system("cd " + catkin_ws_directory + " && catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m")
        return True
    except Exception as e:
        do_log("<install.py> [ERROR] Could not compile catkin_ws folder. " + str(e))
        return False

## Utiliza o comando source no arquivo .bashrc.
def source_bashrc() -> None:
    bashrc_file: str = home + ".bashrc"
    setup_file: str = catkin_ws_directory + "devel/setup.bash"
    already_sourced: bool = False
    try:
        with open(bashrc_file,"r") as file:
            for line in file.readlines():
                line = line.rstrip('\n')
                if(line == "source " + setup_file):
                    already_sourced = True
            file.close()
        if(already_sourced == False):
            try:
                with open(bashrc_file,"a") as file:
                    file.write("source " + setup_file + "\n")
                    file.close()
            except Exception as e:
                do_log("<install.py> [ERROR] Could not write to .bashrc. " + str(e))
        else:
            do_log("<install.py> [INFO] .bashrc has already been sourced to catkin_ws/devel/setup.bash.")
    except Exception as e:
        do_log("<install.py> [ERROR] Could not read .bashrc file. " + str(e))

## Utiliza o comando source no arquivo .zshrc.
def source_zshrc() -> None:
    if(which("zsh") is not None):
        zshrc_file: str = home + ".zshrc"
        setup_file: str = catkin_ws_directory + "devel/setup.zsh"
        already_sourced: bool = False
        try:
            with open(zshrc_file,"r") as file:
                for line in file.readlines():
                    line = line.rstrip('\n')
                    if(line == "source " + setup_file):
                        already_sourced = True
                file.close()
            if(already_sourced == False):
                try:
                    with open(zshrc_file,"a") as file:
                        file.write("source " + setup_file + "\n")
                        file.close()
                except Exception as e:
                    do_log("<install.py> [ERROR] Could not write to .zshrc. " + str(e))
            else:
                do_log("<install.py> [INFO] .zshrc has already been sourced to catkin_ws/devel/setup.zsh.")
        except Exception as e:
            do_log("<install.py> [ERROR] Could not read .zshrc file. " + str(e))
    else:
        do_log("<install.py> [WARNING] Could not find zsh installed.")

## Pega a versão atual do projeto do arquivo README.md no repositório.
def get_current_code_version() -> str:
    version: str = "NULL"
    readme_file: str = current_directory+"../../README.md"
    try:
        with open(readme_file,"r") as file:
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
        do_log("<install.py> [ERROR] Could not read README.md while trying to get current version. " + str(e))
    if(version == "NULL"):
        do_log("<install.py> [ERROR] Could not get robot version. NULL will be used instead.")
    return version

## Atualiza o json dentro da pasta agrobot/src com a versão atual do código.
def update_version() -> bool:
    json_object = None
    info_file: str = catkin_ws_directory+"src/agrobot/info.json"
    try:
        with open(info_file,"r") as file:
            json_object = json.load(file) 
            file.close()
        with open(info_file,"w") as file:
            json_object['version'] = get_current_code_version()
            json.dump(json_object,file)
            file.close()
        return True
    except Exception as e:
        do_log("<install.py> [ERROR] Could not read info.json while trying to set current version. " + str(e))
        return False

## Instala a bilbioteca robot_utils pelo pip.
def install_robot_utils() -> None:
    agrobot_src_folder: str = catkin_ws_directory + "src/agrobot/src"
    dist_folder: str = catkin_ws_directory + "src/agrobot/src/dist/"
    try:
        os.system("cd " + agrobot_src_folder + " && python3 install_utils.py sdist bdist_wheel") # Compila e gera as pastas para instalar o pacote pelo pip.
        os.system("cd " + dist_folder + " && python3 -m pip install robot_utils-0.0.1-py3-none-any.whl") # Instala o pacote utilizando o pip.
        os.system("cd " + agrobot_src_folder + " && mv dist .dist && mv build .build && mv robot_utils.egg-info .robot_utils.egg-info") # Renomeia as pastas para pastas invisíveis.
    except Exception as e:
        do_log("<install.py> [ERROR] Could not install robot_utils " + str(e))

## Instala o script de inicialização do serviço.
def install_service_script() -> str:
    script_location: str = home + "bin/"
    script_file: str = script_location+"start_robot.sh"
    try:
        if(not exists(script_location)):
            os.mkdir(script_location)
        if(exists(script_file)):
            os.system("sudo rm " + script_file)
        script: str = ""    
        script += "#!/bin/bash\n"
        script += "source " + home + ".envs/agrobot_env/bin/activate && "
        script += "source /opt/ros/melodic/setup.bash && "
        script += "source " + catkin_ws_directory+"devel/setup.bash && "
        script += "roslaunch agrobot run.launch\n"
        try:
            with open(script_file,"w") as file:
                file.write(script)
                file.close()
                os.chmod(script_file,0o777)
        except Exception as e:
            do_log("<install.py> [ERROR] Could not create service script on /usr/bin/. " + str(e))
    except Exception as e:
        do_log("<install.py> [ERROR] Could not create " + script_location + " folder. " + str(e))
    return script_location

## Instala o serviço que inicia o código do robô automaticamente.
def install_service() -> None:
    service_directory: str = "/etc/systemd/system/"
    service_file: str = current_directory+"start_robot.service"
    try:
        service: str = ""
        service += "[Unit]\n"
        service += "Description=Serviço que inicializa o agrobot.\n\n"
        service += "[Service]\n"
        service += "Type=simple\n"
        service += "ExecStart=/bin/bash " + install_service_script() + "start_robot.sh\n\n"
        service += "[Install]\n"
        service += "WantedBy=multi-user.target\n"
        try:
            with open(service_file,"w") as file:
                file.write(service)
                file.close()
                os.chmod(service_file,0o644)
                os.system("sudo mv " + service_file + " " + service_directory)
                if(user == "labiot"):
                    os.system("sudo systemctl enable start_robot.service")
                    do_log("<install.py> [INFO] The service was installed and enabled.")
                else:
                    do_log("<install.py> [INFO] The service was installed but not started or enabled.")
        except Exception as e:
            do_log("<install.py> [ERROR] Could not write .service file. " + str(e))
    except Exception as e:
        do_log("<install.py> [ERROR] Could not run install_service_script(). " + str(e))

## Executa as rotinas que limpam instalações anteriores.
def clear_previous_installation() -> None:
    global previous_version_was_uninstalled
    previous_version_was_uninstalled = uninstall_previous_versions() and remove_previous_compilation()

## Executa as rotinas que instalam o código novo.
def install() -> None:
    global installed_successfully
    if(previous_version_was_uninstalled):
        create_catkin_folder()
        installed_successfully = copy_agrobot_to_catkin_ws() and compile_catkin_ws() and update_version()
        time.sleep(3) # Tempo para garantir que comandos executados com o os.sytem tenham tempo para executar.
    else:
        do_log("<install.py> [ERROR] Could not install. Previous version of code was not uninstalled.")

## Executa as rotinas de configuração pós instalação.
def post_installation_configurations() -> None:
    if(installed_successfully):
        install_web_server()
        source_bashrc()
        source_zshrc()
        install_robot_utils() # Biliboteca de serviços utilizados pelo agrobot.
        install_service() # Serviço que inicializa o robô automaticamente.
        time.sleep(3) # Tempo para garantir que comandos executados com o os.sytem tenham tempo para executar.
    else:
        do_log("<install.py> [ERROR] Could not configure post installation. Code was not installed.")

## Testa se a instalação ocorreu conforme o esperado e imprime o resultado na tela.
def test_installation() -> None:
    test_install_file: str = current_directory+"tests/test_install.py"
    try:
        if(exists(test_install_file)):
            os.system(current_directory+"tests/./test_install.py")
        else:
            do_log("<install.py> [ERROR] Could not run instalattion tests. Test file was not found.")
    except Exception as e:
        do_log("<install.py> [ERROR] Could not run instalattion tests. " + str(e))

## Executa as rotinas de instalação.
if __name__ == "__main__":
    try:
        do_log(">>>-------------------------------->>>START INSTALLATION<<<--------------------------------<<<\n")
        clear_previous_installation()
        install()
        post_installation_configurations()
        test_installation()
        do_log("<<<================================<<<FINISHED INSTALLATION>>>================================>>>\n")
    except Exception as e:
        do_log("<install.py> [ERROR] Could not run install.py. " + str(e))
