#!/usr/bin/env python3

# Script que realiza os testes para descobrir se a instalação foi bem sucedida.

import os,pathlib,time,rospy,rosservice,pwd
from datetime import datetime
from shutil import which

# Caminhos para as pastas.
user: str = pwd.getpwuid(os.getuid())[0]
home: str = "/home/" + user + "/"
catkin_ws_dir: str = home + "catkin_ws/"
current_dir: str = str(pathlib.Path(__file__).parent.absolute()) + "/"

# Variáveis de controle de bug. Utilizadas para saber se as funções rodaram corretamente ou não. Impedem a execução de funções com dependência.
installation_tests: bool = False
post_installation_tests: bool = False

# Constantes utilizadas para pintar o texto.
blue: str = '\033[94m'
green: str = '\033[92m'
red: str = '\033[91m'
yellow: str = '\033[93m'
end: str = '\033[0m'

## Recebe um texto e o retorna com uma cor específica.
def set_color(color: str,text: str) -> str:
    return color + text + end

# Variáveis para o resultado dos testes.
catkin_folder_exists = set_color(red,"NO")
files_copied = set_color(red,"NO")
compilation_done = set_color(red,"NO")
source_bashrc = set_color(red,"NO")
source_zshrc = set_color(red,"NO")
ran_properly = set_color(red,"NO")
sym_links = set_color(red,"NO")
service_script = set_color(red,"NO")
service = set_color(red,"NO")

## Escreve mensagens de log no arquivo de logs.
def do_log(msg: str) -> None:
    if(not os.path.exists(current_dir+"../logs/")):
        os.mkdir(current_dir+"../logs/")
    try:
        with open(current_dir+"../logs/log.txt","a") as file:
            current_time = datetime.now().strftime("%H:%M:%S")
            file.write("(" + current_time + ") " + msg+"\n")
            file.close()
    except Exception as e: 
        print("[ERROR] Could not log msg properly. " + str(e))

## Testa se a pasta de projetos do ROS existe.
def test_catkin_folder_exists() -> bool:
    global catkin_folder_exists
    try:
        if(os.path.exists(catkin_ws_dir)):
            catkin_folder_exists = set_color(green,"OK")
            return True
    except:
        return False

## Testa se os arquivos do agrobot foram copiados.
def test_files_where_copied() -> bool:
    global files_copied
    try:
        if(os.path.exists(catkin_ws_dir+"src/agrobot/")):
            files_copied = set_color(green,"OK")
            return True
    except:
        return False

## Testa se o código foi compilado com sucesso.
def test_compilation() -> bool:
    global compilation_done
    aux1,aux2,aux3 = False,False,False
    try:
        if(os.path.exists(catkin_ws_dir+"devel")):
            aux1 = True
        if(os.path.exists(catkin_ws_dir+"build")):
            aux2 = True
        if(os.path.exists(catkin_ws_dir+"src/CMakeLists.txt")):
            aux3 = True
    except:
        pass

    if(aux1 == True and aux2 == True and aux3 == True):
        compilation_done = set_color(green,"OK")
        return True
    return False

## Testa se a função source foi utilizada para o bashrc.
def test_source_bashrc() -> bool:
    global source_bashrc
    bashrc_path = home + ".bashrc"
    try:
        with open(bashrc_path,"r") as file:
            for line in file.readlines():
                line = line.rstrip('\n')
                if(line == "source " + catkin_ws_dir + "devel/setup.bash"):
                    source_bashrc = set_color(green,"OK")
            file.close()
        return True
    except:
        return False

## Testa se a função source foi utilizada para o zshrc.
def test_source_zshrc() -> bool:
    global source_zshrc
    if(which("zsh") is not None):
        zshrc_path = home + ".zshrc"
        try:
            with open(zshrc_path,"r") as file:
                for line in file.readlines():
                    line = line.rstrip('\n')
                    if(line == "source " + catkin_ws_dir + "devel/setup.zsh"):
                        source_zshrc = set_color(green,"OK")
                file.close()
            return True
        except:
            return False
    else:
        source_zshrc = set_color(green,"OK")
        do_log("<test_install.py> [WARNING] Zsh was not found while testing but no errors will be produced.")
        return True

## Verifica se o código foi compilado corretamente e está rodando.
def test_run() -> bool:
    global ran_properly
    ## Deleta o script temporário.
    def delete_tmp_file():
       if(os.path.exists(current_dir+"run.tmp")):
            os.system("rm " + current_dir+"run.tmp")

    ## Cria um .sh temporário para ser executado no bash.
    def create_tmp_file(content: str):
        sources_path = "source /opt/ros/$ROS_DISTRO/setup.bash && source " + catkin_ws_dir+"devel/setup.bash && source ~/.bashrc && "
        with open(current_dir+"run.tmp","w") as file:
            file.write(sources_path + content)
            os.system("chmod +x " + current_dir+"run.tmp")
            file.close()

    ## Executa o script .sh temporário.
    def run_tmp_file():
        os.system("bash -e "+current_dir+"run.tmp")

    ## Espera até todos os serviços utilizados estejam disponível.
    def wait_for_services_availability() -> bool:
        available_services: list = ['/log_info']
        services_attempt_limit: int = 20000
        limit_reached: bool = False
        index: int = 0
        for service in available_services:
            index = 0
            waiting = None
            while(waiting == None):
                try:
                    waiting = rosservice.get_service_type(service)
                except:
                    pass
                index = index + 1
                if(index > services_attempt_limit):
                    limit_reached = True
                    break
            if(limit_reached == True):
                return False
        return True

    # Rodando o código.
    try:
        delete_tmp_file()
        os.system("roscore& ")
        while(rospy.is_shutdown()):
            pass
        create_tmp_file("rosrun agrobot log.py& ")
        run_tmp_file()
        time.sleep(1)
        if(wait_for_services_availability()):
            delete_tmp_file()
            create_tmp_file("rosservice call /log_info 'Installation test (run) worked properly.' 'test_install.py'")
            run_tmp_file()
            time.sleep(2)
        os.system("pkill ros")
        delete_tmp_file()
        time.sleep(3)
    except:
        pass
    
    # Verificando se rodou.
    try:
        with open(catkin_ws_dir+"src/agrobot/log/log.txt","r") as file:
            for line in file.readlines():
                line = line.rstrip('\n')
                line = line.split("]")
                try:
                    if(line[1] == " Installation test (run) worked properly."):
                        ran_properly = set_color(green,"OK")
                except:
                    pass
                file.close()
            return True
    except Exception as e:
        ran_properly = set_color(red,"NO")
        do_log("<test_install.py> [ERROR] Could not open agrobot log file. " + str(e))
        return False

## Testa se os links simbólicos para o código no python path foram criados corretamente.
def test_sym_link() -> bool:
    global sym_links
    def get_python_version() -> str:
        try:
            python_version = "-1"
            command = "python3 --version > " + current_dir+"python_version.tmp"
            os.system(command)
            with open(current_dir+"python_version.tmp","r") as file:
                for line in file.readlines():
                    line = line.rstrip('\n')
                    line = line.split(" ")
                    line = line[1].split(".")
                    line = line[0] + "." + line[1]
                    python_version = line
                file.close()
            if(os.path.exists(current_dir+"python_version.tmp")):
                os.system("rm " + current_dir+"python_version.tmp")
            return python_version
        except Exception as e:
            do_log("<test_install.py> [ERROR] Could not get python 3 version. "+str(e))
    try:
        paths_to_copy = ["robot_utils","test_robot_utils"]
        python_version = get_python_version()
        sym_links = set_color(green,"OK")
        for path in paths_to_copy:
            if(not os.path.exists("/usr/lib/python" + python_version + "/site-packages/" + path)):
                sym_links = set_color(red,"NO")
                break
        return True
    except Exception as e:
        sym_links = set_color(red,"NO")
        do_log("<test_install.py> [ERROR] Could not check some needed symlinks. "+str(e))
        return False

## Testa se o script do serviço foi instalado corretamente.
def test_service_script() -> None:
    global service_script
    try:
        if(os.path.exists(home+"bin/start_robot.sh")):
            service_script = set_color(green,"OK")
    except Exception as e:
        do_log("<test_install.py> [ERROR] Could not find service script on " + home+"bin/start_robot.sh. "+str(e))

## Testa se o serviço foi instalado corretamente.
def test_service() -> None:
    global service
    try:
        if(os.path.exists("/etc/systemd/system/start_robot.service")):
            service = set_color(green,"OK")
    except Exception as e:
        do_log("<test_install> [ERROR] Could not find service on /etc/systemd/system/start_robot.service. "+str(e))

## Auxiliar para o cálculo do sucesso da instalação.
def calc_installation_aux(variable_to_check: str, log_msg: str) -> int:
    if(variable_to_check == set_color(green,"OK")):
        return 1
    do_log(log_msg)
    return 0

## Calcula a procentagem que deu certo da instalação.
def calc_installation_percent() -> bool:
    count = 0
    total = 7
    # Precisa passar no teste (Entra para o total).
    count += calc_installation_aux(catkin_folder_exists,"<test_install.py> [ERROR] Could not find catkin_ws folder.")
    count += calc_installation_aux(files_copied,"<test_install.py> [ERROR] Could not copy files to catkin_ws/src/agrobot/")
    count += calc_installation_aux(compilation_done,"<test_install.py> [ERROR] Could not compile the src files.")
    count += calc_installation_aux(source_bashrc,"<test_install.py> [ERROR] Could not source .bashrc.")
    count += calc_installation_aux(source_zshrc,"<test_install.py> [WARNING] Could not source .zshrc. It may be caused by missing zsh installation.")
    count += calc_installation_aux(ran_properly,"<test_install.py> [ERROR] Code was not installed or compiled properly. Check the compilation output for more information.")
    count += calc_installation_aux(sym_links,"<test_install.py> [ERROR] Could not find some needed symlinks. Check /usr/lib/python<version>/site-packages/ and look for robot_* symlinks.")
    # Não precisa passar no teste.
    calc_installation_aux(service_script,"<test_install> [WARNING] Could not setup the robot service.")
    calc_installation_aux(service,"<test_install> [WARNING] Could not setup the robot service.")
    return count == total

## Roda os testes relacionados à instalação.
def run_installation_tests():
    global installation_tests
    installation_tests = test_catkin_folder_exists() and test_files_where_copied() and test_compilation()

## Roda os testes relacionados à pós instalação.
def run_post_installation_tests():
    global post_installation_tests
    if(installation_tests):
        post_installation_tests =  test_source_bashrc() and test_source_zshrc() and test_sym_link() and test_run()
    else:
        do_log("<test_install.py> [ERROR] Could not run post_installation_tests(). Installation tests where not executed.")

## Imprime na tela o resultado dos testes.
def tests_results() -> None:
    installattion_result = calc_installation_percent()
    if(installattion_result == True):
        print(set_color(green,"Successfully Installation."))
    else:
        print(set_color(red,"Could not Install properly. Check log files under " + current_dir + "../logs/log.txt for more details."))

## Executa as rotinas de teste.
if __name__ == "__main__":
    try:
        run_installation_tests()
        run_post_installation_tests()
        tests_results()
    except Exception as e:
        do_log("<test_install.py> [ERROR] Could not run test_install.py. " + str(e))