#1/usr/bin/env python3

import rosservice,os,rosparam
from typing import Any, Final

## Serviços disponíveis para serem utilizados.
# Utilizado para esperar até que todos estejam disponível.
available_services: Final = ['/log_error','/log_info','/log_warning','/relay','/control_robot']

## Variável de controle de tentativa.
services_attempt_limit: int = 10000
parameters_attempt_limit: int = 2000

## Espera até todos os serviços utilizados estejam disponível.
def wait_for_services_availability() -> bool:
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

## Envia o sinal para o relé.
def send_signal_to_relay(signal: int) -> None:
    os.system("rosservice call /relay '" + str(signal)+"'")

## Faz log de erro.
def do_log_error(msg: str, file: str):
    os.system("rosservice call /log_error '" + msg + "' '" + file + "'")

## Faz log de info.
def do_log_info(msg: str, file: str):
    os.system("rosservice call /log_info '" + msg + "' '" + file + "'")

## Faz log de warning.
def do_log_warning(msg: str, file: str):
    os.system("rosservice call /log_warning '" + msg + "' '" + file + "'")

## Envia o comando de controle para o arduino.
def send_command_to_robot(speed: int, steer: int, limit: int, power_signal: int) -> None:
    os.system("rosservice call /control_robot '{speed: " + str(speed) + ", steer: " + str(steer) + ", limit: " + str(limit) + ", power_signal: " + str(power_signal) + "}'")

## Envia o comando de controle para o arduino.
def send_command_to_mini_robot(speed: int, steer: int, limit: int) -> None:
    os.system("rosservice call /control_mini_robot '{speed: " + str(speed) + ", steer: " + str(steer) + ", limit: " + str(limit) + "}'")

## Pega um parâmetro do rosparam, caso ele não exista, retorna -1 (int).
def get_parameter(parameter_name: str) -> Any:
    count: int = 0
    parameter: str = ""
    while(parameter == "" and count < parameters_attempt_limit):
        try:
            parameter = rosparam.get_param(parameter_name)
        except:
            pass
        count = count + 1
    if(parameter != ""):
        return parameter
    return -1