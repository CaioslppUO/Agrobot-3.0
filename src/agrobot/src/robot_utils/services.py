#1/usr/bin/env python3

import rosservice,os,rosparam
from typing import Any, Final
from agrobot.msg import complete_command

## Serviços disponíveis para serem utilizados.
# Utilizado para esperar até que todos estejam disponível.
available_services: Final = ['/log_error','/log_info','/log_warning']

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

## Faz log de erro.
def do_log_error(msg: str, file: str):
    os.system("rosservice call /log_error '" + msg + "' '" + file + "'")

## Faz log de info.
def do_log_info(msg: str, file: str):
    os.system("rosservice call /log_info '" + msg + "' '" + file + "'")

## Faz log de warning.
def do_log_warning(msg: str, file: str):
    os.system("rosservice call /log_warning '" + msg + "' '" + file + "'")

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

## Faz a verificação e corrige os valores do comando de controle do robô, caso necessário.
def check_complete_control_command(command: complete_command) -> None:
    if(command.move.linear.x < -100 or command.move.linear.x > 100):
        command.move.linear.x = 0
        do_log_error("Speed value is under -100 or above 100. Sending 0 instead.","services.py")
    if(command.move.linear.y < -100 or command.move.linear.y > 100):
        command.move.linear.y = 0
        do_log_error("Steer value is under -100 or above 100. Sending 0 instead.","services.py")
    if(command.limit.speed_limit < 0 or command.limit.speed_limit > 100):
        command.limit.speed_limit = 0
        do_log_error("Limit value is under 0 or above 100. Sending 0 instead.","services.py")
    if(command.relay.signal_relay_module != 0 and command.relay.signal_relay_module != 1):
        command.relay.signal_relay_module = 0
        do_log_error("Module relay value is not 1 or 0. Sending 0 instead.","services.py")
    if(command.relay.signal_relay_power != 0 and command.relay.signal_relay_power != 1):
        command.relay.signal_relay_power = 0
        do_log_error("Power relay value is not 1 or 0. Sending 0 instead.","services.py")