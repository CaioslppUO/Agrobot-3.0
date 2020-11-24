#!/usr/bin/env python3

import time,serial,rospy
from agrobot.msg import complete_command
from robot_utils import testing

if(testing.is_test_running()):
    from test_robot_utils import services_dependency as services
else:
    from robot_utils import services

# Nó do controle do robô.
rospy.init_node("control_robot", anonymous=True)

# Variável de controle e definição.
uart_names: list = ['ttyUSB_CONVERSOR-0','ttyUSB_CONVERSOR-1','ttyUSB_CONVERSOR-2','ttyUSB_CONVERSOR-3']
used_uarts: dict = {'0':None,'1':None,'2':None,'3':None}

## Pega a e retorna e porta UART.
def get_uart_port(port_name: str) -> serial.Serial:
    uart = None
    try:
        uart = serial.Serial(
                port='/dev/'+port_name,
                baudrate = 9600,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                timeout=1
        )
    except:
        pass
    return uart

## Define quais uarts serão utilizados.
def set_uarts() -> None:
    global used_uarts
    count: int = 0
    for uart in used_uarts:
        used_uarts[uart] = get_uart_port(uart_names[count])
        count = count + 1

## Converte os valores de controle para valores que o arduino entenda.
# Padrão: SINAL_NÚMERO: o Sinal ocupa 1 posição, o número ocupa 3 posições em uma string.
def converts_to_arduino_pattern(value: int) -> str:
    if(value >= 0):
        std_value: str = '1'
    else:
        std_value: str = '0'
    if(value < 10 and value > -10):
        std_value += '00'
    elif(value < 100 and value > -100):
        std_value += '0'
    std_value += str(abs(value))
    return std_value

## Envia a string pelo UART.
def write_to_uart(command: str) -> None:
    try:
        for uart in used_uarts:
            if(used_uarts[uart] != None):
                used_uarts[uart].write(str.encode(command))
        time.sleep(0.02)
    except Exception as e:
        services.do_log_error("Could not send command through UART. " + str(e),"control_robot.py")

## Envia o comando de controle para o robô.
def control_robot_callback(data: complete_command) -> str:
    speed: int = int(data.move.linear.x)
    steer: int = - int(data.move.angular.z) # O negativo desconverte do padrão.
    limit: int = int(data.limit.speed_limit)
    power_signal: int = int(data.relay.signal_relay_power)
    command: str = str(speed)+';'+str(steer)+';'+str(limit)+';'+str(power_signal)+";"
    write_to_uart(command)
    return command

## Escuta o chamado dos serviços.
def listen_control_robot() -> None:
    rospy.Subscriber("control_robot",complete_command, control_robot_callback)

## Executa as rotinas do serviço.
if __name__ == "__main__":
    set_uarts()
    listen_control_robot()
    rospy.spin()