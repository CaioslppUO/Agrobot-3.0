#!/usr/bin/env python3

import os

## Envia o comando de controle para o arduino.
def send_command(speed: int, steer: int, limit: int, power_signal: int) -> None:
    os.system("rosservice call /control_robot '" + str(speed) + "' '" + str(steer) + "' '" + str(limit) + "' '" + str(power_signal) + "'")