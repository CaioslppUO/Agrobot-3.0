#!/usr/bin/env python3

import os

## Envia o sinal para o relé.
def send_signal(signal: int) -> None:
    os.system("rosservice call /relay '" + str(signal)+"'")