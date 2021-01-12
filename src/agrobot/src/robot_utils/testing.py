#!/usr/bin/env python3

"""
@package testing
Serviço para verificar se o projeto está ou não rodando em modo de teste.
"""

import rosparam

## Verifica se o módulo de testes está ou não rodando.
def is_test_running() -> bool:
    testing: str = ""
    limit_attempts: int = 100
    count:int = 0
    is_running: bool = False
    while(testing == "" and count < limit_attempts):
        try:
            testing = rosparam.get_param("testing")
            is_running = True
        except:
            pass
        count = count + 1
    return is_running