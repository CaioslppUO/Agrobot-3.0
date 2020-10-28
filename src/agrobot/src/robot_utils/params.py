#1/usr/bin/env python3

import rosparam

## Espera até todos os parâmetros utilizados estejam disponível.
def wait_for_param_availability(needed_params: list) -> bool:
    attempt_limit: int = 1000
    limit_reached: bool = False
    index: int = 0
    for param in needed_params:
        try:
            if(param != ''):
                index = 0
                waiting = None
                while(waiting == None):
                    try:
                        waiting = rosparam.get_param(param)
                    except:
                        pass
                    index = index + 1
                    if(index > attempt_limit):
                        limit_reached = True
                        break
                if(limit_reached == True):
                    return False
        except:
            raise Exception("The parameter (needed_params) is not a string list.")
    return True