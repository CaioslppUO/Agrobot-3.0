#1/usr/bin/env python3

import rosparam

## Injeção de dependências.
def wait_for_param_availability(needed_params: list) -> bool:
    needed_params = ['testing']
    attempt_limit: int = 1000
    limit_reached: bool = False
    index: int = 0
    for param in needed_params:
        try:
            if(param != ''):
                index = 0
                waiting = rosparam.get_param(param)
                while(waiting == None):
                    waiting = rosparam.get_param(param)
                    index = index + 1
                    if(index > attempt_limit):
                        limit_reached = True
                        break
                if(limit_reached == True):
                    return False
        except:
            raise Exception("The parameter (needed_params) is not a string list.")
    return True