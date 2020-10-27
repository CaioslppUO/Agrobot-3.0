#1/usr/bin/env python3

import rosservice

available_services = ['/log_error','/log_info','/log_warning']

## Espera até todos os serviços utilizados estejam disponível.
## O nome de cada serviço deve ser passado utilizando a '\' na frente. Ex: 'log_error'
def wait_for_services_availability() -> bool:
    attempt_limit: int = 1000
    limit_reached: bool = False
    index: int = 0
    for service in available_services:
        index = 0
        waiting = rosservice.get_service_type(service)
        while(waiting == None):
            waiting = rosservice.get_service_type(service)
            index = index + 1
            if(index > attempt_limit):
                limit_reached = True
                break
        if(limit_reached == True):
            return False
    return True