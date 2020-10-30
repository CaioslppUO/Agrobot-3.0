#1/usr/bin/env python3

import rosservice

available_services = ['/log_error','/log_info','/log_warning','/relay']

## Espera até todos os serviços utilizados estejam disponível.
def wait_for_services_availability() -> bool:
    attempt_limit: int = 1000
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
            if(index > attempt_limit):
                limit_reached = True
                break
        if(limit_reached == True):
            return False
    return True