#1/usr/bin/env python3

import rostopic

available_topics = ['']

## Espera até todos os nodes utilizados estejam disponível.
def wait_for_nodes_availability() -> bool:
    attempt_limit: int = 1000
    limit_reached: bool = False
    index: int = 0
    for topic in available_topics:
        if(topic != ''):
            index = 0
            waiting = None
            while(waiting == None):
                try:
                    waiting = rostopic.get_topic_type(topic)
                except:
                    pass
                index = index + 1
                if(index > attempt_limit):
                    limit_reached = True
                    break
            if(limit_reached == True):
                return False
    return True