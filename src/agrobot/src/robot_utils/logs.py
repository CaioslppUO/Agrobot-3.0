import rosservice

## Faz log de erro.
def do_log_error(msg: str, file: str):
    rosservice.call_service("/log_error",[msg,file])

## Faz log de info.
def do_log_info(msg: str, file: str):
    rosservice.call_service("/log_info",[msg,file])

## Faz log de warning.
def do_log_warning(msg: str, file: str):
    rosservice.call_service("/log_warning",[msg,file])