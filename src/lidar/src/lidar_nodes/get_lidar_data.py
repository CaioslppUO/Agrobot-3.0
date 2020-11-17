import rosparam,rospy,json,requests,pathlib,time
from robot_utils import services
from lidar.msg import parameters

def convert_bool_to_int(data: bool) -> int:
    if(data):
        return 1
    return 0

## Separa e configura o tipo de cada comando recebido.
def setup_command(command) -> parameters:
    parameters_lidar = parameters()
    try:
        parameters_lidar.stopTime = int(command["stopTime"])
        parameters_lidar.moveTime = int(command["moveTime"])
        parameters_lidar.detectDistance = float(command["detectDistance"])
        parameters_lidar.correctionFactor = int(command["correctionFactor"])
        parameters_lidar.correctionMovements = int(command["correctionMovements"])
        parameters_lidar.complete_command.move.linear.x = int(command["speed"])
        parameters_lidar.complete_command.move.linear.y = int(command["steer"])
        parameters_lidar.complete_command.limit.speed_limit = int(command["limit"])
        parameters_lidar.complete_command.relay.signal_relay_module = convert_bool_to_int(command["module"])
        parameters_lidar.complete_command.relay.signal_relay_power = 1
    except Exception as e:
        services.do_log_error("Could not load json. " + str(e), "get_robot_commands.py")
    services.check_complete_control_command(parameters_lidar.complete_command)
    return parameters_lidar

## Publica a mensagem no tópico /get_robot_commands.
def publish_msg(msg: parameters) -> None:
    try:
        rosparam.set_param("stopTime",str(msg.stopTime))
        rosparam.set_param("moveTime",str(msg.moveTime))
        rosparam.set_param("detectDistance",str(msg.detectDistance))
        rosparam.set_param("correctionFactor",str(msg.correctionFactor))
        rosparam.set_param("correctionMovements",str(msg.correctionMovements))
        rosparam.set_param("speed",msg.complete_command.move.linear.x)
        rosparam.set_param("steer",msg.complete_command.move.linear.y)
        rosparam.set_param("limit",msg.complete_command.limit.speed_limit)
        rosparam.set_param("relay_module",msg.complete_command.relay.signal_relay_module)
        rosparam.set_param("relay_power",msg.complete_command.relay.signal_relay_power)
    except Exception as e:
        services.do_log_error("Could not setting msg to rosparam " + str(e),"get_lidar_commands.py")

## Classe que gerencia o servidor http.
def Get_lidar_commands(ip: str):
    try:
        data = json.loads(requests.get(ip).content.decode('utf-8'))
        publish_msg(setup_command(data))
    except Exception as e:
        services.do_log_error("Could not run get_robot_commands.py. " + str(e),"get_robot_commands.py")

## Execução das rotinas do get_robot_commands.
if __name__ == '__main__':
    try:
        ip: str = "http://192.168.1.2:3000/auto_mode_params"
        while not rospy.is_shutdown():
            Get_lidar_commands(ip)
            time.sleep(1);
    except rospy.ROSInterruptException:
        services.do_log_warning("The roscore was interrupted.","get_lidar_commands.py")

