#!/usr/bin/env python3
import rosparam,rospy,json,requests,time
from lidar_utils import services

def convert_bool_to_int(data: bool) -> int:
    if(data):
        return 1
    return 0

def publish_msg(command: dict) -> None:
    try:
        rosparam.set_param("stopTime",str(command["stopTime"]))
        rosparam.set_param("moveTime",str(command["moveTime"]))
        rosparam.set_param("detectDistance",str(command["detectDistance"]))
        rosparam.set_param("correctionFactor",str(command["correctionFactor"]))
        rosparam.set_param("correctionsMovements",str(command["correctionsMovements"]))
        rosparam.set_param("speed",str(command["speed"]))
        rosparam.set_param("steer",str(command["steer"]))
        rosparam.set_param("limit",str(command["limit"]))
        rosparam.set_param("relay_module",str(convert_bool_to_int(command["module"])))
        rosparam.set_param("relay_power","0")
        rosparam.set_param("autoMode",str(convert_bool_to_int(command["autoMode"])))
    except Exception as e:
        services.do_log_error("Could not setting msg to rosparam. " + str(e),"get_lidar_commands.py")

## Classe que gerencia o servidor http.
def Get_lidar_commands(ip: str):
    try:
        data = json.loads(requests.get(ip).content.decode('utf-8'))
        publish_msg(data)
    except Exception as e:
        services.do_log_error("Could not run get_lidar_data.py. " + str(e),"get_lidar_data.py")

## Execução das rotinas do get_robot_commands.
if __name__ == '__main__':
    try:
        ip: str = "http://192.168.1.2:3000/auto_mode_params"
        while not rospy.is_shutdown():
            Get_lidar_commands(ip)
            time.sleep(1);
    except rospy.ROSInterruptException:
        services.do_log_warning("The roscore was interrupted.","get_lidar_commands.py")

