#!/usr/bin/env python3

import rospy,time,rosparam
from std_msgs.msg import String
from robot_utils import services

const_pub_control_command = rospy.Publisher("walk", String, queue_size=10)
rospy.init_node('walk', anonymous=True)

def read_rosparam():
    try:
        return int(rosparam.get_param("moveTime")),int(rosparam.get_param("stopTime"))
    except:
        services.do_log_warning("Cold not read from rosparam.","movementTiming.py")
        return -1,-1

if __name__ == "__main__":
    try:
        stop_time:int = -1
        walk_time:int = -1
        while not rospy.is_shutdown():
            walk_time, stop_time = read_rosparam()
            if(walk_time != -1 and stop_time != -1):
              if(walk_time != 0 and stop_time != 0):
                const_pub_control_command.publish("stop")
                time.sleep(stop_time)
                const_pub_control_command.publish("walk")
                time.sleep(walk_time)
              else:
                const_pub_control_command.publish("walk")
    except Exception as e:
        services.do_log_error("Cold not run movement Timing","movementTiming.py")
