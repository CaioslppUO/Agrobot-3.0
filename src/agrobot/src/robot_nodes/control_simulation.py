#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from agrobot.msg import complete_command

rospy.init_node("control_simulation",anonymous=True)

pub = rospy.Publisher("/cmd_vel",Twist,queue_size=10)

def listen_callback(data: complete_command):
    msg = Twist()
    msg.linear.x = data.move.linear.x/100
    msg.angular.z = -data.move.linear.y/100
    pub.publish(msg)

def listen():
    rospy.Subscriber("/control_robot",complete_command,listen_callback)

if __name__ == "__main__":
    listen()
    rospy.spin()