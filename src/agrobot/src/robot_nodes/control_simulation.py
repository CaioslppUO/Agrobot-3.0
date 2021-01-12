#!/usr/bin/env python3

## Controla a comunicação com o topico de controle do robô utilizado pela simulação.
import rospy
from geometry_msgs.msg import Twist
from agrobot.msg import complete_command

# Nó que publicara os comandos.
rospy.init_node("control_simulation",anonymous=True)

# Nó no qual serão publicados os comandos.
pub = rospy.Publisher("/cmd_vel",Twist,queue_size=10)

# Recebe comandos do topico control_robot e os envia para o topico da simulação.
def listen_callback(data: complete_command):
    msg = Twist()
    msg.linear.x = data.move.linear.x/100
    msg.angular.z = data.move.angular.z/100
    pub.publish(msg)

# Escuta o topico control_robot.
def listen():
    rospy.Subscriber("/control_robot",complete_command,listen_callback)

if __name__ == "__main__":
    listen()
    rospy.spin()