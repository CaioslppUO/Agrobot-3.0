#!/usr/bin/env python3

import rospy, rosparam
from lidar_utils import services
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

## Ponto central(Frente do robô) do vetor de pontos.
const_center_point = 0
## Ângulo ou número de pontos a serem pegos para o cálculo no vetor de pontos(Em cada direção).
const_angle_range = 16

rospy.init_node('lidar_values', anonymous=True)
pub_processed_lidar_data = rospy.Publisher("lidar_values", String,queue_size=10)

## Função que monta 3 vetores de pontos lidos do vetor de pontos do lidar.
def select_points(vet:list,range:int,central_point:int):#->tuple:list
    i:int = 0
    right_vet:list = []
    left_vet:list = []
    center_vet:list = []
    center_vet.append(vet[central_point])
    while(i < range/2):
        center_vet.append(vet[central_point+i])
        center_vet.append(vet[central_point-i])
        i=i+1
    i=0
    right_vet.append(vet[central_point])
    left_vet.append(vet[central_point])
    while(i < range):
        right_vet.append(vet[central_point+i])
        left_vet.append(vet[central_point-i])
        i = i+1
    return right_vet,center_vet,left_vet

## Callback para o param_server. Lê e atualiza a distância de detecção de colisão.
def read_rosparam() -> float:
    try:
        detect_collision_distance = float(rosparam.get_param("detectDistance"))
    except:
        detect_collision_distance = 1.5
        services.do_log_warning("error reading the rosparam detectColision","process_lidar.py")
    return detect_collision_distance


## Callback do topico do scan do lidar. Processa a mensagem recebida e publica o resultado no tópico lidar.
def callback_lidar_scan(msg):
    r_vet, c_vet, l_vet = select_points(msg.ranges, const_angle_range, const_center_point)
    pub_processed_lidar_data.publish(str(
        get_distance_from_object(l_vet, read_rosparam()) + "$" +
        get_distance_from_object(c_vet, read_rosparam()) + "$" +
        get_distance_from_object(r_vet,read_rosparam())))

## Função que retorna se existe um objeto 'próximo' a um dos sensores do robô, baseado na distância de colisão.
def get_distance_from_object(vet:list ,detect_collision_distance:float) -> str:
    for test_value in vet:
        if(not isinstance(test_value, str)):
            if(test_value <= detect_collision_distance):
                return "busy" 
    return "free"

if __name__ == "__main__":
    try:
        if(services.wait_for_topic_availability("/scan")):
            rospy.Subscriber('/scan', LaserScan, callback_lidar_scan)
            rospy.spin()
        else:
            services.do_log_warning("Error reading topic scan."+ str(e),"process_lidar.py")
    except Exception as e:
        services.do_log_error("Error reading from topic scan, topico process_lidar is over."+ str(e),"process_lidar.py")
