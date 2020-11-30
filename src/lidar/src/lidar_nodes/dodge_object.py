#!/usr/bin/env python3

import time,rospy,rosparam
from std_msgs.msg import String
from lidar_utils import services
from lidar.msg import parameters,complete_command

rospy.init_node('control_lidar', anonymous=True)
const_pub_control_command = rospy.Publisher("control_lidar", complete_command,queue_size=10)

class Control_lidar():
    def __init__(self):
        
        # É utilizada para controlar a funcionalidade de andar por 'x' segundos e parar por 'y' segundos.
        self.walk = True
        ## Variável que controla a quantidade de movimentos de correção que serão aplicados.
        self.tick = 0

        self.parameters = parameters()
        self.complete_command = complete_command()
        
        self.correction_dir = ""
        ## Variável que contém a informação se existe algum 'objeto' próximo ao sensor da esquerda.
        self.left_sensor = None
        ## Variável que contém a informação se existe algum 'objeto' próximo ao sensor da direita.
        self.right_sensor = None
        ## Variável que contém a informação se existe algum 'objeto' próximo ao sensor do meio.
        self.center_sensor = None

    ## Método seta o movimento necessário para corrigir o robô(se for necessário).
    def define_correction_movement(self):
        if(self.left_sensor == "busy"):
            self.tick = self.parameters.correctionsMovements
            self.correction_dir = "right"
        elif(self.right_sensor == "busy"):
            self.tick = self.parameters.correctionsMovements
            self.correction_dir = "left"

    ## Método que avalia os valores lidos pelos sensores e decide se precisa corrigir ou não o movimento, alterando o valor da direção.
    # Também altera a variável 'tick', indicando que um movimento de correção foi efetuado.
    def set_steer(self):
        if(self.tick == 1):
            self.tick = self.parameters.correctionsMovements
        elif (self.correction_dir == "right"):
            self.complete_command.move.move.y = self.parameters.move.move.move.y - self.parameters.correctionFactor
        else:
            self.complete_command.move.move.y = self.parameters.move.move.move.y + self.parameters.correctionFactor
        self.tick = self.tick - 1

    ## Método que verifica o valor do tick e decide se é ou não necessário realizar a leitura dos sensores e alterar o valor da direção.
    def check_tick(self):
        if(self.tick == 0):
            self.define_correction_movement()
        else:
            self.set_steer()

    ## Método que checa se o modo automático está ativo ou não, liberando ou bloqueando a movimentação automática.
    def check_move_permission(self):
        if (self.parameters.correctionsMovements == 0 and
            self.parameters.correctionDistance == 0 and
            self.move.move.linear.x == 0 and
            self.move.move.linear.y == 0 and
            self.move.limit.speed_limit == 0):
            return False
        return True
        

    ## Método callback para a classe que controla o tempo de movimento e parada do robô.
    # Define a variável walk, que diz para o robô quando ele pode andar e quando deve ficar parado.
    def callback_walk(self,msg:String):
        info:str = str(msg.data)
        if(info == 'walk'):
            self.walk = True
        else:
            self.walk = False

    ## Método callback para as mensagens recebidas pelo app, no modo de controle automático.
    # Recebe e separa as variáveis passadas pelo app.
    def read_rosparam(self):
        self.parameters.detectDistance = float(rosparam.get_param("detectDistance"))
        self.parameters.correctionFactor = int(rosparam.get_param("correctionFactor"))
        self.parameters.correctionsMovements = int(rosparam.get_param("correctionsMovements"))
        self.parameters.move.relay.signal_relay_module = int(rosparam.get_param("relay_module"))
        self.parameters.move.move.linear.x = int(rosparam.get_param("speed"))
        self.parameters.move.move.linear.y = int(rosparam.get_param("steer"))
        self.parameters.move.limit.speed_limit = int(rosparam.get_param("limit"))
        

    ## Método que verifica se existe algum objeto 'próximo' ao sensor central do robô. Caso exista para o robô e desliga a lâmpada UV.
    def check_foward(self):
        if(self.center_sensor == 'free'):
            self.complete_command.move.linear.x = self.parameters.move.move.linear.x
            self.complete_command.move.linear.y = self.parameters.move.move.linear.y
            self.complete_command.relay.signal_relay_module = self.parameters.move.relay.signal_relay_module
            self.check_tick()
        else:
            self.complete_command.move.linear.x = 0
            self.complete_command.move.linear.y = 0
            self.complete_command.relay.signal_relay_module = 0
            

    ## Método que controla o movimento do robô.
    def move(self):
        if(self.check_move_permission()):
            if(self.walk):
                self.check_foward()
                const_pub_control_command.publish(self.complete_command)
            else:
                self.complete_command.move.x = 0
                self.complete_command.move.limit.speed_limit = 0
                const_pub_control_command.publish(self.complete_command)
                
    ## Método callback para a leitura do lidar.
    # O envio de comandos ao robô é baseado na velocidade de leitura do lidar.
    def callback_lidar(self, msg):
        self.read_rosparam()
        info = str(msg.data)
        if(services.wait_for_topic_availability("walk")):
            rospy.Subscriber('walk', String, self.callback_walk)  # Atualiza o valor da variável 'walk'.
        else:
            services.do_log_warning("Error reading topic walk.","dodge_object.py")
        self.left_sensor,self.center_sensor,self.right_sensor = info.split('$')
        self.move() # Chama o método que controla o robô.

if __name__ == "__main__":
    try:
        if(services.wait_for_topic_availability("lidar_values")):
            control_lidar = Control_lidar()
            rospy.Subscriber('lidar_values', String, control_lidar.callback_lidar)
            rospy.spin()
        else:
            services.do_log_warning("Error reading topic lidar_values.","dodge_object.py")
    except Exception as e:
        services.do_log_error("Error when giving object dodge.\n"+str(e),"dodge_object.py")
