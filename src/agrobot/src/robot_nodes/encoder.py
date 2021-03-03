#!/usr/bin/env python3

"""
@package encoder
Realiza a leitura do encoder, processa os valores e publica no tópico encoder.
"""

import rospy
from std_msgs.msg import String
from robot_utils import testing

# Injeção de dependência.
if(testing.is_test_running()):
    from test_robot_utils import services_dependency as services
else:
    from robot_utils import services

try:
    import RPi.GPIO as GPIO
    gpio_imported: bool = True
except Exception as e:
    gpio_imported: bool = False
    services.do_log_warning('Could not import RPi.GPIO. This will be ignored \
        and no real signal will be read from the encoder. ' + 
        str(e),'encoder.py')

# Pinos de leitura do encoder.
clk_pin = 7
dt_pin = 13

# Configurações GPIO.
if(gpio_imported):
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(clk_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(dt_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Nó do encoder.
rospy.init_node('control_robot', anonymous=True)

# Variáveis de controle de publicação.
pub_encoder: rospy.Publisher = rospy.Publisher("/encoder", String, queue_size=10)

# Variáveis de controle.
last_clk = -1
last_dt = -1
count = 0

def read_encoder():
    """Realiza a leitura do encoder e retorna os dois valores lidos.
       A biblioteca RPi.GPIO precisa ter sido importada.
    """
    return GPIO.input(clk_pin),GPIO.input(dt_pin)

def process_encoder_reading(clk,dt) -> int:
    """Processa o valor lido do encoder e atualiza o valor do count.
       A biblioteca RPi.GPIO precisa ter sido importada.
    """
    global last_clk,last_dt,count
    if(clk != last_clk or dt != last_dt):
            if(clk == dt):
                last_clk = clk
                last_dt = dt
                clk,dt = read_encoder()
                while(last_clk == clk and last_dt == dt):
                    clk,dt = read_encoder()
                if(last_clk == 1):
                    if(clk == 0 and dt == 1):
                        count += 1
                    elif(clk == 1 and dt == 0):
                        count -= 1
                elif(last_clk == 0):
                    if(clk == 1 and dt == 0):
                        count += 1
                    elif(clk == 0 and dt == 1):
                        count -= 1
            else:
                pass
            last_clk = clk
            last_dt = dt

def publish_encoder(value: str) -> None:
    """Publica o valor processado do encoder."""
    pub_encoder.publish(value)

if __name__ == '__main__':
    try:
        while not rospy.is_shutdown():
            if(gpio_imported):
                clk,dt = read_encoder()
                process_encoder_reading(clk,dt)
                publish_encoder(str(count))
    except Exception as e:
        services.do_log_error('Could not run encoder.py. ' + str(e),'encoder.py')
