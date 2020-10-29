#!/usr/bin/env python3

import rospy,pathlib,os,time
from typing import Final
from std_msgs.msg import String

## Definição do nó.
rospy.init_node("test_priority_decider",anonymous=True)

# Constantes utilizadas para pintar o texto.
blue: Final = '\033[94m'
green: Final = '\033[92m'
red: Final = '\033[91m'
yellow: Final = '\033[93m'
end: Final = '\033[0m'

## Recebe um texto e o retorna com uma cor específica.
def set_color(color: str,text: str) -> str:
    return color + text + end

# Caminhos para as pastas.
current_directory: Final = str(pathlib.Path(__file__).parent.absolute()) + "/"

# Variáveis de controle.
topics_to_test: Final = ['web_server','control_lidar']
number_of_topics_to_test: Final = len(topics_to_test)
successfully_communicated_topics: list = []
priority_test_topics: list = []

# Variáveis para o resultado dos testes.
is_listenning = set_color(red,"NO")
is_publishing = set_color(red,"NO")
priority_worked = set_color(red,"NO")

## Deleta o arquivo temporário utilizado para saber quais tópicos conseguiram comunicar-se.
def delete_tmp_file() -> None:
    try:
        if(os.path.exists(current_directory+"priority.tmp")):
                os.system("rm " + current_directory+"priority.tmp")
    except:
        pass

## Auxiliar para o cálculo do sucesso instalação.
def calc_installation_aux(variable_to_check: str) -> int:
    if(variable_to_check == set_color(green,"OK")):
        return 1
    return 0

## Calcula a procentagem que deu certo da desinstalação.
def all_tests_ok() -> bool:
    count = 0
    total = 3
    count += calc_installation_aux(is_listenning)
    count += calc_installation_aux(is_publishing)
    count += calc_installation_aux(priority_worked)
    return count == total

## Imprime na tela o resultado dos testes.
def tests_results() -> None:
    time.sleep(0.5)
    os.system("clear")
    print("===========Test Result==============")
    print("  Listening --------------------- " + is_listenning)
    print("  Publishing -------------------- " + is_publishing)
    print("  Priority ---------------------- " + priority_worked)
    print("----------------------------------")
    print("  Result ------------------------ ",end="")
    if(all_tests_ok()):
        print(set_color(green,"OK"))
    else:
        print(set_color(red,"NO"))
    print("====================================")
    os.system("pkill ros")

## Verifica se o teste de prioridade ocorreu conforme o esperado.
# A comparação com o valor 2 acontece pelo modo como funciona a prioridade. No teste, é enviado um comando de alta prioridade
# e 'n'+2 comandos de baixa prioridade, logo, após a execução, deverão apenas existiar 2 linhas de recebimento de comandos, 1 para
# o teste de alta prioridade e 1 para o de baixa, pois as n tentativas de executar o de baixa prioridade devem ser ignoradas.
def verify_priority_test() -> None:
    global priority_worked
    if(len(priority_test_topics) == 2):
        priority_worked = set_color(green,"OK")

## Verifica se a comunicação ocorreu de forma esperada.
def verify_communication_tests() -> None:
    global is_listenning,is_publishing
    for topic in topics_to_test:
        topic_ok: bool = False
        for communicated_topic in successfully_communicated_topics:
            if(topic == communicated_topic):
                topic_ok = True
        if(topic_ok == False): # Um ou mais tópicos não funcionaram.
            is_listenning = set_color(red,"NO")
            is_publishing = set_color(red,"NO")
            break
        is_listenning = set_color(green,"OK")
        is_publishing = set_color(green,"OK")

## Finaliza os testes executando as rotinas necessárias.
def post_test_executions() -> None:
    delete_tmp_file()
    tests_results()

## Executa as rotinas de teste.
def run_tests() -> None:
    verify_communication_tests()
    verify_priority_test()
    post_test_executions()

## Callback utilizado para verificar se o test_talk.py, test_listen.py e o test_priority.py funcionaram. Executa as rotinas de teste.
def talk_callback(msg: String) -> None:
    global successfully_communicated_topics,priority_test_topics
    try:
        with open(current_directory+"priority.tmp","r") as file:
            count: int = 0
            for line in file.readlines():
                line = line.rstrip('\n')
                if(count < number_of_topics_to_test): # Testes do test_talk.py
                    successfully_communicated_topics.append(line)
                else: # Testes do test_priority.py
                    priority_test_topics.append(line)
                count = count + 1
            file.close()
        run_tests()
    except:
        pass

## Escuta o sinal do test_talk.py para saber quando todos os comandos foram enviados.
def listen_to_talker() -> None:
    rospy.Subscriber("/test_priority_decider",String,talk_callback)
    rospy.spin()

## Executa as rotinas de teste.
if __name__ == "__main__":
    listen_to_talker()