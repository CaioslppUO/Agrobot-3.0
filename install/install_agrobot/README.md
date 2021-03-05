# Instalação

1) Caso já tenha instalado anteriormente ou o código ainda esteja rodando, execute o seguinte comando para finalizar o serviço do robô:

    > sudo systemctl stop start_robot.service
    
2) Execute o script install_dependencies.sh. Caso ele já tenha sido executado, uma mensagem de erro irá aparecer na tela, apenas ignore.

    > ./install_dependencies.sh        

3) Execute o script install.py.

    > ./install.py

4) Caso queira rodar o processo, execute:

    > sudo systemclt start start_robot.service