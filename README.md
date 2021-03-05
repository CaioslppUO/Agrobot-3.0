# ROS

  * Controle do robô a partir do raspberry pi.

## Organização

### - img
  - Imagens de documentação.

### - install
  - Instalador do código.

### - src
  - Código fonte do raspberry.

### - doc
  - Documentação gerada com auxílio do Doxygen.

---

# Explicação do funcionamento

### Partes

  - O código está dividido nas seguintes partes:

    * Recepção de comandos.
    * Tratamento dos comandos para o manipulação interna.
    * Separação dos diferentes comandos e envio ao destino correto.

## Estrutura

  - O código segue a seguinte estrutura

    * Receive Command -> Command Priority Decider -> Command Standardize -> Command Assembler -> Send Command.

## Instalação

  - Siga os seguintes passos para instalar o código:

    1) Baixe o repositório e altere para a branch ros-unstable.
    2) Entre no diretório install.
    3) Execute o script install_depedencies.sh.
    4) Abra um novo terminal.
    5) Execute o script install.py.
    6) Abra um novo terminal e rode o projeto.

## Execução

  - Após a instalação do projeto, siga os seguintes passos:

    1) Abra um terminal.
    2) Execute o comando: roslaunch agrobot <execution_mode>
    
    OBS: substituir o <execution_mode> pelo modo de execução desejado.

# Versão: 15.2