# ROS

  * Controle do robô a partir do raspberry pi.

## Organização

### - img
  - Imagens de documentação.

### - install
  - Instalador do código.

### - src
  - Código fonte do raspberry.

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

# Versão: 7.5