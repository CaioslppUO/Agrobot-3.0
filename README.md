<p align="center"> 
  <img alt="Arduino-Stable Size" src="https://img.shields.io/github/size/CaioslppUO/src/main/main.ino">
</p>
# Arduino

  * O Arduino recebe informações por UART com o Raspberry(TTL) e as evia para a placa do HoverBoard através de I2C.

## Organização

### - src
  - Código fonte da placa do arduino.

---

# Protocolo utilizado para a comunicação do Raspberry com o Arduino através do UART:

 * É necessário enviar para o Arduino através do UART quatro variáveis, seguindo o padrão: speed;steer;limit;relay;
       * OBS: O formato é exatamente o descrito acima, com as quatro variáveis e quatro ponto e vírgulas separando-as

 * São necessárias quatro variáveis para controlar o robô:
       * Speed.
       * Steer.
       * Limit.
       * Relay.
     
  * Speed aceita valores entre -100 e +100.
     *  0 significa parado.
     *  +100 significa potência total para frente.
     *  -100 significa potência total para trás.
  
   * Steer aceita valores entre -100 e +100.
     *  0 significa ir para frente.
     *  +100 significa ir para a direita.
     *  -100 significa ir para a esquerda.
     
   * Limit aceita valores entre 0 e +100.
     * 0 significa sem potência para as rodas.
     * +100 Significa potência total para as rodas.

  * Relay aceita os valores 0 ou 1.
    * É utilizado para ligar/desligar a placa do hoverboard.
    * 0 significa desligado.
    * 1 significa ligado.
    * para ligar a placa do hoverboard,é necessário enviar 1 e logo em seguida 0 para o relé(Dar curto na placa).
 
 ---
 
 # Recebendo os valores por UART:
  
  * O protocolo implementado é: ABCD;ABCD;ABCD;N; onde ABCD e N são números com os seguintes significados:
  
       * O número A é o sinal, com duas opções possíveis. 1 significa que o número é positivo(+). 0 significa que o númeor é negativo(-). BCD são números que definem a velocidade, a direção e o limite. É necessário que todas as letras(ABCD) sejam preenchidas, mesmo que com o valor 0.
       * N pode ser 0 ou 1.


  * Exêmplo de uso:
       * Para enviar 80% de velocidade, virando 100% para a esquerda, com 50% da potência máxima e relé desligado, é necessário enviar o seguinte comando: 1080;0100;1050;0;