# TCC2

Versão 0.57

- WifiManager ok, funcionando. Ativa AP e configuração de cadastro em rede WiFi no 192.168.4.1 caso não consiga se conectar a uma rede. Com conecção WiFi na internet ativa, desativa o AP e o StarPortal.

Versão 0.56

- Adicionado WifiManager. Testes sendo realizados.

Versão 0.55

- Task receber_comandos finalizada. Comandos pela UI do Node-Red sendo recebidos.

Versão 0.54

- Wattímetro funcionando, para isso foi necessário criar um objeto do hardwareserial, e indicar as portas utilizadas: RX 25 e TX33. As porta foram indicadas na inicialização begin. Utilizar o Serial2 mesmo modificando o hardwareserial.cpp, colocando outras portas (modificando as nativas, RX 16 e TX 17) não funcionou. Os GPIOs originais (RX 3 e TX1), do UART0, não podem ser utilizados se estiver usando a USB da ESP.

Versão 0.53

- Criação tasks enviar_dados, receber_dados e ler_sensores;
- Testes com tasks, verificando tempo necessário para execução da tarefa e consumo máximo de memória

Versão 0.52

- Função getDeviceCount da biblioteca DallasTemperature incompatível com VisualStudioCode/PlatformIO;
- Adicionado função temperatura. Identifica quantos sensores estão conectados, em qual porta, e atrela o endereço específico de cada sensor na variável Temps[6] específica;
- Modificado objeto JSON

Versão 0.51

- Correção erros interrupções
