#include "Includes.h" // Todos os includes necessários para o projeto
// Variáveis globais utilizadas estão no "Variaveis.h"
////////////////////////////////////////////////////////////////////////////////////////////////////////

Interrupcoes interrupt(tempo_db_Pinos_temp);

// Conectar em uma rede WiFi préviamente definida
void setup_wifi() {

  delay(10);
  // iniciar conexão wifi
  WiFi.mode(WIFI_STA);
  //WiFi.config(local_ip,gateway, subnet);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
}

// Reconecta no broker MQTT.
void reconnect() {

  unsigned int tentativas_reconnect_MQTT = 0;
  // Loop until we're reconnected
  while (!client.connected() && tentativas_reconnect_MQTT < 5) {
    if (client.connect("projeto_1")) {

      client.subscribe("comandosTCC"); // Dar subscribe no tópico que envia os comandos do NodeRed para cá.
      return;
    } else {
      tentativas_reconnect_MQTT++;
    }  
    delay(1000); // Delay de 1 segundo para cada tentativa de reconectar no broker MQTT.  
  }

  if (tentativas_reconnect_MQTT == 4){
    Serial.println("Tentativa de se conectar no tópico do MQTT falhou, adicionar mudança de variável aqui futuramente");
  }
}

// Receber mensagens do tópico MQTT assinado. Essa função é enviada como parâmetro na função client.loop, da biblioteca do MQTT. Lá ele carrega os dados no payload caso haja msg nova.
void callback(char* topic, byte* payload, unsigned int length) {
  
  Serial.println();
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    comandos_recebidos[i] = payload[i];
    Serial.print((char)payload[i]);
  }
  Serial.println();
  // Só deserializa se chegar nova mensagem.
  deserializeJson(JSON_recebe_comandos, comandos_recebidos); // Deserializa a msg recebida e joga no objeto JSON.

  // TED: Tempo Envio Dados
  tempo_task_enviar_dados = JSON_recebe_comandos["TED"];
  tempo_task_enviar_dados = tempo_task_enviar_dados * 1000;
  tipo_gas = JSON_recebe_comandos["Gas"];
  Serial.println(tempo_task_enviar_dados);
  Serial.println(tipo_gas);

}

void indice_bateria(){

  for (int i = 0; i < 100; i++){ // Faz 100 leituras da porta e soma.
    percentual_bateria = analogRead(porta_bateria) + percentual_bateria;
  }
  percentual_bateria = percentual_bateria / 100; // Tira a média das 100 leituras

  if (bateriaMax >= percentual_bateria) bateriaMax = percentual_bateria; // Salva o maior valor encontrado na leitura da carga da bateria.
  if (bateriaMin <= percentual_bateria) bateriaMin = percentual_bateria; // Salva o menor valor encontrado na leitura da carga da bateria.

  JSON_envia_dados["bateriaMax"] = bateriaMax;
  JSON_envia_dados["bateriaMin"] = bateriaMin;

  int indiceBateria = map(percentual_bateria, 2800, 4094, 0, 100); // Mapeia os valores lidos para o intervalo de 0 a 100, para ficar em percentual.

  if (indiceBateria <= 0) indiceBateria = 0; // Caso ocorra alguma leitura menor que 2800 antes do map, fica valor negativo, estabelece o valor 0 para que não ocorra isso.
  else if (indiceBateria >= 100) indiceBateria = 100; // Caso o valor lido tenha ficado maior que 4094, para não dar valor maior que 100, estabelece esse limite. 
  JSON_envia_dados["Bat"] = indiceBateria;

}

void temperatura(){
    
    interrupt.atualizar_estado_portas(); // Rotina para atualizar o estado das portas dos conectores de temperatura.
    interrupt.retorna_vetor(sensores_conectados); // Verifica quais portas estão com algo conectado ou não. True para conectado, false para desconectado.

    sensortemp.requestTemperatures(); // Adiciona pelo menos meio segundo de tempo de processamento, mas varia muito.

    for (int n = 0; n <= 5; n++){
      if (sensores_conectados[n]) { // Se no n específico, tiver um sensor conectado, entrará na condição para pegar o endereço do sensor e medir a temperatura.
        sensortemp.getAddress(Thermometer, n); // Pega o endereço de cada sensor conectado.
        Temps[n] = sensortemp.getTempC(Thermometer); // Pega a temperatura já convertida para Celsius e grava na posição do sensor no pino no vetor correspondente.
        JSON_envia_dados["T"][n] = Temps[n]; // Grava o valor da temperatura no vetor do JSON da temperatura.
      }

      if (!sensores_conectados[n]) { // Se no n específico, não tiver um sensor conectado, entra na condição. 
        Temps[n] = -127; // Gravará o valor -127. Esse valor é padrão na biblioteca da Dallas para sensor não conectado.
        JSON_envia_dados["T"][n] = Temps[n]; // Grava o valor da temperatura no vetor do JSON da temperatura.
      }
    }
}

void pressao(){

  P1 = 0;
  int i = 0;
  while (i < 50) {
    P1 = P1 + analogRead(porta_P1);
    i++;
  }
  P1 = P1 / 50; // Tira a média das 50 leituras.
  JSON_envia_dados["P1"] = P1;
  P1 = (P1 * 0.1875) / 1000; // Obter tensão.
  P1 = (P1 * 30 - 19.8) / 2.64; // fórmula obtida fazendo uma matriz com 0 bar a 30 bar e 0,66V a 3.3V. Valor do 21.8 na formula é 19.8, foi acrescentado 1 para correção das variações e ficar o mesmo valor do manômetro

  P2 = 0;
  i = 0;
  while (i < 50) {
    P2 = P2 + analogRead(porta_P2);;
    i++;
  }
  P2 = P2 / 50; // Tira a média das 50 leituras.
  JSON_envia_dados["P2"] = P2;
  P2 = (P2 * 0.1875) / 1000; // Obter tensão.
  P2 = (P2 * 10 - 6.6) / 2.64; // Fórmula obtida fazendo uma matriz com 0 bar a 10 bar e 0,66V a 3.3V.

  //pressaoAlta = pressaoAlta * 14.504; //convertendo pra psi

}

void wattimetro(){

  V = pzem.voltage();
  I = pzem.current();
  W = pzem.power();
  Wh = pzem.energy();
  Freq = pzem.frequency();
  FP = pzem.pf();
  
  if (isnan(W)) SerialController.begin(9600,SERIAL_8N1,25,33);
  
  // Condição caso o wattímetro esteja desligado/problema de conexão, substituí o nan (not a number) por -1.
  if (isnan(W)) W = -1;
  if (isnan(V)) V = -1;
  if (isnan(I)) I = -1;
  if (isnan(FP)) FP = -1;
  if (isnan(Wh)) Wh = -1;
  if (isnan(Freq)) Freq = -1;

  JSON_envia_dados["W"] = pzem.power(); // potência Watts
  JSON_envia_dados["V"] = pzem.voltage(); // tensão
  JSON_envia_dados["I"] = pzem.current(); // corrente
  JSON_envia_dados["FP"] = pzem.pf(); // fator potência
  JSON_envia_dados["Wh"] = pzem.energy(); // watt hora
  JSON_envia_dados["freq"] = pzem.frequency(); // frequência

}

void enviar_dados(void *pvParameters){

  UBaseType_t uxHighWaterMark; // Variável para identificar o consumo máximo de memória de uma task.

  for(;;){
    float tempo_ant = millis(); 

    // Se perder a conexão WiFi, tenta reconectar.
    while (WiFi.status() != WL_CONNECTED) {}

    // Se perder a conexão com o broker MQTT, tenta reconectar.
    if (!client.connected()) reconnect();

    client.publish("subscriberTCC", dados_envio); // Publica o JSON no tópico do broker MQTT.

    /* Obtém o High Water Mark da task atual.
   Lembre-se: tal informação é obtida em words! */
    uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );

    Serial.print("Tempo de enviar dados dentro da função enviar_dados: ");
    Serial.println(tempo_task_enviar_dados);

    float tempo_dep = millis();
    tempo_ant = tempo_dep - tempo_ant;

    vTaskDelay(tempo_task_enviar_dados/portTICK_PERIOD_MS);
  }

}

void receber_dados(void *pvParameters){

  UBaseType_t uxHighWaterMark; // Variável para identificar o consumo máximo de memória de uma task

  for(;;){
    float tempo_ant = millis();

    if (!client.connected()) reconnect(); // Caso não esteja conectado no broker MQTT, tenta reconectar.
      
    client.loop(); // Função para verificar se existe mensagem nova no subscriber do MQTT e já grava os dados na variável comandos_recebidos.


    uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
    float tempo_dep = millis();   
    tempo_ant = tempo_dep - tempo_ant;

    vTaskDelay(tempo_task_receber_dados/portTICK_PERIOD_MS); 
    
  }
}

void ler_sensores(void *pvParameters){

  UBaseType_t uxHighWaterMark; // Variável para identificar o consumo máximo de memória de uma task

  for(;;){
    float tempo_ant = millis();

    indice_bateria();
    temperatura();
    pressao();
    wattimetro();

    serializeJson(JSON_envia_dados, dados_envio);

    /* Obtém o High Water Mark da task atual.
    Lembre-se: tal informação é obtida em words! */
    uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
    
    float tempo_dep = millis();
    tempo_ant = tempo_dep - tempo_ant;

    vTaskDelay(tempo_task_ler_sensores/portTICK_PERIOD_MS);
  }
}

void setup(){

  Serial.begin(9600); // Inicialisa serial port, apenas para print no serial monitor para debug

  pinMode(25, INPUT); // RX do Pzem
  pinMode(33, INPUT); // TX do Pzem
  pinMode(32, INPUT); // GPIO bateria

  SerialController.begin(9600,SERIAL_8N1,25,33); 
  //Serial2.begin(9600,SERIAL_8N1,25,33);

  // Declaração das variáveis no documeto JSON
  JSON_envia_dados["T"][0] = Temp1; // T1 - Sensor do pino 1. No vetor está na posição 0. Está no GPIO 16.
  JSON_envia_dados["T"][1] = Temp2; // T2 - Sensor do pino 2. No vetor está na posição 1. Está no GPIO 17.
  JSON_envia_dados["T"][2] = Temp3; // T3 - Sensor do pino 3. No vetor está na posição 2. Está no GPIO 5.
  JSON_envia_dados["T"][3] = Temp4; // T4 - Sensor do pino 4. No vetor está na posição 3. Está no GPIO 18.
  JSON_envia_dados["T"][4] = Temp5; // T5 - Sensor do pino 5. No vetor está na posição 4. Está no GPIO 19.
  JSON_envia_dados["T"][5] = Temp6; // T6 - Sensor do pino 6. No vetor está na posição 5. Está no GPIO 21.
  JSON_envia_dados["W"] = W; // potência Watts
  JSON_envia_dados["V"] = V; // tensão
  JSON_envia_dados["I"] = I; // corrente
  JSON_envia_dados["FP"] = FP; // fator potência
  JSON_envia_dados["Wh"] = Wh; // watt hora
  JSON_envia_dados["freq"] = Freq; // frequência
  JSON_envia_dados["P1"] = P1; // Sensor de pressão 1. Está no GPIO 35
  JSON_envia_dados["P2"] = P2; // Sensor de pressão 2. Está no GPIO 34
  JSON_envia_dados["Bat"] = percentual_bateria; // Percentual da bateria
  JSON_envia_dados["TED"] = tempo_task_enviar_dados; // Tempo de envio dos dados lidos dos sensores para a aplicação no Android ou nuvem.
  JSON_envia_dados["Gas"] = tipo_gas; // Tempo de envio dos dados lidos dos sensores para a aplicação no Android ou nuvem.

  interrupt.atualizar_estado_portas(); // Rotina para atualizar o estado das portas dos conectores de temperatura.
  interrupt.ativar_interrupcoes(); // Ativa as interrupções das portas dos sensores de temperatura.

  sensortemp.begin(); // Inicialização dos DS18B20.

  //////////////
  setup_wifi(); // Configura o WiFi, caso não esteja usando o WiFi Manager.
  client.setBufferSize(MSG_BUFFER_SIZE); // Setar o tamanho do buffer do payload mqtt.
  client.setServer(mqtt_server, 1883);  // Seta o servidor MQTT (Broker) e a porta (porta padrão).
  client.setCallback(callback); // Seta a função para receber mensagens do tópico no broker MQTT.
  if (!client.connected()) { // Conecta no broker MQTT.
    reconnect();
  }
  client.subscribe("comandosTCC"); // Dar subscribe no tópico que envia os comandos do NodeRed para cá.
  //////////////

  pzem.resetEnergy(); // Reseta as informações salvas internamente no PZEM, como o Wh.

  xTaskCreate(
    enviar_dados,      // Função a ser chamada
    "Enviar dados",    // Nome da tarefa
    5000,              // Tamanho (bytes) This stack size can be checked & adjusted by reading the Stack Highwater
    NULL,              // Parametro a ser passado
    3,                 // Prioridade da tarefa Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    NULL               // Task handle
    //0,          // Núcleo que deseja rodar a tarefa (0 or 1)
  );

xTaskCreate(
    receber_dados,      // Função a ser chamada
    "Receber dados",    // Nome da tarefa
    2000,               // Tamanho (bytes) This stack size can be checked & adjusted by reading the Stack Highwater
    NULL,               // Parametro a ser passado
    2,                  // Prioridade da tarefa Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    NULL               // Task handle
    //0,          // Núcleo que deseja rodar a tarefa (0 or 1)
);

  xTaskCreate(
    ler_sensores,      // Função a ser chamada
    "Ler dados",    // Nome da tarefa
    5000,               // Tamanho (bytes) This stack size can be checked & adjusted by reading the Stack Highwater
    NULL,               // Parametro a ser passado
    1,                  // Prioridade da tarefa Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    NULL               // Task handle
    //0,          // Núcleo que deseja rodar a tarefa (0 or 1)
  );

    Serial.printf("Internal Total heap %d, internal Free Heap %d\n", ESP.getHeapSize(), ESP.getFreeHeap());
    Serial.printf("SPIRam Total heap %d, SPIRam Free Heap %d\n", ESP.getPsramSize(), ESP.getFreePsram());
    Serial.printf("ChipRevision %d, Cpu Freq %d, SDK Version %s\n", ESP.getChipRevision(), ESP.getCpuFreqMHz(), ESP.getSdkVersion());
    Serial.printf("Flash Size %d, Flash Speed %d\n", ESP.getFlashChipSize(), ESP.getFlashChipSpeed());

}

void loop() {}