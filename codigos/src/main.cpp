#include "Includes.h" // Todos os includes necessários para o projeto
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <esp_system.h>
//#include "OTA.h"

//WebServer server(80);

TaskHandle_t xHandle_enviar_dados = NULL;
TaskHandle_t xHandle_conexoes_wireless = NULL;
TaskHandle_t xHandle_ler_sensores = NULL;
TaskHandle_t xHandle_receber_comandos = NULL;
TaskHandle_t xHandle_Debug_MQTT = NULL;

long tempo_enviar_dados = 0;
long tempo_conexoes_wireless = 0;
long tempo_ler_sensores = 0;
long tempo_receber_comandos = 0;
long tempo_debug_mqtt = 0;

/* Semáforos */
SemaphoreHandle_t xDebug_semaphore;
// Variáveis globais utilizadas estão no "Variaveis.h"
////////////////////////////////////////////////////////////////////////////////////////////////////////

Interrupcoes interrupt(tempo_db_Pinos_temp); // Objeto das interrupções dos GPIOs dos sensores de temperatura.

void atualizacao_OTA(){

  if (WiFi.status() == WL_CONNECTED){ // Só entra na condição se o WiFi estiver conectado a uma rede.
  
    // Atende uma solicitação para a raiz e devolve a página 'verifica'.
    //
    server.on("/", HTTP_GET, [](){ // Atende uma solicitação para a raiz   
      server.sendHeader("Connection", "close");
      server.send(200, "text/html", verifica);
    });

    // Atende uma solicitação para a página avalia
    server.on("/avalia", HTTP_POST, [] (){
       Serial.println("Em server.on /avalia: args= " + String(server.arg("autorizacao"))); //somente para debug

      if (server.arg("autorizacao") != "projetoIFSC"){// confere se o dado de autorização atende a avaliação
      
        // Se não atende, serve a página indicando uma falha
        server.sendHeader("Connection", "close");
        server.send(200, "text/html", Resultado_Falha);
        //ESP.restart();
      }
      else{      
        // Se atende, solicita a página de índice do servidor e sinaliza que o OTA está autorizado.
        OTA_AUTORIZADO = true;
        server.sendHeader("Connection", "close");
        server.send(200, "text/html", serverIndex);
      }
    });

    // Serve a página de indice do servidor para seleção do arquivo.
    server.on("/serverIndex", HTTP_GET, [](){
      server.sendHeader("Connection", "close");
      server.send(200, "text/html", serverIndex);
    });

    // Tenta iniciar a atualização . . .
    server.on("/update", HTTP_POST, [](){
      // Verifica se a autorização é false.
      // Se for falsa, serve a página de erro e cancela o processo.
      if (OTA_AUTORIZADO == false){
        server.sendHeader("Connection", "close");
        server.send(200, "text/html", Resultado_Falha);
        return;
      }
      // Serve uma página final que depende do resultado da atualização.
      server.sendHeader("Connection", "close");
      server.send(200, "text/html", (Update.hasError()) ? Resultado_Falha : Resultado_Ok);
      delay(1000);
      ESP.restart();
    }, [](){
      //Mas estiver autorizado, inicia a atualização
      HTTPUpload& upload = server.upload();
      if (upload.status == UPLOAD_FILE_START){
        Serial.setDebugOutput(true);
        Serial.printf("Atualizando: %s\n", upload.filename.c_str());
        if (!Update.begin()){
          //se a atualização não iniciar, envia para serial mensagem de erro.
          Update.printError(Serial);
        }
      }
      else if (upload.status == UPLOAD_FILE_WRITE){
        if (Update.write(upload.buf, upload.currentSize) != upload.currentSize){
          //se não conseguiu escrever o arquivo, envia erro para serial
          Update.printError(Serial);
        }
      }
      else if (upload.status == UPLOAD_FILE_END){
        if (Update.end(true)){
          //se finalizou a atualização, envia mensagem para a serial informando
          Serial.printf("Atualização bem sucedida! %u\nReiniciando...\n", upload.totalSize);
        }
        else{
          //se não finalizou a atualização, envia o erro para a serial.
          Update.printError(Serial);
        }
        Serial.setDebugOutput(false);
      }
      else{
        // se não conseguiu identificar a falha no processo, envia uma mensagem para a serial
        Serial.printf("Atualização falhou inesperadamente! (possivelmente a conexão foi perdida.): status=%d\n", upload.status);
      }
    });

    server.begin(); // inicia o servidor

    Serial.println(INFOS); //envia as informações armazenadas em INFOS, para debug

    // Envia ara a serial o IP atual do ESP
    Serial.print("Servidor em: ");
    Serial.println( WiFi.localIP().toString() + ":" + PORTA);

    /////////////////////////////////////////////////////////
    // Serial.print("SubnetMask: ");
    // Serial.println( WiFi.subnetMask().toString());
    //     Serial.print("gatewayIP: ");
    // Serial.println( WiFi.gatewayIP().toString());
    //     Serial.print("dnsIP: ");
    // Serial.println( WiFi.dnsIP().toString());
    //     Serial.print("broadcastIP: ");
    // Serial.println( WiFi.broadcastIP().toString());
    //       Serial.print("networkID: ");
    // Serial.println( WiFi.networkID().toString());
    //           Serial.print("subnetCIDR: ");
    // Serial.println( WiFi.subnetCIDR());
    //           Serial.print("RSSI: ");
    // Serial.println( WiFi.RSSI());


  }
  else{
    // Avisa se não conseguir conectar no WiFi
    Serial.println("Falha ao conectar ao WiFi.");
  }
}

void wifi(){
  
    if(!wifiManager.autoConnect("ESP_AP", "testeesp")){
       wifiManager.startConfigPortal("ESP_AP", "testeesp"); // Caso perca a conexão, abilita novamente o AP pra ser conectado pelo celular e configurar/logar em nova rede WiFi.
    wifiManager.process(); // Ativa a página do portal, pra verificar o wifi, desconectar, update do firmware, reset da ESP  
    }
    //wifiManager.erase();
    //wifiManager.resetSettings();

}

// Reconecta no broker MQTT.
void reconnect() {

  if (!client.connected()) {
    if (client.connect("projeto_1")) {
      client.subscribe("comandosTCC"); // Dar subscribe no tópico que envia os comandos do NodeRed para cá.
    }
    //delay(100); // Delay de 1 segundo para cada tentativa de reconectar no broker MQTT.  
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

  xSemaphoreTake(xDebug_semaphore, portMAX_DELAY);
  JSON_envia_dados["bateriaMax"] = bateriaMax;
  JSON_envia_dados["bateriaMin"] = bateriaMin;
  xSemaphoreGive(xDebug_semaphore);

  int indiceBateria = map(percentual_bateria, 2800, 4094, 0, 100); // Mapeia os valores lidos para o intervalo de 0 a 100, para ficar em percentual.

  if (indiceBateria <= 0) indiceBateria = 0; // Caso ocorra alguma leitura menor que 2800 antes do map, fica valor negativo, estabelece o valor 0 para que não ocorra isso.
  else if (indiceBateria >= 100) indiceBateria = 100; // Caso o valor lido tenha ficado maior que 4094, para não dar valor maior que 100, estabelece esse limite. 

  xSemaphoreTake(xDebug_semaphore, portMAX_DELAY);
  JSON_envia_dados["Bat"] = indiceBateria;
  xSemaphoreGive(xDebug_semaphore);
}

void temperatura(){
    
    interrupt.atualizar_estado_portas(); // Rotina para atualizar o estado das portas dos conectores de temperatura.
    interrupt.retorna_vetor(sensores_conectados); // Verifica quais portas estão com algo conectado ou não. True para conectado, false para desconectado.

    sensortemp.requestTemperatures(); // Adiciona pelo menos meio segundo de tempo de processamento, mas varia muito.

    for (int n = 0; n <= 5; n++){
      if (sensores_conectados[n]) { // Se no n específico, tiver um sensor conectado, entrará na condição para pegar o endereço do sensor e medir a temperatura.
        sensortemp.getAddress(Thermometer, n); // Pega o endereço de cada sensor conectado.
        //Temps[n] = sensortemp.getTempC(Thermometer); // Pega a temperatura já convertida para Celsius e grava na posição do sensor no pino no vetor correspondente.
        xSemaphoreTake(xDebug_semaphore, portMAX_DELAY);
        JSON_envia_dados["T"][n] = sensortemp.getTempC(Thermometer);; // Grava o valor da temperatura no vetor do JSON da temperatura.
        xSemaphoreGive(xDebug_semaphore);
      }

      if (!sensores_conectados[n]) { // Se no n específico, não tiver um sensor conectado, entra na condição. 
        //Temps[n] = -127; // Gravará o valor -127. Esse valor é padrão na biblioteca da Dallas para sensor não conectado.
        xSemaphoreTake(xDebug_semaphore, portMAX_DELAY);
        JSON_envia_dados["T"][n] = -127; // Grava o valor da temperatura no vetor do JSON da temperatura.
        xSemaphoreGive(xDebug_semaphore);
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
  // P1 = P1 / 50; // Tira a média das 50 leituras.
  xSemaphoreTake(xDebug_semaphore, portMAX_DELAY);
  JSON_envia_dados["P1"] = (P1 / 50);
  xSemaphoreGive(xDebug_semaphore);
  P1 = (P1 * 0.1875) / 1000; // Obter tensão.
  P1 = (P1 * 30 - 19.8) / 2.64; // fórmula obtida fazendo uma matriz com 0 bar a 30 bar e 0,66V a 3.3V. Valor do 21.8 na formula é 19.8, foi acrescentado 1 para correção das variações e ficar o mesmo valor do manômetro

  P2 = 0;
  i = 0;
  while (i < 50) {
    P2 = P2 + analogRead(porta_P2);;
    i++;
  }
  //P2 = P2 / 50; // Tira a média das 50 leituras.
  xSemaphoreTake(xDebug_semaphore, portMAX_DELAY);
  JSON_envia_dados["P2"] = (P2 / 50);
  xSemaphoreGive(xDebug_semaphore);
  P2 = (P2 * 0.1875) / 1000; // Obter tensão.
  P2 = (P2 * 10 - 6.6) / 2.64; // Fórmula obtida fazendo uma matriz com 0 bar a 10 bar e 0,66V a 3.3V.

  //pressaoAlta = pressaoAlta * 14.504; //convertendo pra psi

}

void wattimetro(){

  V = pzem.voltage(); // tensão
  I = pzem.current(); // corrente
  W = pzem.power(); // potência Watts
  Wh = pzem.energy(); // watt hora
  Freq = pzem.frequency(); // frequência
  FP = pzem.pf(); // fator potência
  
  if (isnan(W)) SerialController.begin(9600,SERIAL_8N1,25,33); // Tentar reconectar no Wattimetro caso ocorra perda de conexão.
  
  // Condição caso o wattímetro esteja desligado/problema de conexão, substituí o nan (not a number) por -1.
  if (isnan(W)) W = -1;
  if (isnan(V)) V = -1;
  if (isnan(I)) I = -1;
  if (isnan(FP)) FP = -1;
  if (isnan(Wh)) Wh = -1;
  if (isnan(Freq)) Freq = -1;

  xSemaphoreTake(xDebug_semaphore, portMAX_DELAY);
  JSON_envia_dados["W"] = W; // potência Watts
  JSON_envia_dados["V"] = V; // tensão
  JSON_envia_dados["I"] = I; // corrente
  JSON_envia_dados["FP"] = FP; // fator potência
  JSON_envia_dados["Wh"] = Wh; // watt hora
  JSON_envia_dados["freq"] = Freq; // frequência
  xSemaphoreGive(xDebug_semaphore);

}

void conexoes_wireless(void *pvParameters){

  //UBaseType_t uxHighWaterMark; // Variável para identificar o consumo máximo de memória de uma task.

  for (;;){
    float tempo_ant = millis();

    // Se perder a conexão WiFi, tenta reconectar.
    if (WiFi.status() != WL_CONNECTED && ativar_wifi == true){
      Serial.println("desconectado");
      wifi();
    }
    
    if (WiFi.status() == WL_CONNECTED){
      //Serial.println("CONECTADO");
      //wifiManager.stopConfigPortal();
      //wifiManager.stopWebPortal();
      //atualizacao_OTA();      
      if (!client.connected()) reconnect(); // Se perder a conexão com o broker MQTT, tenta reconectar.
      server.handleClient(); // Escutar a porta do webserver para atualização via OTA.
    }


    /* Obtém o High Water Mark da task atual.
    Lembre-se: tal informação é obtida em words! */
    //uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);

    float tempo_dep = millis();
    xSemaphoreTake(xDebug_semaphore, portMAX_DELAY );
    tempo_conexoes_wireless = tempo_dep - tempo_ant;
    xSemaphoreGive(xDebug_semaphore);

    vTaskDelay(tempo_task_conexoes_wireless / portTICK_PERIOD_MS);
  }
}

void enviar_dados(void *pvParameters){

  //UBaseType_t uxHighWaterMark; // Variável para identificar o consumo máximo de memória de uma task.

  for(;;){
    float tempo_ant = millis(); 

    // Usando semáforo pois o JSON está sendo usada em duas tasks com tempos de execução e prioridades diferentes.
      xSemaphoreTake(xDebug_semaphore, portMAX_DELAY );
      serializeJson(JSON_envia_dados, dados_envio);
      if(WiFi.status() == WL_CONNECTED && client.connected()) client.publish("subscriberTCC", dados_envio); // Publica o JSON no tópico do broker MQTT.
      xSemaphoreGive(xDebug_semaphore);

    /* Obtém o High Water Mark da task atual.
   Lembre-se: tal informação é obtida em words! */
   // uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );

    float tempo_dep = millis();
    xSemaphoreTake(xDebug_semaphore, portMAX_DELAY );
    tempo_enviar_dados = tempo_dep - tempo_ant;
    xSemaphoreGive(xDebug_semaphore);

    vTaskDelay(tempo_task_enviar_dados/portTICK_PERIOD_MS);
  }

}

void receber_comandos(void *pvParameters){

  //UBaseType_t uxHighWaterMark; // Variável para identificar o consumo máximo de memória de uma task
  //wifiManager.resetSettings();

  for(;;){
    float tempo_ant = millis();

    if (client.connected() && WiFi.status() == WL_CONNECTED) client.loop(); // Função para verificar se existe mensagem nova no subscriber do MQTT e já grava os dados na variável comandos_recebidos.

    //uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
    float tempo_dep = millis();   
    xSemaphoreTake(xDebug_semaphore, portMAX_DELAY );
    tempo_receber_comandos = tempo_dep - tempo_ant;
    xSemaphoreGive(xDebug_semaphore);

    vTaskDelay(tempo_task_receber_comandos/portTICK_PERIOD_MS); 
    
  }
}

void ler_sensores(void *pvParameters){

  //UBaseType_t uxHighWaterMark; // Variável para identificar o consumo máximo de memória de uma task

  for(;;){
    float tempo_ant = millis();

    indice_bateria();
    temperatura();
    pressao();
    wattimetro();

    /* Obtém o High Water Mark da task atual.
    Lembre-se: tal informação é obtida em words! */
    //uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
    
    float tempo_dep = millis();
    xSemaphoreTake(xDebug_semaphore, portMAX_DELAY );
    tempo_ler_sensores = tempo_dep - tempo_ant;
    xSemaphoreGive(xDebug_semaphore);

    vTaskDelay(tempo_task_ler_sensores/portTICK_PERIOD_MS);
  }
}

void Debug_MQTT(void *pvParameters){

  UBaseType_t uxHighWaterMark; // Variável para identificar o consumo máximo de memória de uma task

  for(;;){
    float tempo_ant = millis();

    if (!client.connected()) reconnect(); // Caso não esteja conectado no broker MQTT, tenta reconectar.

    uxHighWaterMark = uxTaskGetStackHighWaterMark(xHandle_enviar_dados);
    JSON_DEBUG_MQTT["Size_enviar_dados"] = (stack_size_enviar_dados - uxHighWaterMark);

    uxHighWaterMark = uxTaskGetStackHighWaterMark(xHandle_conexoes_wireless);
    JSON_DEBUG_MQTT["Size_conex_wirl"] = (stack_size_conexoes_wireless - uxHighWaterMark);

    uxHighWaterMark = uxTaskGetStackHighWaterMark(xHandle_receber_comandos);
    JSON_DEBUG_MQTT["Size_rec_com"] = (stack_size_receber_comandos - uxHighWaterMark);

    uxHighWaterMark = uxTaskGetStackHighWaterMark(xHandle_ler_sensores);
    JSON_DEBUG_MQTT["Size_ler_sens"] = (stack_size_ler_sensores - uxHighWaterMark);

    uxHighWaterMark = uxTaskGetStackHighWaterMark(xHandle_Debug_MQTT);
    JSON_DEBUG_MQTT["Size_debug"] = (stack_size_debug_mqtt - uxHighWaterMark);

    // Usando semáforo pois o JSON está sendo usada em duas tasks com tempos de execução e prioridades diferentes.
    xSemaphoreTake(xDebug_semaphore, portMAX_DELAY );
    JSON_DEBUG_MQTT["T_enviar_dados"] = tempo_enviar_dados;
    JSON_DEBUG_MQTT["T_conex_wirl"] = tempo_conexoes_wireless;
    JSON_DEBUG_MQTT["T_ler_sens"] = tempo_ler_sensores;
    JSON_DEBUG_MQTT["T_rec_com"] = tempo_receber_comandos;
    JSON_DEBUG_MQTT["T_debug"] = tempo_debug_mqtt;  
    JSON_DEBUG_MQTT["MinFreeHeap"] = ESP.getMinFreeHeap();
    serializeJson(JSON_DEBUG_MQTT, DEBUG_MQTT);
    if(WiFi.status() == WL_CONNECTED && client.connected()) client.publish("subscriber_Debug_TCC", DEBUG_MQTT); // Publica o JSON no tópico do broker MQTT.
    xSemaphoreGive(xDebug_semaphore);

    float tempo_dep = millis();   
    xSemaphoreTake(xDebug_semaphore, portMAX_DELAY );
    tempo_debug_mqtt = tempo_dep - tempo_ant;
    xSemaphoreGive(xDebug_semaphore);

    vTaskDelay(tempo_task_debug_mqtt/portTICK_PERIOD_MS); 
    
  }
}

void setup(){

  Serial.begin(9600); // Inicialisa serial port, apenas para print no serial monitor para debug

  pinMode(25, INPUT); // RX do Pzem
  pinMode(33, INPUT); // TX do Pzem
  pinMode(32, INPUT); // GPIO bateria

  SerialController.begin(9600,SERIAL_8N1,25,33); // Ativando o hardware serial para conexão com o Wattimetro PZEM
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
  //setup_wifi(); // Configura o WiFi, caso não esteja usando o WiFi Manager.
  //wifiManager.erase();
  //wifiManager.resetSettings();
  

  wifiManager.setConfigPortalBlocking(false); // Não deixa que o WiFiManager bloqueie a execução enquanto não estiver conectado em nada.
  wifiManager.setWiFiAutoReconnect(true); // Permite a reconexão em rede WiFi, caso perca a conexão momentaneamente.
  wifiManager.setDebugOutput(false); // Desabilita as saídas impressas no serial.

  wifiManager.autoConnect("ESP_AP", "testeesp");
  //delay(100); 
  //wifiManager.startWebPortal();

  if (WiFi.status() == WL_CONNECTED) atualizacao_OTA(); // Prepara a ESP como webserver para ser conectada e atualizar via OTA.

  client.setBufferSize(MSG_BUFFER_SIZE); // Setar o tamanho do buffer do payload mqtt.
  client.setServer(mqtt_server, 1883);  // Seta o servidor MQTT (Broker) e a porta (porta padrão).
  client.setCallback(callback); // Seta a função para receber mensagens do tópico no broker MQTT.
  // if (!client.connected()) { // Conecta no broker MQTT.
  // //Serial.println("4");
  //   reconnect();
  //   //Serial.println("5");
  // }
  client.subscribe("comandosTCC"); // Dar subscribe no tópico que envia os comandos do NodeRed para cá.
  //////////////

  pzem.resetEnergy(); // Reseta as informações salvas internamente no PZEM, como o Wh.

  xDebug_semaphore = xSemaphoreCreateMutex(); // Criação do semáforo.

  xTaskCreate(
    enviar_dados,      // Função a ser chamada
    "Enviar dados",    // Nome da tarefa
    stack_size_enviar_dados,              // Tamanho (bytes) This stack size can be checked & adjusted by reading the Stack Highwater
    NULL,              // Parametro a ser passado
    4,                 // Prioridade da tarefa Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    &xHandle_enviar_dados               // Task handle
    //0,          // Núcleo que deseja rodar a tarefa (0 or 1)
  );

xTaskCreate(
    receber_comandos,      // Função a ser chamada
    "Receber comandos",    // Nome da tarefa
    stack_size_receber_comandos,               // Tamanho (bytes) This stack size can be checked & adjusted by reading the Stack Highwater
    NULL,               // Parametro a ser passado
    3,                  // Prioridade da tarefa Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    &xHandle_receber_comandos               // Task handle
    //0,          // Núcleo que deseja rodar a tarefa (0 or 1)
);

  xTaskCreate(
    ler_sensores,      // Função a ser chamada
    "Ler dados",    // Nome da tarefa
    stack_size_ler_sensores,               // Tamanho (bytes) This stack size can be checked & adjusted by reading the Stack Highwater
    NULL,               // Parametro a ser passado
    2,                  // Prioridade da tarefa Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    &xHandle_ler_sensores              // Task handle
    //0,          // Núcleo que deseja rodar a tarefa (0 or 1)
  );

  xTaskCreate(
    conexoes_wireless,      // Função a ser chamada
    "Conexoes wireless",    // Nome da tarefa
    stack_size_conexoes_wireless,               // Tamanho (bytes) This stack size can be checked & adjusted by reading the Stack Highwater
    NULL,               // Parametro a ser passado
    5,                  // Prioridade da tarefa Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    &xHandle_conexoes_wireless               // Task handle
    //0,          // Núcleo que deseja rodar a tarefa (0 or 1)
  );
  
  xTaskCreate(
    Debug_MQTT,      // Função a ser chamada
    "Debug MQTT",    // Nome da tarefa
    stack_size_debug_mqtt,               // Tamanho (bytes) This stack size can be checked & adjusted by reading the Stack Highwater
    NULL,               // Parametro a ser passado
    1,                  // Prioridade da tarefa Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    &xHandle_Debug_MQTT               // Task handle
    //0,          // Núcleo que deseja rodar a tarefa (0 or 1)
  );

   //Serial.printf("Internal Total heap %d, internal Free Heap %d\n", ESP.getHeapSize(), ESP.getFreeHeap());
   //Serial.printf("SPIRam Total heap %d, SPIRam Free Heap %d\n", ESP.getPsramSize(), ESP.getFreePsram());
   //Serial.printf("ChipRevision %d, Cpu Freq %d, SDK Version %s\n", ESP.getChipRevision(), ESP.getCpuFreqMHz(), ESP.getSdkVersion());
   //Serial.printf("Flash Size %d, Flash Speed %d\n", ESP.getFlashChipSize(), ESP.getFlashChipSpeed());

}

void loop() {
  
    // Serial.print("remoteIP: ");
    // Serial.println(WifiClient.remoteIP());
    // delay(2000);
    //  Serial.println("Dados do wifi:");
    // Serial.print("remoteIP: ");
    // Serial.println(WifiClient.remoteIP());
    // //dados_json["remoteIP"] = espClient.remoteIP(); // json não aceita tipo IPADDRESS
    
    // //Serial.print("WiFI remote IP: ");
    // //Serial.println(WiFi.remoteIP());

    // Serial.print("remotePort: ");
    // Serial.println(WifiClient.remotePort());
    // // Serial.print("WiFI remote port: ");
    // // Serial.println(WiFi.remotePort());

}