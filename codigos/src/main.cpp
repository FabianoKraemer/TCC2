#include "Includes.h"
#include <OneWire.h> // Protocolo da Dallas, usado para os NTCs digitais DS18B20
#include <DallasTemperature.h> // Biblioteca do NTC digital DS18B20
#include "HardwareSerial.h" //somente para a ESP32
#include <PZEM004Tv30.h> // biblioteca do Wattímetro, já com a compatibilidade pra software serial ou hardware serial
////////////////////////////////////////////////////////////////////////////////////////////////////////

Interrupcoes interrupt(tempo_db_Pinos_temp);

// Objetos para lidar com os DS18B20
const int oneWireBus = 15; //porta que os NTCs digitais estão conectados
OneWire oneWire(oneWireBus); // Prepara uma instância oneWire para comunicar com qualquer outro dispositivo oneWire.
DallasTemperature sensortemp(&oneWire); // Passa uma referência oneWire para a biblioteca DallasTemperature.

// Criando objeto do wattímetro
#define RX2 25
#define TX2 33
//PZEM004Tv30 pzem(&Serial2); //usa o Serial1 do hardwareserial, pinos instânciados no construtor, padrão, 3 RX e 1 TX

HardwareSerial SerialController(2);

PZEM004Tv30 pzem(&SerialController);


// variable to hold device addresses
DeviceAddress Thermometer;

long tempo_task_enviar_dados = 5000;
long tempo_task_receber_dados = 1000;
long tempo_task_ler_sensores = 3000;


void setup_wifi() {

  delay(10);
  // iniciar conexão wifi
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  //WiFi.config(local_ip,gateway, subnet);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

// receber mensagens
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

}

// reconecta no broker mqtt
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      //client.publish("outTopic", "hello world");
      // ... and resubscribe
      //client.subscribe("inTopic");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void indice_bateria(){
  for (int i = 0; i < 100; i++){
    percentual_bateria = analogRead(porta_bateria) + percentual_bateria;
  }
  percentual_bateria = percentual_bateria / 100;

  if (bateriaMax <= percentual_bateria) bateriaMax = percentual_bateria;
  if (bateriaMin >= percentual_bateria) bateriaMin = percentual_bateria;

  doc["bateriaMax"] = bateriaMax;
  doc["bateriaMin"] = bateriaMin;

  int indiceBateria = map(percentual_bateria, 2800, 4094, 0, 100);

  if (indiceBateria <= 0) indiceBateria = 0;
  else if (indiceBateria >= 100) indiceBateria = 100;
  doc["Bat"] = indiceBateria;

  }

void temperatura(){
    
    interrupt.atualizar_estado_portas(); // Rotina para atualizar o estado das portas dos conectores de temperatura.
    interrupt.retorna_vetor(sensores_conectados); // Verifica quais portas estão com algo conectado ou não. True para conectado, false para desconectado.

    sensortemp.requestTemperatures(); // Adiciona pelo menos meio segundo de tempo de processamento, mas varia muito.

    for (int n = 0; n <= 5; n++){
      if (sensores_conectados[n]) {
        sensortemp.getAddress(Thermometer, n); // Pega o endereço de cada sensor conectado.
        Temps[n] = sensortemp.getTempC(Thermometer);
        doc["T"][n] = Temps[n];
      }

      if (!sensores_conectados[n]) {
        //qtdSensores = qtdSensores - 1;
        Temps[n] = -127;
        doc["T"][n] = Temps[n];
      }
      //vetor = vetor + sensores_conectados[n];
      //vetor = vetor + " | ";
    }

    //Serial.println(vetor);
    //Serial.print("qtde de sensores: ");
    //Serial.println(qtdSensores);

}

void pressao(){

  P1 = 0;
  int i = 0;
  while (i < 50) {
    P1 = P1 + analogRead(porta_P1);
    i++;
  }
  P1 = P1 / 50; // Tira a média das 50 leituras.
  doc["P1"] = P1;
  P1 = (P1 * 0.1875) / 1000; // Obter tensão.
  P1 = (P1 * 30 - 19.8) / 2.64; // fórmula obtida fazendo uma matriz com 0 bar a 30 bar e 0,66V a 3.3V. Valor do 21.8 na formula é 19.8, foi acrescentado 1 para correção das variações e ficar o mesmo valor do manômetro

  P2 = 0;
  i = 0;
  while (i < 50) {
    P2 = P2 + analogRead(porta_P2);;
    i++;
  }
  P2 = P2 / 50; // Tira a média das 50 leituras.
  doc["P2"] = P2;
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
  
  Serial.println(V);
  if (isnan(W)) SerialController.begin(9600,SERIAL_8N1,25,33);
  
  // Condição caso o wattímetro esteja desligado/problema de conexão, substituí o nan (not a number) por -1.
  if (isnan(W)) W = -1;
  if (isnan(V)) V = -1;
  if (isnan(I)) I = -1;
  if (isnan(FP)) FP = -1;
  if (isnan(Wh)) Wh = -1;
  if (isnan(Freq)) Freq = -1;

  Serial.println(V);

  doc["W"] = pzem.power(); // potência Watts
  doc["V"] = pzem.voltage(); // tensão
  doc["I"] = pzem.current(); // corrente
  doc["FP"] = pzem.pf(); // fator potência
  doc["Wh"] = pzem.energy(); // watt hora
  doc["freq"] = pzem.frequency(); // frequência

}

void enviar_dados(void *pvParameters){

  //UBaseType_t uxHighWaterMark; // Variável para identificar o consumo máximo de memória de uma task.



  for(;;){
    float tempo_ant = millis(); 

    // Se perder a conexão WiFi, tenta reconectar.
    while (WiFi.status() != WL_CONNECTED) {}

    // Se perder a conexão com o broker MQTT, tenta reconectar.
    if (!client.connected()) reconnect();

    client.publish("Bateria", dados); // Publica o JSON no tópico do broker MQTT.

   // if(!(client.publish("Bateria", dados))) Serial.println("iih, não enviou pro MQTT");

    /* Obtém o High Water Mark da task atual.
   Lembre-se: tal informação é obtida em words! */
    // uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
    // Serial.print("High water mark (words) da task enviar_dados: ");
    // Serial.println(uxHighWaterMark);
   
    // float tempo_dep = millis();
    // Serial.print("enviar dados, tempo: ");
    // tempo_ant = tempo_dep - tempo_ant;
    // Serial.println(tempo_ant);

    vTaskDelay(tempo_task_enviar_dados/portTICK_PERIOD_MS);
  }

}

void receber_dados(void *pvParameters){

  for(;;){
    float tempo_ant = millis();


    if (!client.connected()) reconnect(); 
    client.loop();

    vTaskDelay(tempo_task_receber_dados/portTICK_PERIOD_MS);
    float tempo_dep = millis();
    Serial.print("Task receber_dados, tempo: ");
    tempo_ant = tempo_dep - tempo_ant;
    Serial.println(tempo_ant);
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

    serializeJson(doc, dados);

    /* Obtém o High Water Mark da task atual.
    Lembre-se: tal informação é obtida em words! */
    uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
    //Serial.print("High water mark (words) da task ler sensores: ");
    //Serial.println(uxHighWaterMark);
    
    float tempo_dep = millis();
    //Serial.print("Task ler sensores, tempo: ");
    tempo_ant = tempo_dep - tempo_ant;
    //Serial.println(tempo_ant);

    vTaskDelay(tempo_task_ler_sensores/portTICK_PERIOD_MS);
  }
}

void setup(){

  Serial.begin(9600); // Inicialisa serial port, apenas para print no serial monitor para debug

  pinMode(25, INPUT);
  pinMode(33, INPUT);

  pinMode(32, INPUT); // GPIO bateria

  SerialController.begin(9600,SERIAL_8N1,25,33); 
  //Serial2.begin(9600,SERIAL_8N1,25,33);
  // Declaração das variáveis no documeto JSON
  //doc["T1"] = Temp1; // T1 - Sensor do pino 1. No vetor está na posição 0. Está no GPIO 16.
  //doc["T2"] = Temp2; // T2 - Sensor do pino 2. No vetor está na posição 1. Está no GPIO 17.
  //doc["T3"] = Temp3; // T3 - Sensor do pino 3. No vetor está na posição 2. Está no GPIO 5.
  //doc["T4"] = Temp4; // T4 - Sensor do pino 4. No vetor está na posição 3. Está no GPIO 18.
  //doc["T5"] = Temp5; // T5 - Sensor do pino 5. No vetor está na posição 4. Está no GPIO 19.
  //doc["T6"] = Temp6; // T6 - Sensor do pino 6. No vetor está na posição 5. Está no GPIO 21.
  doc["T"][0] = Temp1;
  doc["T"][1] = Temp2;
  doc["T"][2] = Temp3;
  doc["T"][3] = Temp4;
  doc["T"][4] = Temp5;
  doc["T"][5] = Temp6;

  doc["W"] = W; // potência Watts
  doc["V"] = V; // tensão
  doc["I"] = I; // corrente
  doc["FP"] = FP; // fator potência
  doc["Wh"] = Wh; // watt hora
  doc["freq"] = Freq; // frequência
  doc["P1"] = P1; // Sensor de pressão 1. Está no GPIO 35
  doc["P2"] = P2; // Sensor de pressão 2. Está no GPIO 34
  doc["Bat"] = percentual_bateria; // Percentual da bateria
  doc["Tat"] = tempo_at; // Tempo de envio dos dados lidos dos sensores para a aplicação no Android ou nuvem

  interrupt.atualizar_estado_portas(); // Rotina para atualizar o estado das portas dos conectores de temperatura.
  interrupt.ativar_interrupcoes(); // Ativa as interrupções das portas dos sensores de temperatura.

  //Serial.print("Locating devices...");
  sensortemp.begin(); // Inicialização dos DS18B20 e rastreio dos conectados
  //Serial.print("Found ");
  //Serial.print(sensortemp.getDeviceCount(), DEC);
  //Serial.println(" devices.");

  //////////////
  setup_wifi(); // Configura o WiFi, caso não esteja usando o WiFi Manager
  client.setBufferSize(MSG_BUFFER_SIZE); // Setar o tamanho do buffer do payload mqtt
  client.setServer(mqtt_server, 1883);  // Seta o servidor MQTT e a porta (porta padrão) 
  client.setCallback(callback); // Seta a função para receber mensagens do tópico no broker MQTT
  if (!client.connected()) {
    reconnect();
  }
  //////////////

//pzem.resetEnergy();

xTaskCreate(
    enviar_dados,      // Função a ser chamada
    "Enviar dados",    // Nome da tarefa
    5000,              // Tamanho (bytes) This stack size can be checked & adjusted by reading the Stack Highwater
    NULL,              // Parametro a ser passado
    3,                 // Prioridade da tarefa Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    NULL               // Task handle
    //0,          // Núcleo que deseja rodar a tarefa (0 or 1)
);

// xTaskCreate(
//     receber_dados,      // Função a ser chamada
//     "Receber dados",    // Nome da tarefa
//     1000,               // Tamanho (bytes) This stack size can be checked & adjusted by reading the Stack Highwater
//     NULL,               // Parametro a ser passado
//     2,                  // Prioridade da tarefa Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
//     NULL               // Task handle
//     //0,          // Núcleo que deseja rodar a tarefa (0 or 1)
// );

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