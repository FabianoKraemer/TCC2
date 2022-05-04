#include "Includes.h"
#include <OneWire.h> // Protocolo da Dallas, usado para os NTCs digitais DS18B20
#include <DallasTemperature.h> // Biblioteca do NTC digital DS18B20
#include "HardwareSerial.h" //somente para a ESP32
////////////////////////////////////////////////////////////////////////////////////////////////////////

Interrupcoes interrupt(tempo_db_Pinos_temp);

// Objetos para lidar com os DS18B20
const int oneWireBus = 15; //porta que os NTCs digitais estão conectados
OneWire oneWire(oneWireBus);
DallasTemperature sensortemp(&oneWire);


// variable to hold device addresses
DeviceAddress Thermometer;

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

  doc["Bat"] = percentual_bateria;
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

    String vetor; // Para imprimir no serial monitor
    qtdSensores = 0;
    for (int n = 0; n <= 5; n++){
      if (sensores_conectados[n]) {
        qtdSensores = qtdSensores + 1;
        sensortemp.requestTemperatures();
        sensortemp.getAddress(Thermometer, n);
        Temps[n] = sensortemp.getTempC(Thermometer);
        doc["T"][n] = Temps[n];
      }   
      if (!sensores_conectados[n]) {
        //qtdSensores = qtdSensores - 1;
        Temps[n] = -127;
        doc["T"][n] = Temps[n];
      }
      vetor = vetor + sensores_conectados[n];
      vetor = vetor + " | ";
    }

    Serial.println(vetor);
    Serial.print("qtde de sensores: ");
    Serial.println(qtdSensores);

}

void setup(){

  Serial.begin(9600); // Inicialisa serial port, apenas para print no serial monitor para debug

  pinMode(16, INPUT);
  pinMode(17, INPUT);
  pinMode(32, INPUT); // GPIO bateria

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
  //////////////

}

void loop() { // Print pin levels every 50ms

  if (!client.connected()) reconnect(); 
  client.loop();

  indice_bateria();

  serializeJson(doc, dados);

  ////////////////////////////////////////////////////////
  unsigned long tempo = millis();

  if ((tempo - prevMillis) >= delayLoop){
    client.publish("Bateria", dados);
    //Serial.println((tempo - prevMillis)); // tempo entre loops
    prevMillis = tempo;

    temperatura();
    //interrupt.imprimir();

    for (int i = 0; i < 6; i++){

      Serial.print("Temperaturas: ");
      Serial.print(Temps[i]);
      Serial.print("C° | ");
      Serial.println("");
    }
  }
////////////////////////////////////////////////////////


}