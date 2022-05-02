#include <Arduino.h>
//#include <Debounce.h>
#include "Interrupcoes.h"
#include "Temperatura.h"

////////////////////////////////////////////////////////////////////////////////////////////////////////
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h> // Json pra Arduino, serializa e desserializa
//#include <Wire.h>
#include <SPI.h>

// Dados pra conectar no wifi e no server mqtt
const char* ssid = "VIVOFIBRA-1650";
const char* password = "6343CC203B";
const char* mqtt_server = "projetoifsc.duckdns.org";
//const char* mqtt_server = "192.168.15.105";
//const char* mqtt_server = "192.168.15.167";
//IPAddress raspberry(192, 168, 15, 105);
//IPAddress local_ip(192, 168, 15, 150);
//IPAddress gateway(192, 168, 15, 1);
//IPAddress subnet(255, 255, 255, 0);
unsigned long prevMillis = 0;
unsigned long delayLoop = 10000; // em milisegundos

WiFiClient WifiClient;
//WifiClient.config(local_ip,gateway, subnet);
PubSubClient client(WifiClient);
unsigned long tempoMsg = 0;
#define MSG_BUFFER_SIZE	(384)
char msg[MSG_BUFFER_SIZE];

//testando JSON
const int capacity = JSON_OBJECT_SIZE(384);
StaticJsonDocument<capacity> doc;
char dados[capacity];

  float bateriaMin = 5000;
  float bateriaMax = 0;

////////////////////////////////////////////////////////////////////////////////////////////////////////


Interrupcoes interrupt;

const int porta_bateria = 32;

//Debounce _deb0(300); // Debouncer for INT0 with 100ms delay
//Debounce _deb1(300);  // Debouncer for INT1 with 70ms delay

/*
volatile static bool teste_vetor[6];

volatile bool pin2, pin3; // Values to store the current pin level
                          // pin2-INT0 / pin3-INT1

void IRAM_ATTR int0handler(){

  //Debounce _deb0(300);
  //Serial.println("entrou na int_1");
  if (_deb0.debounce()) {// If interrupt is valid (debounced)
    if (digitalRead(16) == 0){
      teste_vetor[0] = true;
        //Serial.println("Sensor conectado na P1");
    } else if (digitalRead(16) == 1){
      teste_vetor[0] = false;
       // Serial.println("Sensor desconectado na P1");
    }
  }
}

void IRAM_ATTR int1handler() // INT1 Interrupt Handler
{
  if (_deb1.debounce()){ // If interrupt is valid (debounced)
    if (digitalRead(17) == 0){
      Serial.println("Sensor conectado na P2");
    }
    else if (digitalRead(17) == 1){
      Serial.println("Sensor desconectado na P2");
    }
  }
}
*/

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
    String clientId = "ESP8266Client-";
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


void setup(){

  Serial.begin(9600); // Initialize serial port

  pinMode(16, INPUT);
  pinMode(17, INPUT);
  pinMode(porta_bateria, INPUT);

  interrupt.atualizar_estado_portas();
  interrupt.AtivarInterrupcoes();

  //////////////
  setup_wifi(); 
  client.setBufferSize(385); //setar o tamanho do buffer do payload mqtt
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  //////////////

    // Register interrupt handlers and enable interrupts
    //attachInterrupt(digitalPinToInterrupt(16), int0handler, CHANGE);
    //attachInterrupt(digitalPinToInterrupt(17), int1handler, CHANGE);
}

void loop() { // Print pin levels every 50ms

  float bateria = 0;

  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  

  for (int i = 0; i < 100; i++){
    bateria = analogRead(porta_bateria) + bateria;
  }
  bateria = bateria / 100;

  if (bateriaMax <= bateria) bateriaMax = bateria;
  if (bateriaMin >= bateria) bateriaMin = bateria;

  doc["rawBateria"] = bateria;
  doc["bateriaMax"] = bateriaMax;
  doc["bateriaMin"] = bateriaMin;

  int indiceBateria = map(bateria, 2800, 4094, 0, 100);

  if (indiceBateria <= 0) indiceBateria = 0;
  else if (indiceBateria >= 100) indiceBateria = 100;
  doc["indiceBateria"] = indiceBateria;

  serializeJson(doc, dados);

  unsigned long tempo = millis();
  
  if((tempo - prevMillis) >= delayLoop){
    client.publish("Bateria", dados);
    Serial.println((tempo - prevMillis));
    prevMillis = tempo;
    
  }
  

  //Serial.println("oi");
    //Serial.print(pin2);
    //Serial.print("|");
    //Serial.println(pin3);    
    //Serial.println(digitalRead(16));
  
    //interrupt.imprimir();
    
    //Serial.println(teste_vetor[0]);
    //delay(10000);
}