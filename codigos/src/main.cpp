#include "Includes.h"
////////////////////////////////////////////////////////////////////////////////////////////////////////


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

  }

void setup(){

  Serial.begin(9600); // Inicialisa serial port, apenas para print no serial monitor para debug

  pinMode(16, INPUT);
  pinMode(17, INPUT);
  pinMode(32, INPUT); // GPIO bateria

  interrupt.atualizar_estado_portas();
  interrupt.ativar_interrupcoes();

  //////////////
  setup_wifi(); // Configura o WiFi, caso não esteja usando o WiFi Manager
  client.setBufferSize(MSG_BUFFER_SIZE); // Setar o tamanho do buffer do payload mqtt
  client.setServer(mqtt_server, 1883);  // Seta o servidor MQTT e a porta (porta padrão) 
  client.setCallback(callback); // Seta a função para receber mensagens do tópico no broker MQTT
  //////////////

}

void loop() { // Print pin levels every 50ms

  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  indice_bateria();

  serializeJson(doc, dados);

  unsigned long tempo = millis();
  
  if((tempo - prevMillis) >= delayLoop){
    client.publish("Bateria", dados);
    Serial.println((tempo - prevMillis));
    prevMillis = tempo;

    interrupt.imprimir();
    volatile bool testevetor[6];
    interrupt.retorna_vetor(testevetor);

    String vetor; 

    for (int n = 0; n <= 5; n++){
      vetor = vetor + testevetor[n];
      vetor = vetor + " | ";
    }

  Serial.println(vetor);
    
  }
  
  

    
  

}