#ifndef VARIAVEIS_h__
#define VARIAVEIS_h__

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
PubSubClient client(WifiClient); // Criação do objeto MQTT
unsigned long tempoMsg = 0;
#define MSG_BUFFER_SIZE	(1000) // Declaração do tamanho do buffer da mensagem MQTT
char msg[MSG_BUFFER_SIZE]; // Criação do array char da mensagem MQTT

//testando JSON
//const int capacity = JSON_OBJECT_SIZE(384);
const int capacity = JSON_OBJECT_SIZE(MSG_BUFFER_SIZE);
StaticJsonDocument<capacity> doc;
char dados[capacity];

float bateria = 0;
float bateriaMin = 5000;
float bateriaMax = 0;
const int porta_bateria = 32;

unsigned long tempo_db_Pinos_temp = 300; // tempo do debouncer das portas dos sensores de temperatura, em milisegundos
Interrupcoes interrupt(tempo_db_Pinos_temp);

//interrupt.tempo_debounce(testando);

volatile static bool sensores_conectados[6]; // vetor booleano com os estados das portas dos sensores de temperatura. True para conectado, false para desconectado

#endif //VARIAVEIS_h__