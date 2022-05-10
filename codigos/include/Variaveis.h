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
unsigned long delayLoop = 5000; // em milisegundos

WiFiClient WifiClient;
//WifiClient.config(local_ip,gateway, subnet);
PubSubClient client(WifiClient); // Criação do objeto MQTT
unsigned long tempoMsg = 0;
#define MSG_BUFFER_SIZE	(2000) // Declaração do tamanho do buffer da mensagem MQTT
char msg[MSG_BUFFER_SIZE]; // Criação do array char da mensagem MQTT

//testando JSON
//const int capacity = JSON_OBJECT_SIZE(384);
const int capacity = JSON_OBJECT_SIZE(MSG_BUFFER_SIZE);
StaticJsonDocument<capacity> doc;
char dados[capacity];

// Tempo de envio dos dados lidos dos sensores para a aplicação no Android ou nuvem
float tempo_at = 5000; // 5000 milisegundos por padrão

// Variáveis sensores de temperatura. DS18B20 range: -55°C a +125°C. Valores iniciados com -127 pois é o valor padrão da biblioteca caso sensor esteja desconectado.
static float Temp1 = -127;
static float Temp2 = -127;
static float Temp3 = -127;
static float Temp4 = -127;
static float Temp5 = -127;
static float Temp6 = -127;
static float Temps[6] = {-127, -127, -127, -127, -127, -127,};
volatile static bool sensores_conectados[6]; // vetor booleano com os estados das portas dos sensores de temperatura. True para conectado, false para desconectado
int qtdSensores = 0; // quantidade de sensores conectados
//static const int oneWireBus = 15; // Porta que os NTCs digitais estão conectados
//int ndispositivos = 0; // Número de sensores DS18B20 conectados

// Variáveis Wattímetro. Valores iniciados com negativo para sinalizar não uso/conexão do Wattímetro
static float W = -1;   // Potência instantânea
static float V = -1;   // Tensão
static float I = -1;   // Corrente
static float FP = -1;  // Fator de Potência
static float Freq = -1;// Frequência da rede
static float Wh = -1;  // Watt hora

// Variáveis dos sensores de pressão. Valores iniciados com negativo para sinalizar não uso/conexão dos sensores de pressão
static float P1 = -1;
static float P2 = -2;
static unsigned int porta_P1 = 35;
static unsigned int porta_P2 = 34;


// Indicador de percentual de carga da bateria
float percentual_bateria = 0;


float bateriaMin = 5000;
float bateriaMax = 0;
const int porta_bateria = 32;

unsigned long tempo_db_Pinos_temp = 300; // tempo do debouncer das portas dos sensores de temperatura, em milisegundos

//interrupt.tempo_debounce(testando);



#endif //VARIAVEIS_h__