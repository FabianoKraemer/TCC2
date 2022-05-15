#ifndef VARIAVEIS_h__
#define VARIAVEIS_h__

// Dados pra conectar no wifi e no server mqtt
//const char* ssid = "xxxxxxxxxx";
//const char* password = "xxxxxxxxxx";
const char* mqtt_server = "projetoifsc.duckdns.org";
static bool ativar_wifi = true;
//IPAddress raspberry(192, 168, 15, 105);
//IPAddress local_ip(192, 168, 15, 150);
//IPAddress gateway(192, 168, 15, 1);
//IPAddress subnet(255, 255, 255, 0);
unsigned long prevMillis = 0;

WiFiClient WifiClient;
WiFiManager wifiManager;
//WifiClient.config(local_ip,gateway, subnet);
PubSubClient client(WifiClient); // Criação do objeto MQTT
unsigned long tempoMsg = 0;
#define MSG_BUFFER_SIZE	(2000) // Declaração do tamanho do buffer da mensagem MQTT
//char msg[MSG_BUFFER_SIZE]; // Criação do array char da mensagem MQTT

//const int capacity = JSON_OBJECT_SIZE(384);
const int capacity = JSON_OBJECT_SIZE(MSG_BUFFER_SIZE);
StaticJsonDocument<capacity> JSON_envia_dados; // Objeto JSON que recebe os dados dos sensores e é serializado para envio pelo MQTT.
char dados_envio[capacity]; // Array char das leituras dos sensores, enviado para o publisher MQTT.
DynamicJsonDocument JSON_recebe_comandos(100);
char comandos_recebidos[100]; // dados que serão recebidos do broker mqtt pros comandos do compressor e exaustores/coolers/ventiladores

// Tempo de envio dos dados lidos dos sensores para a aplicação no Android ou nuvem
static long tempo_task_enviar_dados = 5000; // Tempo do ticket da task que envia os dados por WiFi e/ou Bluetooth. 5000 milisegundos por padrão.
static long tempo_task_receber_dados = 1000; // Tempo de execução da Task que recebe os comandos via WiFi e/ou Bluetooth.
static long tempo_task_ler_sensores = 3000; // Tempo de execução da Task que aciona as tarefas de ler todos os sensores.
static long tempo_task_conexoes_wireless = 500; // Tempo de loop da task de verificação e conexões WiFi e Bluetooth.



// Variáveis sensores de temperatura. DS18B20 range: -55°C a +125°C. Valores iniciados com -127 pois é o valor padrão da biblioteca caso sensor esteja desconectado.
static float Temp1 = -127;
static float Temp2 = -127;
static float Temp3 = -127;
static float Temp4 = -127;
static float Temp5 = -127;
static float Temp6 = -127;
static float Temps[6] = {-127, -127, -127, -127, -127, -127,};
volatile static bool sensores_conectados[6]; // Vetor booleano com os estados das portas dos sensores de temperatura. True para conectado, false para desconectado.
// Objetos para lidar com os DS18B20
static const int oneWireBus = 15; // Porta que os NTCs digitais estão conectados.
OneWire oneWire(oneWireBus); // Prepara uma instância oneWire para comunicar com qualquer outro dispositivo oneWire.
DallasTemperature sensortemp(&oneWire); // Passa uma referência oneWire para a biblioteca DallasTemperature.
//int qtdSensores = 0; // Quantidade de sensores conectados.
//int ndispositivos = 0; // Número de sensores DS18B20 conectados.
DeviceAddress Thermometer; // Variável que armazena o endereço do dispositivo (device adress).

// Variáveis Wattímetro. Valores iniciados com negativo para sinalizar não uso/conexão do Wattímetro
static float W = -1;   // Potência instantânea
static float V = -1;   // Tensão
static float I = -1;   // Corrente
static float FP = -1;  // Fator de Potência
static float Freq = -1;// Frequência da rede
static float Wh = -1;  // Watt hora
#define RX2 25 // Padrão do Serial2 é porta 16, modificado para usar o GPIO que fisicamente está com o Wattimetro conectado.
#define TX2 33 // Padrão do Serial2 é porta 17, modificado para usar o GPIO que fisicamente está com o Wattimetro conectado.
/* Cria objeto HardwareSerial para ser adicionado como parâmetro na criação do objeto do Wattímetro.
ESP32 tem 3 UARTs/seriais em hardware e também aceita software serial (porém mais lento). UART0, usado com o chip USB do módulo, na comunicação e upload do firmware, GPIO 3 e 1.
UART1, GPIO RX 10, TX O9, reservado para comunicação com o chip de memória flash. E UART2, GPIOs RX 16 e TX 17, ficando livre para o uso.*/
HardwareSerial SerialController(2); 
PZEM004Tv30 pzem(&SerialController); // Cria objeto do Wattímetro, passando como parâmetro o objeto e portas do Hardware Serial.

// Variáveis dos sensores de pressão. Valores iniciados com negativo para sinalizar não uso/conexão dos sensores de pressão.
static float P1 = -1;
static float P2 = -2;
static unsigned int porta_P1 = 35;
static unsigned int porta_P2 = 34;


// Indicador de percentual de carga da bateria.
float percentual_bateria = 0;
float bateriaMin = 5000;
float bateriaMax = 0;
const int porta_bateria = 32;

unsigned long tempo_db_Pinos_temp = 300; // Tempo do debouncer das portas dos sensores de temperatura, em milisegundos.

unsigned int tipo_gas = 0; // Tipo de gás escolhido pelo operador. 


///////////////// Variáveis OTA //////////////////////////

static const uint32_t PORTA = 5000; // A porta que será utilizada (padrão 80).
// Inicia o servidor na porta selecionada
// Configurado para escutar na porta 5000, ao invés da 80 padrão.
static WebServer server(PORTA);

// Algumas informações que podem ser interessantes
const uint32_t chipID = (uint32_t)(ESP.getEfuseMac() >> 32); // Um ID exclusivo do Chip.
const String CHIP_ID = "<p> Chip ID: " + String(chipID) + "</p>"; // Montado para ser usado no HTML.
const String VERSION = "<p> Versão: 5.8 </p>"; // Exemplo de um controle de versão.

// // Informações interessantes agrupadas.
const String INFOS = VERSION + CHIP_ID;

// // Sinalizador de autorização do OTA.
boolean OTA_AUTORIZADO = false;



// // Páginas HTML utilizadas no procedimento OTA.
String verifica = "<!DOCTYPE html><html><head><title>ESP32 webOTA</title><meta charset='UTF-8'></head><body><h1>ESP32 webOTA</h1><h2>Digite a chave de verificação.<p>Clique em ok para continuar. . .</p></h2>" + INFOS + "<form method='POST' action='/avalia 'enctype='multipart/form-data'> <p><label>Autorização: </label><input type='text' name='autorizacao'></p><input type='submit' value='Ok'></form></body></html>";
String serverIndex = "<!DOCTYPE html><html><head><title>ESP32 webOTA</title><meta charset='UTF-8'></head><body><h1>ESP32 webOTA</h1><h2>Selecione o arquivo para a atualização e clique em atualizar.</h2>" + INFOS + "<form method='POST' action='/update' enctype='multipart/form-data'><p><input type='file' name='update'></p><p><input type='submit' value='Atualizar'></p></form></body></html>";
String Resultado_Ok = "<!DOCTYPE html><html><head><title>ESP32 webOTA</title><meta charset='UTF-8'></head><body><h1>ESP32 webOTA</h1><h2>Atualização bem sucedida!</h2>" + INFOS + "</body></html>";
String Resultado_Falha = "<!DOCTYPE html><html><head><title>ESP32 webOTA</title><meta charset='UTF-8'></head><body><h1>ESP32 webOTA</h1><h2>Falha durante a atualização. A versão anterior será recarregado.</h2>" + INFOS + "</body></html>";



#endif //VARIAVEIS_h__