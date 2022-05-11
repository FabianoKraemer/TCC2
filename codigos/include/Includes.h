#ifndef INCLUDES_h__
#define INCLUDES_h__

#include <Arduino.h> // Biblioteca padrão para se utilizar ao programar no PlatformIO, para usar o compilador da IDE do arduino por trás
//#include <Debounce.h>

// Bibliotecas "prontas" declaradas primeiro.
#include <WiFi.h> // Biblioteca para utilizar o WiFi da ESP32.
#include <PubSubClient.h> // Biblioteca do MQTT. Conecta, verifica servidor broken, publica e escuta/recebe dados do broker MQTT.
#include <ArduinoJson.h> // Json pra Arduino, serializa e desserializa.
#include <OneWire.h> // Protocolo da Dallas, usado para os NTCs digitais DS18B20.
#include <DallasTemperature.h> // Biblioteca do NTC digital DS18B20.
#include <SPI.h>
#include <PZEM004Tv30.h> // Biblioteca do Wattímetro, já com a compatibilidade pra software serial ou hardware serial.
#include "HardwareSerial.h" // Somente para a ESP32.

// Bibliotecas desenvolvidas declaradas depois, necessariamente nessa ordem.
#include "Variaveis.h"
#include "Interrupcoes.h" // Funções das interrupções dos pinos 1 a 6 dos sensores de temperatura DS18B20.
#include "Debounce.h"

#endif //INCLUDES_h__