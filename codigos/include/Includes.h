#ifndef INCLUDES_h__
#define INCLUDES_h__

#include <Arduino.h> // Biblioteca padrão para se utilizar ao programar no PlatformIO, para usar o compilador da IDE do arduino por trás
//#include <Debounce.h>
#include "Interrupcoes.h" // Funções das interrupções dos pinos 1 a 6 dos sensores de temperatura DS18B20
#include "Temperatura.h" // 


////////////////////////////////////////////////////////////////////////////////////////////////////////
#include <WiFi.h> // Biblioteca para utilizar o WiFi da ESP32
#include <PubSubClient.h> // Biblioteca do MQTT. Conecta, verifica servidor broken, publica e escuta/recebe dados do broker MQTT
#include <ArduinoJson.h> // Json pra Arduino, serializa e desserializa
#include <Wire.h>
#include <SPI.h>

#include "Variaveis.h"
#include "Interrupcoes.h" // Funções das interrupções dos pinos 1 a 6 dos sensores de temperatura DS18B20
#include "Debounce.h"

#endif //INCLUDES_h__