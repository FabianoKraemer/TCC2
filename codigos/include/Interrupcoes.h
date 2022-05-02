#include "Arduino.h"
#include "../include/Debounce.h"

class Interrupcoes{

private:

    static const int8_t DS18B20_1 = 16;      // GPIO sensor porta 1 P2 do DS18B20
    static const int8_t DS18B20_2 = 17;      // GPIO sensor porta 2 P2 do DS18B20
    static const int8_t DS18B20_3 =  5;      // GPIO sensor porta 3 P2 do DS18B20
    static const int8_t DS18B20_4 = 18;      // GPIO sensor porta 4 P2 do DS18B20
    static const int8_t DS18B20_5 = 19;      // GPIO sensor porta 5 P2 do DS18B20
    static const int8_t DS18B20_6 = 21;      // GPIO sensor porta 6 P2 do DS18B20

public:

    Interrupcoes();  // Construtor
    ~Interrupcoes(); // Destrutor

    static void AtivarInterrupcoes(); // Ativa todas as interrupções dos pinos P2 dos sensores de temperatura DS18B20

    void imprimir(); // imprimi o estado das 6 portas dos DS18B20

    static void atualizar_estado_portas(); // le o estado das 6 portas e atualiza o vetor booleano do estado delas, para verificar se tem algo conectada.

    //volatile static bool teste_vetor[6]; // Variavel em vetor booleano para identificar quais portas dos pinos P2 dos sensores de temperatura estão com um sensor conectado

    // métodos das interrupções dos pinos dos sensores de temperatura DS18B20, precisam ser declarados static devido o uso do IRAM_ATTR, característica de interrupção do ESP32
    static void interrupcao_DS18B20_1(); 
    static void interrupcao_DS18B20_2();
    static void interrupcao_DS18B20_3();
    static void interrupcao_DS18B20_4();
    static void interrupcao_DS18B20_5();
    static void interrupcao_DS18B20_6();

};