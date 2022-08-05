#include "Interrupcoes.h"
#include "../include/Debounce.h"

//static const unsigned long tempo_debounce = 300; // em milissegundos

  unsigned long tempo_db = 300; // em milissegundos
  Debounce _deb(tempo_db); // Debouncer para pinos sensores de temperatura

  volatile static bool teste_vetor[6];

  portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

//Interrupcoes::Interrupcoes(){}

Interrupcoes::Interrupcoes(unsigned long tempo){ 
  this->tempo_debounce(tempo);
}

Interrupcoes::~Interrupcoes(){
    Serial.println("Destrutor objeto das interrupções");
}

void Interrupcoes::tempo_debounce(unsigned long teste){
  tempo_db = teste; 
}

void Interrupcoes::retorna_vetor(volatile bool estado_pinos[6]){
  for(int n = 0; n < 6; n++) estado_pinos[n] = teste_vetor[n];
} 

void IRAM_ATTR Interrupcoes::interrupcao_DS18B20_1() {

  if (_deb.debounce(tempo_db)) {// Entra no objeto debounce para realizar o debounced.
    portENTER_CRITICAL_ISR(&mux);
    if (digitalRead(DS18B20_1) == 0){
      teste_vetor[0] = true;
    } else if (digitalRead(DS18B20_1) == 1){
      teste_vetor[0] = false;
    }
    portEXIT_CRITICAL_ISR(&mux);
  }
}

void IRAM_ATTR Interrupcoes::interrupcao_DS18B20_2() {

  if (_deb.debounce(tempo_db)) {// Entra no objeto debounce para realizar o debounced.
    portENTER_CRITICAL_ISR(&mux);
    if (digitalRead(DS18B20_2) == 0){
      teste_vetor[1] = true;
    } else if (digitalRead(DS18B20_2) == 1){
      teste_vetor[1] = false;
    }
    portEXIT_CRITICAL_ISR(&mux);
  }
}

void IRAM_ATTR Interrupcoes::interrupcao_DS18B20_3() {

  Serial.println("Interrupt 3");

  if (_deb.debounce(tempo_db)) {// Entra no objeto debounce para realizar o debounced.
    portENTER_CRITICAL_ISR(&mux);
    if (digitalRead(DS18B20_3) == 0){
      teste_vetor[2] = true;
    } else if (digitalRead(DS18B20_3) == 1){
      teste_vetor[2] = false;
    }
    portEXIT_CRITICAL_ISR(&mux);
  }
}

void IRAM_ATTR Interrupcoes::interrupcao_DS18B20_4() {

  if (_deb.debounce(tempo_db)) {// Entra no objeto debounce para realizar o debounced.
    portENTER_CRITICAL_ISR(&mux);
    if (digitalRead(DS18B20_4) == 0){
      teste_vetor[3] = true;
    } else if (digitalRead(DS18B20_4) == 1){
      teste_vetor[3] = false;
    }
    portEXIT_CRITICAL_ISR(&mux);
  }
}

void IRAM_ATTR Interrupcoes::interrupcao_DS18B20_5() {

  if (_deb.debounce(tempo_db)) {// Entra no objeto debounce para realizar o debounced.
    portENTER_CRITICAL_ISR(&mux);
    if (digitalRead(DS18B20_5) == 0){
      teste_vetor[4] = true;
    } else if (digitalRead(DS18B20_5) == 1){
      teste_vetor[4] = false;
    }
    portEXIT_CRITICAL_ISR(&mux);
  }
}

void IRAM_ATTR Interrupcoes::interrupcao_DS18B20_6() {

  if (_deb.debounce(tempo_db)) {// Entra no objeto debounce para realizar o debounced.
    portENTER_CRITICAL_ISR(&mux);
    if (digitalRead(DS18B20_6) == 0){
      teste_vetor[5] = true;
    } else if (digitalRead(DS18B20_6) == 1){
      teste_vetor[5] = false;
    }
    portEXIT_CRITICAL_ISR(&mux);
  }
}

// Ativa todas as interrupções dos pinos dos sensores de temperatura DS18B20.
void Interrupcoes::ativar_interrupcoes(){

  attachInterrupt(digitalPinToInterrupt(DS18B20_1), interrupcao_DS18B20_1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(DS18B20_2), interrupcao_DS18B20_2, CHANGE);
  //attachInterrupt(digitalPinToInterrupt(DS18B20_3), interrupcao_DS18B20_3, CHANGE); está com problema na porta física. Quando pino conectado, deveria fazer a tensão ir a 0, mas está em 1.9V
  attachInterrupt(digitalPinToInterrupt(DS18B20_4), interrupcao_DS18B20_4, CHANGE);
  attachInterrupt(digitalPinToInterrupt(DS18B20_5), interrupcao_DS18B20_5, CHANGE);
  attachInterrupt(digitalPinToInterrupt(DS18B20_6), interrupcao_DS18B20_6, CHANGE);
}

// Verifica o estado das portas dos sensores DS18B20 e atualiza o valor no vetor. 
void Interrupcoes::atualizar_estado_portas(){

  if (digitalRead(DS18B20_1) == 0) teste_vetor[0] = true; 
  else if (digitalRead(DS18B20_1) == 1) teste_vetor[0] = false;

  if (digitalRead(DS18B20_2) == 0) teste_vetor[1] = true;
  else if (digitalRead(DS18B20_2) == 1)teste_vetor[1] = false;

  if (digitalRead(DS18B20_3) == 0)teste_vetor[2] = true;
  else if (digitalRead(DS18B20_3) == 1)teste_vetor[2] = false;

  if (digitalRead(DS18B20_4) == 0)teste_vetor[3] = true;
  else if (digitalRead(DS18B20_4) == 1)teste_vetor[3] = false;

  if (digitalRead(DS18B20_5) == 0)teste_vetor[4] = true;
  else if (digitalRead(DS18B20_5) == 1)teste_vetor[4] = false;

  if (digitalRead(DS18B20_6) == 0)teste_vetor[5] = true;
  else if (digitalRead(DS18B20_6) == 1)teste_vetor[5] = false;

}