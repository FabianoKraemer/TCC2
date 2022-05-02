#include "../include/Interrupcoes.h"
#include "../include/Debounce.h"

  static const unsigned long tempo_debounce = 300; // em milissegundos

  Debounce _deb1(tempo_debounce); // Debouncer para pino 1 sensor de temperatura
  Debounce _deb2(tempo_debounce); // Debouncer para pino 2 sensor de temperatura
  Debounce _deb3(tempo_debounce); // Debouncer para pino 3 sensor de temperatura
  Debounce _deb4(tempo_debounce); // Debouncer para pino 4 sensor de temperatura
  Debounce _deb5(tempo_debounce); // Debouncer para pino 5 sensor de temperatura
  Debounce _deb6(tempo_debounce); // Debouncer para pino 6 sensor de temperatura

  volatile static bool teste_vetor[6];

  portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

Interrupcoes::Interrupcoes(){
  Serial.println("construtor interrupcoes");
  // classes debounce
  
}

Interrupcoes::~Interrupcoes(){
    Serial.println("Destrutor objeto das interrupções");
}

void IRAM_ATTR Interrupcoes::interrupcao_DS18B20_1() {

  if (_deb1.debounce()) {// If interrupt is valid (debounced)
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

  if (_deb2.debounce()) {// If interrupt is valid (debounced)
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

  if (_deb1.debounce()) {// If interrupt is valid (debounced)
    portENTER_CRITICAL_ISR(&mux);
    if (digitalRead(DS18B20_3) == 0){
      teste_vetor[0] = true;
    } else if (digitalRead(DS18B20_3) == 1){
      teste_vetor[0] = false;
    }
    portEXIT_CRITICAL_ISR(&mux);
  }
}

void IRAM_ATTR Interrupcoes::interrupcao_DS18B20_4() {

  if (_deb4.debounce()) {// If interrupt is valid (debounced)
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

  if (_deb5.debounce()) {// If interrupt is valid (debounced)
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

  if (_deb6.debounce()) {// If interrupt is valid (debounced)
    portENTER_CRITICAL_ISR(&mux);
    if (digitalRead(DS18B20_6) == 0){
      teste_vetor[5] = true;
    } else if (digitalRead(DS18B20_6) == 1){
      teste_vetor[5] = false;
    }
    portEXIT_CRITICAL_ISR(&mux);
  }
}

// Ativa todas as interrupções dos pinos dos sensores de temperatura DS18B20
void Interrupcoes::AtivarInterrupcoes(){

  attachInterrupt(digitalPinToInterrupt(DS18B20_1), interrupcao_DS18B20_1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(DS18B20_2), interrupcao_DS18B20_2, CHANGE);
  //attachInterrupt(digitalPinToInterrupt(DS18B20_3), interrupcao_DS18B20_3, CHANGE); está com problema na porta física. Quando pino conectado, deveria fazer a tensão ir a 0, mas está em 1.9V
  attachInterrupt(digitalPinToInterrupt(DS18B20_4), interrupcao_DS18B20_4, CHANGE);
  attachInterrupt(digitalPinToInterrupt(DS18B20_5), interrupcao_DS18B20_5, CHANGE);
  attachInterrupt(digitalPinToInterrupt(DS18B20_6), interrupcao_DS18B20_6, CHANGE);
}

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

void Interrupcoes::imprimir(){

  Serial.print(teste_vetor[0]);
  Serial.print(" | ");
    Serial.print(teste_vetor[1]);
  Serial.print(" | ");
    Serial.print(teste_vetor[2]);
  Serial.print(" | ");
    Serial.print(teste_vetor[3]);
  Serial.print(" | ");
    Serial.print(teste_vetor[4]);
  Serial.print(" | ");
    Serial.println(teste_vetor[5]);

}