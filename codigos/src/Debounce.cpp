/*
 * Debouncer.cpp - Library for debouncing signals/pins.
 * Created by Arliones Hoeller Jr., November 23rd, 2021.
 * Released under the MIT License.
 */
#include "Arduino.h"
#include "Debounce.h"

//Debounce::Debounce(unsigned long dt)
//: _dt(dt), _last(0)
//{}

Debounce::Debounce(unsigned long dt) // variavel dt não sendo usada para nada por enquanto
: _last(0)
{}

bool Debounce::debounce(unsigned long tempo_db)
{
    unsigned long now = millis();
    unsigned long dt = now - _last;
    _last = now;
    if(dt >= tempo_db)
      Serial.println("ciclo debounce");
        return true;
    return false;
}

