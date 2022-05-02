/*
 * Debouncer.h - Library for debouncing signals/pins.
 * Created by Arliones Hoeller Jr., November 23rd, 2021.
 * Released under the MIT License.
 */
#ifndef DEBOUNCE_H__
#define DEBOUNCE_H__

#include "Arduino.h"

class Debounce
{
    public:
        Debounce(unsigned long dt); // variavel dt não sendo usada para nada por enquanto
        //Debounce();
        bool debounce(unsigned long tempo);

    private:
        unsigned long _last;
};

#endif //DEBOUNCE_H__