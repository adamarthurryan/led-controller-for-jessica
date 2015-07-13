#include <avr/io.h>
#include <stdlib.h>
#include "common.h"
/////
// All i/o is assumed to be on port B
// Pins are given as port B pins from 0 to 4
////

void writePin(byte pin, boolean val)
{
	byte mask=_BV(pin);//1<<pin;

	if (!val) PORTB &= ~mask;
	else PORTB |= mask;
}

boolean readPin(uint8_t pin)
{
	byte mask=_BV(pin);
	
	if (PINB & mask) return true;
	else return false;
}

long rnd(long howbig)
{
  if (howbig == 0){
    return 0;
  }
  return rand() % howbig;
}

long rndRange(long howsmall, long howbig)
{
  if(howsmall >= howbig){
    return howsmall;
  }
  long diff = howbig - howsmall;
  return rnd(diff) + howsmall;
}

