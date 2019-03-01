//
// LocalLibrary.cpp
// Library C++ code
// ----------------------------------
// Developed with embedXcode+
// http://embedXcode.weebly.com
//
// Project 		Netzteil_18
//
// Created by 	Ruedi Heimlicher, 24.11.2018 10:37
// 				Ruedi Heimlicher
//
// Copyright 	© Ruedi Heimlicher, 2018
// Licence		Licence
//
// See 			LocalLibrary.h and ReadMe.txt for references
//


#include "LocalLibrary.h"

elapsedMillis blinkdelay;
// Code
void blink(uint8_t pin, uint8_t times, uint16_t ms, bool level)
{
   
    for (uint8_t i = 0; i < times; i++)
    {
       if (blinkdelay > 100)
       {
          if (digitalRead(pin) == 1)
          {
             digitalWrite(pin, LOW);
          }
          else
          {
             digitalWrite(pin, HIGH);
          }
          blinkdelay = 0;
       }
    }

}

void milliblink(uint8_t pin, uint8_t times, uint16_t ms)
{
   
}

uint8_t read_button(void)
{
   return 1;
}

void update_button(uint16_t *button_history)
{
   *button_history = *button_history << 1;
   *button_history |= read_button();
}
