///
/// @file		LocalLibrary.h
/// @brief		Library header
///
/// @details	<#details#>
/// @n
/// @n @b		Project Netzteil_18
/// @n @a		Developed with [embedXcode+](http://embedXcode.weebly.com)
///
/// @author		Ruedi Heimlicher
/// @author		Ruedi Heimlicher
///
/// @date		24.11.2018 10:37
/// @version	<#version#>
///
/// @copyright	(c) Ruedi Heimlicher, 2018
/// @copyright	Licence
///
/// @see		ReadMe.txt for references
///
// !!! Help: http://bit.ly/2yUWVvd



#include "Arduino.h"
#include <ADC.h>
//#include <SPI.h>
#include "hardware_settings.h"

#ifndef Netzteil_18_LocalLibrary_h
#define Netzteil_18_LocalLibrary_h

///
/// @brief      Blink a LED
/// @details	LED attached to pin is turned on then off
/// @note       Total cycle duration = ms
/// @param      pin pin to which the LED is attached
/// @param      times number of times
/// @param      ms cycle duration in ms
/// @param		level level for HIGH, default=true=positive logic, false=negative logic
///
// !!! Help: http://bit.ly/2l1U25F
void blink(uint8_t pin, uint8_t times, uint16_t ms, bool level = true);

// debounce


void update_button(uint16_t *button_history);


#endif // Netzteil_18_LocalLibrary_h
