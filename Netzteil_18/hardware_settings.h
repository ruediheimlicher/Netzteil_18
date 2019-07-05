/*
 * Hardware settings for the digital dc power supply
 * http://www.tuxgraphics.org/electronics/
 *
 * In this file you can:
 * - calibrate the ampere and voltmeter
 * - choose your hardware type: 22V 2.5A or 30V 2.0A
 *
 *   The ADC has a resolution of 11 bit = values from 0-2047
 */
#ifndef CAL_HW_H
#define CAL_HW_H


/* ================= uncomment this section for the model 22V 2.5A */

#define U_MAX 3220
#define I_MAX 250

#define U_START 1000 // 5V
#define I_START 3000

// Korrektur Instrument
#define I_KORR_0  1.34 // 30mA
#define I_KORR_1  1.34 // 100mA
#define I_KORR_2  1.33 // 300mA
#define I_KORR_3  1.18 // 1A
#define I_KORR_4  1.14 // 3A

// Korrektur digiString



#define P_OFFSET 1500 // Mitte fuer Potential 6.6V
#define P_OBERGRENZE 3000
#define P_PWM_OFFSET 0.44
#define P_REF     1.64 // Referenzspannung Po: Uref/2
#define P_AMPLI   2.73 // 82k/22k Verstärkung Diff-Op
#define P_KORR    1.4
#define P_INSTRUMENTKORR   1.0
# define TRANSISTOR_THRESHOLD 400
// internal adc ref voltage (should be 2.56V, can vary from uC to uC)
#define ADC_REF 3.26

// the divider R3/R4 [(R3+R4)/R4] (calibrate the voltmeter, lower value=higher output)
#define U_DIVIDER 9.8

// the shunt for current measurement, you can calibrate here the 
// amperemeter.
// 2* .33Ohm || 3W: 0.165:
#define I_RESISTOR 0.165

// short circuit protection limit (do not change unless you know what you do):
// TUX: 850=2.85A= (2.85A * 1023 * 0.75 / 2.56 ) Max I * Aufloesung * Shunt / Vref
// Imax=3A Ilim = (3*4095* .165/3.25 // unabh. von Shunt-Widerstand
// #define SH_CIR_PROT 850 // 3700 (3V max aus INA)

#define SH_CIR_PROT 3200 // vorher: 3700 (??)

#define U_KORR 1.21  //  Kalib Messinstrument

/* ================= uncomment this section for the model 30V 2.0A */

//#define U_MAX 300
//#define I_MAX 200

// internal adc ref voltage (should be 2.56V, can vary from uC to uC)
//#define ADC_REF 2.56

// the divider R3/R4 [(R3+R4)/R4], you can calibrate here the voltmeter:
//#define U_DIVIDER 13.24

// the shunt for current measurement, you can calibrate here the 
// amperemeter.
// 2*1.5Ohm 3W=0.75:
//#define I_RESISTOR 0.78

// short circuit protection limit (do not change unless you know what you do):
// 690=2.30A= (2.30A * 1023 * 0.75 / 2.56 )
//#define SH_CIR_PROT 690

// Eingang Teensy


// Drehgeber I
#define DREHGEBER0_A   4    // Pin A
#define DREHGEBER0_B   5    // Pin B

// Drehgebr U
#define DREHGEBER1_A   7    // Pin A
#define DREHGEBER1_B   8    // Pin B

// Potential
#define DREHGEBER2_A   3    // Pin A
#define DREHGEBER2_B   9    // Pin B

#define POTENTIAL_BIT   0     // gesetzt in ISR, Potential anzeigen bei Aenderung
#define POTENTIAL_ZEIT   2000    // Anzeigezeit

#define STROM_SETTING_BIT 1 // gesetzt in ISR, Strom-Setting zeigen bei Aenderung
#define STROM_SETTING_ZEIT 2000 // Anzeigezeit


#define AUSGANG_BIT  2   // gesetzt, wenn Ausgang ON

#define AUSGANG_RAMP_MAX   0xFF

#define CURRENTLIMIT_BIT 4    // gesetzt wenn currentcontrol > 0 
#define CURRENTLIMIT_TONE_BIT 5  // Warnton 
#define CURRENTLIMIT_ZEIT 1000 // Anzeigezeit der Einstellung


#define ON_OFF_0        4 // OFF  schalten
#define ON_OFF_1        5 // ON schalten

// Ausgang SR regA
#define LED_OUT   0
#define LED_I_WARRN  1

// Eingang SR

#define DREHGEBER0_ANZ_POS 4096          // Anzahl Schalterstellugen

#define DREHGEBER1_ANZ_POS 4096          // Anzahl Schalterstellugen

#define DREHGEBER2_ANZ_POS 4096          // Anzahl Schalterstellugen

// Taster

// Umschalter Bereich
#define BEREICH_UP      2
#define BEREICH_DOWN    3

// ON OFF 
#define OUT_ON          4
#define OUT_OFF         5

#define LCD_RESET       0
#define SAVE            1

// Switch Potential 
#define POT_ON          4
#define POT_OFF         5

// Ausgang Potential
#define POTENTIAL_OUT   22

#define SPI_CLK   13
#define SPI_MISO  12
#define SPI_MOSI  11
#define SPI_CS    10

#define TONE      21 // A7



#endif //CAL_HW_H

