/* vim: set sw=8 ts=8 si : */
/*********************************************
* Author: Guido Socher, Copyright: GPL 
*
* Digital analog conversion of channel ADC0 and ADC1 in
* free running mode. 
**********************************************/
#include "Arduino.h"
#include <avr/interrupt.h>
#include <avr/io.h>
#include <inttypes.h>
#include <stdlib.h>
//#include "dac.h"
//#include "uart.h"
#include "hardware_settings.h"

#include "analog.h"

extern int val;
extern volatile uint8_t inbuffer[64];
extern volatile uint8_t outbuffer[64];
//extern int ADC_U, ADC_I;
extern  int readPin;
extern int U_Pot;
extern volatile int16_t analog_result[2]; 

extern void prellcheck();
extern void debounce_ISR(void);
extern void debounce_switch(uint8_t);
extern volatile uint8_t loopcontrol;
extern volatile uint8_t ausgabestatus;
extern volatile uint16_t controllooperrcounterA;
extern volatile uint16_t controllooperrcounterB;
extern volatile uint16_t controllooperrcounterC;
extern volatile uint16_t controllooperrcounterD;

extern ADC_ERROR resultat;

//ADC::Sync_result result;

//debug LED:
// set output to VCC, red LED off
//#define LEDOFF PORTD|=(1<<PORTD0)
// set output to GND, red LED on
//#define LEDON PORTD&=~(1<<PORTD0)
// to test the state of the LED
//#define LEDISOFF PORTD&(1<<PORTD0)
static volatile uint8_t currentcontrol=1; // 0=voltage control, otherwise current control
// adc measurement results (11bit ADC):
extern float sinval;
volatile int16_t raw_analog_u_result[8]; 

// target_val is the value that is requested (control loop calibrates to this).
// We use the same units a the ADC produces.
static volatile int16_t target_val[2];  // datatype int is 16 bit

static volatile int16_t dac_val=800; // the current dac setting

static void control_loop(void);

// https://forum.pjrc.com/threads/25I_RESISTOR
//532 -ADC-library-update-now-with-support-for-Teensy-3-1

#define DEBOUNCECOUNT   20
volatile uint16_t debouncecounter = 0;

void init_analog(void) 
{
 	analog_result[0]=5000; // I
	analog_result[1]=2000;  // U
	target_val[0]=0; // initialize to zero, I, kein Strom
	target_val[1]=0; // initialize to 5000, U
   analogWriteFrequency(4, 375000);
   
   adc->setAveraging(4); // set number of averages 
   adc->setResolution(12); // set bits of resolution
   adc->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED);
   adc->setSamplingSpeed(ADC_SAMPLING_SPEED::MED_SPEED);
   adc->setReference(ADC_REFERENCE::REF_3V3, ADC_0);
   adc->setReference(ADC_REFERENCE::REF_3V3, ADC_1);
   
   adc->enableInterrupts(ADC_0);
   delay(100);
   
   
 }

void adc0_isr(void)  // ADC_CONVERSION_SPEED::HIGH_SPEED ADC_SAMPLING_SPEED::MED_SPEED: delay 12us
{
   digitalWriteFast(OSZIA,LOW);
   static uint8_t adc_counter = 0;
   result = adc->readSynchronizedSingle();
   analog_result[0] = (uint16_t)result.result_adc0; // I
   analog_result[1] = (uint16_t)result.result_adc1; // U
   
 //  adc->printError();
   
   adc->resetError();
   
   control_loop(); // 2us
   adc_counter++;
   if (adc_counter > 8)
   {
      prellcheck(); // 20 us

      adc_counter = 0;
   }
   //controllooperrcounterC = adc_counter;
   digitalWriteFast(OSZIA,HIGH);
}

void adc1_isr(void) 
{
   //analog_result[1] = adc->analogReadContinuous(ADC_1);
   
   //digitalWriteFast(LED_BUILTIN, !digitalReadFast(LED_BUILTIN));
   
}



// TUX
int16_t get_dacval(void) 
{
	return(dac_val);
}

uint8_t get_currentcontrol(void)
{
   return currentcontrol;
}

uint8_t is_current_limit(void) 
{
   // return 1 if current control loop active
   if (currentcontrol)
   {
      return(1);
   }
   return(0);
}

/* set the target adc value for the control loop
 * values for item: 1 = u, 0 = i, units must be of the same as the values 
 * from the dac.
 */
void set_target_adc_val(uint8_t item,int16_t val) 
{
   // here we can directly write to target_val 
   target_val[item]=val;
}

int16_t get_analogresult(uint8_t channel) 
{
	return(analog_result[channel]);
}

int16_t get_targetvalue(uint8_t channel) //
{
   return target_val[channel];
}

void set_targetvalue(uint8_t channel, uint16_t val)
{
   target_val[channel] = val;
}

void inc_targetvalue(uint8_t channel, uint16_t inc) // targetvalue incrementieren um inc
{
   if (target_val[channel] < (0xFDD))
   {
      target_val[channel] += inc;
   }
}

void dec_targetvalue(uint8_t channel, uint16_t dec) // targetvalue decrementieren um dec
{
   if (target_val[channel] > (0x0A))
   {
      target_val[channel] -= dec;
   }
}
     
     // the control loop changes the dac:

static void  control_loop()
{
   
   //digitalWriteFast(OSZIB,LOW);
   int16_t tmp;
   tmp=target_val[0] - analog_result[0]; // current diff
   if (tmp < 0) // current too high
   {
      controllooperrcounterA++;
      //digitalWriteFast(OSZIA,LOW);
      loopcontrol &= ~(1<<4);
      loopcontrol &= ~(1<<5);
      loopcontrol &= ~(1<<6);

      loopcontrol &= ~(1<<7);

      loopcontrol |= (1<<0);
      // ** current control:
      //
      // stay in current control if we are
      // close to the target. We never regulate
      // the difference down to zero otherweise
      // we would suddenly hop to voltage control
      // and then back to current control. Permanent
      // hopping would lead to oscillation and current
      // spikes.
      if (tmp>-20) 
      {
         tmp=0;
      }    
      //currentcontrol=64; // I control
      currentcontrol=10; // I control
      if (analog_result[1] > target_val[1])
      {
//         controllooperrcounterB++;
         loopcontrol |= (1<<1);
         // oh, voltage too high, get out of current control:
         tmp = -40;
         currentcontrol=0; // U control
         
      }
      //digitalWriteFast(OSZIA,HIGH);
   }
   else // voltage-control
   {
      loopcontrol &= ~(1<<0);
      loopcontrol &= ~(1<<1);
      // ** voltage control:
      //
      // if we are in current control then we can only go
      // down (tmp is negative). To increase the current
      // we come here to voltage control. We must slowly
      // count up.
      loopcontrol |= (1<<4);
      tmp = 1 + target_val[1]  - analog_result[1]; // voltage diff
      if (currentcontrol)
      {
       //  controllooperrcounterC++;
         loopcontrol |= (1<<5);
         currentcontrol--;
         //currentcontrol=0;
         // do not go up immediately after we came out of current control:
         if (tmp>0) 
         {
            tmp=0;
         }
      }
   }
   if (tmp> -3 && tmp<4)
   //if (tmp> -32 && tmp< 32)
   { // avoid LSB bouncing if we are close
      tmp=0;
   }
   if (tmp==0) 
   {
      controllooperrcounterD++;
      //digitalWriteFast(OSZIB,HIGH);
      return; // nothing to change
      
   }
   // put a cap on increase
   if (tmp>10)
   {
      tmp=1;
   }
   // put a cap on decrease
   if (tmp<-200)
   {
      tmp=-20;
   }
   else if (tmp<-1)
   {
      tmp=-1;
   }
   
   dac_val+=tmp;
   
   if (dac_val>0x0FFF) // 4095
   {
      dac_val=0x0FFF; //max, 12bit
   }
   if (dac_val<TRANSISTOR_THRESHOLD)
      
   {  // the output is zero below 800 due to transistor threshold
      dac_val=TRANSISTOR_THRESHOLD;
   }
   
   //analogWrite(A14, (int)dac_val);
   //return;
   if (ausgabestatus & (1<<AUSGANG_BIT))
   {
      analogWrite(A14, (int)dac_val);
   }
   else
   {
      analogWrite(A14,TRANSISTOR_THRESHOLD);
   }
   //digitalWriteFast(OSZIB,HIGH);
}
  
uint16_t readPot(uint8_t pin)
{
   return adc->analogRead(pin);
}


// convert adc values to voltage values, disp=10 is 1.0V
// disp_i_val is needed to calculate the offset for the voltage drop over
// the current measurement shunt, voltage measurement is 11bit
int16_t adc_u_to_disp(int16_t adcunits)
{
//   int16_t adcdrop;
//   adcdrop=disp_i_to_u_adc_offset(disp_i_val); 
//   if (adcunits < adcdrop)
   {
//      return(0);
   }
//   adcunits=adcunits-adcdrop;
   
   return((int16_t)((((float)adcunits * 10 /409.6)* ADC_REF * U_DIVIDER)+0.5)); // Umrechnen 12bit
}

// Convert an integer which is representing a float into a string.
// Our display is always 4 digits long (including one
// decimal point position). decimalpoint_pos defines
// after how many positions from the right we set the decimal point.
// The resulting string is fixed width and padded with leading space.
//
// decimalpoint_pos=2 sets the decimal point after 2 pos from the right:
// e.g 74 becomes "0.74"
// The integer should not be larger than 999.
// The integer must be a positive number.
// decimalpoint_pos can be 0, 1 or 2
void int_to_dispstr(uint16_t inum,char *outbuf,int8_t decimalpoint_pos){
   int8_t i,j;
   char chbuf[10];
   itoa(inum,chbuf,10); // convert integer to string
   i=strlen(chbuf);
   if (i>4) i=4; //overflow protection
   strcpy(outbuf,"    0"); //decimalpoint_pos==0
   if (decimalpoint_pos==1) strcpy(outbuf," 0.0");
   if (decimalpoint_pos==2) strcpy(outbuf,"0.00");
   j=5;
   while(i){
      outbuf[j-1]=chbuf[i-1];
      i--;
      j--;
      if (j==4-decimalpoint_pos){
         // jump over the pre-set dot
         j--;
      }
   }
}





/* the following function will be called when analog conversion is done.
 * It will be called every 13th cycle of the converson clock. At 8Mhz
 * and a ADPS factor of 64 this is: ((8000 kHz/ 64) / 13)= 9.6KHz intervalls
 *
 * We do 4 fold oversampling as explained in atmel doc AVR121.
 */

/*
 
ISR(ADC_vect)
{
	uint8_t i=0;
	uint8_t adlow;
	int16_t currentadc;
	static uint8_t channel=0; 
	static uint8_t chpos=0;
	// raw 10bit values:
        static int16_t raw_analog_u_result[8];  
        int16_t new_analog_u_result=0;
	adlow=ADCL; // read low first !! 
	currentadc=(ADCH<<8)|adlow;
	// toggel the channel between 0 an 1. This will however
	// not effect the next conversion as that one is already
	// ongoing.
	channel^=1;
	// 2.56V ref
	ADMUX=(1<<REFS1)|(1<<REFS0)|channel;
	// channel=1 = U, channel=0 = I
	if (channel==1) // Spannung messen
   {
		raw_analog_u_result[chpos]=currentadc;
		//
		// we do 4 bit oversampling to get 11bit ADC resolution
		chpos=(chpos+1)%4; // rotate over 4 
		//analog_result[1]=0;
		while(i<4)
      {
			new_analog_u_result+=raw_analog_u_result[i];
			i++;
		}
		new_analog_u_result=new_analog_u_result>>1; // 11bit
		// mean value:
		analog_result[1]=(new_analog_u_result+analog_result[1])/2; 
	}
   else // channel==0, Strom messen
   {
		analog_result[0]=currentadc; // 10bit
	}
   
	// short circuit protection does not use the over sampling results
	// for speed reasons.
	// short circuit protection, current is 10bit ADC
	if (channel==0 && currentadc > SH_CIR_PROT_3)
   {
		dac_val=400;
		//dac(dac_val);
		currentcontrol=20;
		return;
	}
        //
	if (channel==1)
   {
		// only after full measurement cycle
		control_loop();
	}
   else
   {
		uart_poll_getchar_isr();
	}
	// end of interrupt handler
}
*/
