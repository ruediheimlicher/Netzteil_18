








///
/// @mainpage	Netzteil_18
///
/// @details	Description of the project
/// @n
/// @n
/// @n @a		Developed with [embedXcode+](http://embedXcode.weebly.com)
///
/// @author		Ruedi Heimlicher
/// @author		Ruedi Heimlicher
/// @date		24.11.2018 10:37
/// @version	<#version#>
///
/// @copyright	(c) Ruedi Heimlicher, 2018
/// @copyright	Licence
///
/// @see		ReadMe.txt for references
///


///
/// @file		Netzteil_18.ino
/// @brief		Main sketch
///
/// @details	<#details#>
/// @n @a		Developed with [embedXcode+](http://embedXcode.weebly.com)
///
/// @author		Ruedi Heimlicher
/// @date		24.11.2018 10:37
/// @version	<#version#>
///
/// @copyright	(c) Ruedi Heimlicher, 2018
/// @copyright	Licence
///
/// @see		ReadMe.txt for references
/// @n
///


#include "Arduino.h"

// bibs muessen auch im MAkefile angegeben werden:
// APP_LIBS_LIST = ADC SPI
// https://forum.pjrc.com/threads/43239-Problems-using-Xcode-with-embedXcode
#include <ADC.h>
#include <SPI.h>
#include "gpio_MCP23S17.h"
// Set parameters


// Include application, user and local libraries
#include "LocalLibrary.h"
#include "lcd.h"
#include "analog.h"
#include "hardware_settings.h"
// Define structures and classes

ADC *adc = new ADC(); // adc object
IntervalTimer debouncetimer;
// Define variables and constants
///
/// @brief      Name of the LED
///

uint8_t myLED;
#define USB_DATENBREITE 64
int8_t r;
volatile uint8_t inbuffer[USB_DATENBREITE]={};
volatile uint8_t outbuffer[USB_DATENBREITE]={};
elapsedMillis sinceblink;
elapsedMillis sinceusb;

int period = 1000;
unsigned long time_now = 0;
int val;
int U_soll;
int I_soll;
int U_Pot;

uint16_t input =0;

volatile int16_t analog_result[2]; 


int  readPin = A9; // ADC0

int readPin2 = A2; // ADC0 or ADC1
int readPin3 = A3; // ADC1

ADC::Sync_result result;

// sine wave
float phase = 0.0;
float twopi = 3.14159 * 2;
elapsedMicros usec = 0;
float sinval = 0;

char buf[21];

// Drehgeber
uint8_t drehgeber_dir = 0; //Drehrichtung
volatile uint16_t drehgeber_count = 0;
uint8_t timercounter=0;
uint16_t uptaste_history = 0;
uint16_t downtaste_history = 0;

volatile uint8_t tipptastenstatus = 0;

typedef struct
{
   uint8_t pin;
   uint16_t tasten_history;
   uint8_t pressed;
}tastenstatus;

tastenstatus tastenstatusarray[8] = {}; 

uint8_t tastenbitstatus = 0; // bits fuer tasten

gpio_MCP23S17 mcp(10,0x20);//instance (address A0,A1,A2 tied to +)
uint8_t regA = 0x01;
uint8_t regB = 0;


// Prototypes

unsigned int packetCount = 0;
// Utilities


// Functions


///
/// @brief      Setup
/// @details	Define the pin the LED is connected to
///
// Add setup code

//void yield(void) {}

uint8_t readTaste(uint8_t taste)
{
   return (digitalReadFast(taste) == 0);
}

void update_button(uint8_t taste, uint16_t *button_history)
{
   *button_history = *button_history << 1;
   *button_history |= readTaste(taste);
}

uint8_t is_button_pressed(uint8_t button_history)
{
   return (button_history == 0b01111111);
}
uint8_t is_button_released(uint8_t button_history){
   return (button_history == 0b10000000);
}

uint8_t test_for_press_only(uint8_t pin)
{   
   static uint16_t button_history = 0;
   uint8_t pressed = 0;    
   
   button_history = button_history << 1;
   button_history |= readTaste(pin);
   if ((button_history & 0b11000111) == 0b00000111)
   { 
      pressed = 1;
      button_history = 0b11111111;
   }
   return pressed;
}

uint8_t checktasten(void)
{
   uint8_t count = 0; // Anzahl aktivierter Tasten
   uint8_t i=0;
   while (i<8)
   {
      uint8_t pressed = 0;
      if (tastenstatusarray[i].pin < 0xFF)
      {
         count++;
         tastenstatusarray[i].tasten_history = tastenstatusarray[i].tasten_history << 1;
         tastenstatusarray[i].tasten_history |= readTaste(tastenstatusarray[i].pin);
         if ((tastenstatusarray[i].tasten_history & 0b11000111) == 0b00000111)
         {
            pressed = 1;
            tipptastenstatus |= (1<<i);
            tastenbitstatus |= (1<<i);
            tastenstatusarray[i].tasten_history = 0b11111111;
            tastenstatusarray[i].pressed = pressed;
         }
         
      }// i < 0xFF
      
      i++;
   }
   // tastenstatusarray
   return tipptastenstatus ;
}



void debounce_ISR(void)
{
   digitalWriteFast(OSZIA,LOW);
//   update_button(5,&uptaste_history);
   
 //  if (test_for_press_only(5))
   uint8_t old_tipptastenstatus = tipptastenstatus;
   tipptastenstatus = checktasten();
    digitalWriteFast(OSZIA,HIGH);
   //digitalWriteFast(OSZIA,HIGH);
}

void drehgeber_ISR(void)
{
   //digitalWriteFast(OSZIA,LOW);
   if (digitalReadFast(DREHGEBER_B) == 0) //  Impuls B ist 0,  kommt spaeter: RI A
   {
      if (drehgeber_count < DREHGEBER_ANZ_POS - 10)
      {
         drehgeber_dir = 0;
         drehgeber_count += 10;
         
      }
      inc_targetvalue(1, 16);
   }
   else // //  Impuls B ist 1,  war frueher:RI B
   {
      if (drehgeber_count > 10)
      {
         drehgeber_dir = 1;
         drehgeber_count -= 10;
      }
      dec_targetvalue(1, 16);
   }
   //digitalWriteFast(OSZIA,HIGH);
}



void setup()
{
   Serial.begin(38400);
   // !!! Help: http://bit.ly/2l2pqAL
   pinMode(OSZIA,OUTPUT);
   digitalWriteFast(OSZIA,HIGH);
   myLED = 8;
   // LCD
   pinMode(LCD_RSDS_PIN, OUTPUT);
   pinMode(LCD_ENABLE_PIN, OUTPUT);
   pinMode(LCD_CLOCK_PIN, OUTPUT);
   digitalWrite(LCD_RSDS_PIN,1);
   digitalWrite(LCD_ENABLE_PIN,1);
   digitalWrite(LCD_CLOCK_PIN,1);
   pinMode(myLED, OUTPUT);
   analogReadResolution(12);
   analogWriteResolution(12);
   
   // example
   pinMode(readPin, INPUT);
   pinMode(readPin2, INPUT);
   
   pinMode(readPin3, INPUT);
   
   //pinMode(9, INPUT);
   
   // Drehgeber
   pinMode(DREHGEBER_A,INPUT); // Kanal A
   attachInterrupt(DREHGEBER_A, drehgeber_ISR, FALLING); //
   pinMode(DREHGEBER_B,INPUT); // Kanal B
   
   // debounce
   
   for (uint8_t i= 0;i<8;i++)
   {
      tastenstatusarray[i].tasten_history = 0;
      tastenstatusarray[i].pressed = 0;
      tastenstatusarray[i].pin = 0xFF;
   }
   tastenstatusarray[0].pin = 5;
   pinMode(5,INPUT); // Taste
   tastenstatusarray[1].pin = 6;
   pinMode(6,INPUT); // Taste
   //
   
   // SPI
   
   //pinMode(SPI_CLK,OUTPUT);
   
   lcd_initialize(LCD_FUNCTION_8x2, LCD_CMD_ENTRY_INC, LCD_CMD_ON);
   
   _delay_ms(100);
   lcd_puts("Teensy");
   
   init_analog(); 
   
   pinMode(SPI_CS,OUTPUT);
   mcp.begin();
   /*
    • PortA registeraddresses range from 00h–0Ah
    • PortB registeraddresses range from 10h–1Ah
    PortA output, PortB input: Direction 1 output: direction 0
    0x0F: A: out B: in
    */
   //mcp.gpioPinMode(OUTPUT);
   mcp.gpioPinMode(0x00FF);
   mcp.portPullup(0x00FF);
   mcp.gpioPort(0xFFFF);
   
   debouncetimer.begin(debounce_ISR,1000);
  
   
   
}
///
/// @brief      Loop
/// @details	Call blink
///
// Add loop code
void loop()
{

   int n;
   if (sinceblink > 1000) 
   {  
      // sine wave
      sinval = sin(phase) * 400.0 + 800.0;
      phase = phase + 0.2;
      if (phase >= twopi)
      {
         phase = 0;
      }
  //    set_target_adc_val(1,sinval);
  //    Serial.printf("sine wave: phase: \t%2.2f\t sin: \t%2.2f \t",phase, sin(phase));
   //   Serial.println(val);
      //mcp.gpioPort(0xFFFF);
      // end sine wave
      if (digitalRead(myLED) == 1)
      {
         //Serial.printf("LED ON\n");
         digitalWriteFast(myLED, 0);
         digitalWriteFast(OSZIA,LOW);
         //digitalWriteFast(SPI_CLK,LOW);
         //mcp.gpioDigitalWrite(1,LOW);
         //mcp.gpioDigitalWrite((regA ),LOW);
         //mcp.gpioPort(0xFFFF);
         /*
         for (int i=0;i<16;i++)
         {
            mcp.gpioDigitalWrite(i,LOW);
           // delay(150);
         }
          */

      }
      else
      {
         //Serial.printf("LED OFF\n");
         digitalWriteFast(myLED, 1);
         digitalWriteFast(OSZIA,HIGH);
         //digitalWriteFast(SPI_CLK,HIGH);
         //mcp.gpioDigitalWrite((regA ),HIGH);
         
           /*
         for (int i=0;i<16;i++)
         {
            mcp.gpioDigitalWrite(i,HIGH);
         }
          */

      }
      
      
      // https://forum.arduino.cc/index.php?topic=353678.0
 //     SPI.beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE0));
      //mcp.gpioPort((regA << 8));
      mcp.gpioWritePortA(regA);
      //uint16_t portdata = mcp.exchangeGpioPort((regA << 8));
//      
      if (regA < 0x80)
      {
         regA <<= 1;
      }
      else
      {
         regA = 1;
      }
   
   //   uint8_t portBdata  = (mcp.readGpioPort() & 0x00FF);
      uint8_t portBdata = mcp.gpioReadPortB(); 
//      SPI.endTransaction();
      //regA &= 0x0F;
      lcd_gotoxy(9,1); 
      // PORTB
      //lcd_puthex(portdata & 0x00FF);
      lcd_puthex(portBdata);
      //lcd_putc(' ');
      //lcd_puthex(portdata & 0xFF00);
      
      //digitalWriteFast(SPI_CLK,(!(digitalReadFast(SPI_CLK))));
      sinceblink = 0;
      
     //mcp.gpioRegisterWriteByte(0,(regA << 8 ),false);
      
      
      
      //lcd_gotoxy(10,0);
      //lcd_puts("pot: ");
      
      //lcd_putint12(U_Pot);
      //lcd_gotoxy(0,1);
      //lcd_puts("adc0:");
      //lcd_putint12(analog_result[0]);
      //lcd_putc(' ');
      
       //lcd_gotoxy(0,2);
      //lcd_puts("adc1:");
      //lcd_putint12(analog_result[1]);
      //lcd_putc(' ');
      
      //lcd_putint12(analog_result[2]);
      
      outbuffer[0] = 0;
      outbuffer[1] = (U_Pot & 0xFF00) >> 8;
      outbuffer[2] = U_Pot & 0x00FF;
      
      lcd_gotoxy(0,0);
      lcd_puts("U: ");
      uint16_t U = get_analogresult(1);
      lcd_putint12(U);
      lcd_putc(' ');
      uint16_t Udisp = adc_u_to_disp(U);
      int_to_dispstr(Udisp,buf,1);
      lcd_putint12(adc_u_to_disp(U));
      lcd_putc(' ');
      lcd_puts(buf);
      // "sincePrint" auto-increases
      lcd_gotoxy(0,0);
      //lcd_puts("adc: ");
      int out0 = outbuffer[2];
      // val = analogRead(0);
      //val = target_val[0];
      //lcd_putint12(val);
      //lcd_putint12(out0);

      input = inbuffer[4];
      input <<= 8;
      input += inbuffer[5];
  //    Serial.print("analog 0 is: ");
   //   Serial.println(val);
      
      /*
      //Serial.println("Print every 2.5 seconds");
      Serial.printf("usb_rawhid_recv r: %x\t ",r);
      
      uint8_t i=0;
      Serial.printf("inbuffer:\t");
      while (i<10)
      {         
         //        Serial.printf("%x\t",inbuffer[i]);
         i++;
      }
      */
     
      
      
   //   Serial.printf("\n");
      
      
      
      //blink(myLED, 1, 150);
      //
      if (digitalRead(myLED) == 1)
      {
         //digitalWriteFast(myLED, LOW);
      }
      else
      {
         //digitalWriteFast(myLED, HIGH);
      }
      
      //`
      adc->printError();
      adc->resetError();
      
      n = RawHID.send((void*)outbuffer, 100);
     
      if (n > 0) 
      {
         Serial.print(F("Transmit packet "));
      } 
      else 
      {
         Serial.println(F("Unable to transmit packet"));
      }
      
      int outcontrol = (outbuffer[1]<<8) + outbuffer[2];
      Serial.printf("outH: %02X outL: %02X wert: \t%d\n",outbuffer[1],outbuffer[2],outcontrol);
      
      packetCount = packetCount + 1;
      /*
      lcd_gotoxy(0,1);
      lcd_puts("in: ");
      lcd_putc('L');
      lcd_puthex(inbuffer[4]);
      lcd_putc(' ');
      lcd_putc('H');
      lcd_puthex(inbuffer[5]);
      lcd_putc(' ');
      lcd_puts("W: ");
       */
      /*
      int incontrol = (inbuffer[4]<<8) + inbuffer[5];
      //set_target_U(incontrol);
      //lcd_putint12(incontrol);
      Serial.printf("input wert:\t %d\n",incontrol);
      analogWrite(9,incontrol);
       
       
       */
      
#pragma mark debounce
      lcd_gotoxy(0,1);
      lcd_putc('d');
      lcd_putint12(drehgeber_count);
      lcd_putc('t');
      lcd_puthex(tipptastenstatus);
      
      // Taste[0]
      if (tastenstatusarray[0].pressed) // Taste gedrueckt U+
      {
         lcd_gotoxy(14,1);
         lcd_putc('A');
         inc_targetvalue(1, 8);
         tastenstatusarray[0].pressed = 0;
      }
      else
      {
         lcd_gotoxy(14,1);
         lcd_putc(' ');
      }
      
 
      // Taste[1]
      if (tastenstatusarray[1].pressed) // Taste gedrueckt U-
      {
         lcd_gotoxy(15,1);
         lcd_putc('B');
         dec_targetvalue(1, 8);
         tastenstatusarray[1].pressed = 0;
      }
      else
      {
         lcd_gotoxy(15,1);
         lcd_putc(' ');
      }
      

      lcd_gotoxy(18,1);
      lcd_puthex(tastenbitstatus);
      tastenbitstatus = 0;
      
      //lcd_putint1(drehgeber_dir);
     // if ((timercounter % 1000) == 0)
      {
         lcd_putc(' ');
         
      }
      
      lcd_gotoxy(18, 0);
      //lcd_puts("cc:");
      lcd_putint2(get_currentcontrol());
      
  
      lcd_gotoxy(0,2);
      lcd_putc('I');
      lcd_putc(' ');
      lcd_puts(" ");
      //lcd_putint12(get_dacval());
      lcd_puts("    ");
      lcd_putc(' ');
      lcd_puts("a");
      lcd_putint12(get_analogresult(0));
      lcd_putc(' ');
      lcd_puts("t");
      lcd_putint12(get_targetvalue(0));

      
     // lcd_putc(' ');
      lcd_gotoxy(0,3);
      lcd_putc('U');
      lcd_putc(' ');
      lcd_puts("d");
      lcd_putint12(get_dacval());

      lcd_putc(' ');
      lcd_puts("a");
      lcd_putint12(get_analogresult(1));
      lcd_putc(' ');
      lcd_puts("t");
      lcd_putint12(get_targetvalue(1));
      
      
     
   } // if sinceblink
   
   if (sinceusb > 100)
   {
      //digitalWriteFast(OSZIA,LOW);
      r = RawHID.recv((void*)inbuffer, 0);
      //digitalWriteFast(OSZIA,HIGH);
      if (r > 0)
      {
         //Serial.println("Print every 2.5 seconds");
         //Serial.printf("usb_rawhid_recv: %x\n",r);
         int in_Usoll = (inbuffer[4]<<8) + inbuffer[5];
         
         //lcd_putint12(in_Usoll);
         Serial.printf("in_Usoll:\t %d\n",in_Usoll);
         analogWrite(9,in_Usoll);
         
         set_target_adc_val(1,in_Usoll);
         
          int in_Isoll = (inbuffer[6]<<8) + inbuffer[7];
         
         set_target_adc_val(0,in_Isoll);
         
         uint8_t i=0;
         //Serial.printf("inbuffer:\t");
         while (i<10)
         {         
            //Serial.printf("%x\t",inbuffer[i]);
            i++;
         }
          
      }
      
   }
   
}
