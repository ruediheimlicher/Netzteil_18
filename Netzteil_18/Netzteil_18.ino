








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
ADC::Sync_result result;
IntervalTimer prelltimer;
// Define variables and constants
///
///

uint8_t loopLED;
#define USB_DATENBREITE 64
int8_t r;
volatile uint8_t inbuffer[USB_DATENBREITE]={};
volatile uint8_t outbuffer[USB_DATENBREITE]={};
elapsedMillis sinceblink;
elapsedMillis sincelcd;
elapsedMillis sinceusb;

elapsedMillis sincepot; // Zeitdauer der Anzeige des Potentialwertes



elapsedMillis since_U;  // Zeit Drehgeberimpuls U
elapsedMillis since_I;  // Zeit Drehgeberimpuls I
elapsedMillis since_P;  // Zeit Drehgeberimpuls Potential

elapsedMillis since_WARN; // Zeit fuer Warnton



int period = 1000;
unsigned long time_now = 0;
int val;
int U_soll;
int I_soll;
int U_Pot; 
//int U_soll;
//int in_Isoll;

#define MAX_U     3200
#define MAX_I     3200
#define MAX_POT   3200
uint16_t input =0;

volatile int16_t analog_result[2]; 

volatile int16_t potential = P_OFFSET;
volatile int16_t oldpotential = P_OFFSET;


//ADC::Sync_result result;

// sine wave
float phase = 0.0;
float twopi = 3.14159 * 2;
elapsedMicros usec = 0;
float sinval = 0;

char buf[21];

// Drehgeber
uint8_t drehgeber0_dir = 0; //Drehrichtung
volatile uint16_t drehgeber0_count = 0;

uint8_t drehgeber1_dir = 0; //Drehrichtung
volatile uint16_t drehgeber1_count = 0;

uint8_t drehgeber2_dir = 0; //Drehrichtung
volatile uint16_t drehgeber2_count = 2000;

volatile uint8_t ausgabestatus = 0; // Anzeige Instrumente: Potential oder Ausgangsspannung, currentlimit oder current

volatile uint16_t U_instrumentcounter = 0; // Counter fuer U-Anzeigezeit
volatile uint16_t I_instrumentcounter = 0; // Counter fuer I-Anzeigezeit

volatile uint16_t ausgangsspannung = 0;

volatile uint8_t bereichpos = 1;

ADC_ERROR resultat;

uint8_t timercounter=0;
uint16_t uptaste_history = 0;
uint16_t downtaste_history = 0;

volatile uint8_t tipptastenstatus = 0;

volatile uint8_t SPItastenstatus = 0;
volatile uint8_t SPIcheck=0;

//volatile uint8_t anzeigestatus = 0;
#define potential_on  0


#define I_OUT  23 // A9
#define U_OUT  6

volatile uint16_t controllooperrcounterA=0;
volatile uint16_t controllooperrcounterB=0;
volatile uint16_t controllooperrcounterC=0;

volatile uint16_t controllooperrcounterD=0;

uint8_t ausgangcheck=0;

uint8_t ausgangramp = 0; // weiches einschalten
uint16_t lastausgangspannung = 0;


volatile uint8_t loopcontrol=0;

volatile uint8_t prellcounter = 0;

volatile uint8_t currentstatus=0; // status currentcontrol
#define CURRENTWARNUNG              1 // Warnton einschalten
#define CURRENTWARNUNG_ANZAHL       3 // Anz Warntoene
#define CURRENTWARNUNG_DELAY       1000 // Abstand Warntoene
volatile uint8_t currentcontrol_level=0; // currentcontrol von controlloop
volatile uint16_t currentcontrolcounter = 0; // Counter fuer Warnton

float I_korr_array[5] = {I_KORR_0,I_KORR_1,I_KORR_2,I_KORR_3,I_KORR_4};
char* I_einheit[] = {"mA","A"};

typedef struct
{
   uint8_t pin;
   uint16_t tasten_history;
   uint8_t pressed;
   long lastDebounceTime;
}tastenstatus;

//long lastDebounceTime = 0;  // the last time the output pin was toggled
long debounceDelay = 20;    // the debounce time; increase if the output flickers

tastenstatus tastenstatusarray[8] = {}; 

uint8_t tastenbitstatus = 0; // bits fuer tasten

volatile uint8_t tastencode = 0;

// http://www.ganssle.com/debouncing-pt2.htm
#define MAX_CHECKS 8
volatile uint8_t last_debounced_state = 0;
volatile uint8_t debounced_state = 0;
volatile uint8_t state[MAX_CHECKS] = {0};

volatile uint8_t debounceindex = 0;
void debounce_switch(uint8_t port)
{
   uint8_t i,j;
   state[debounceindex] = port;
   ++debounceindex;
   j = 0xFF;
   for (i=0;i<MAX_CHECKS;i++)
   {
      j=j & state[i];
   }
   debounced_state = j;
   
   if (debounceindex >= MAX_CHECKS)
   {
      debounceindex = 0;
   }
   
}

// end ganssle


gpio_MCP23S17 mcp0(10,0x20);//instance 0 (address A0,A1,A2 tied to 0)
//gpio_MCP23S17 mcp1(10,0x21);//instance 1 (address A0 to +, A1,A2 tied to 0)
uint8_t regA = 0x00;
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
   uint8_t tastencode = 0;
   while (i<8)
   {
      uint8_t pressed = 0;
      if (tastenstatusarray[i].pin < 0xFF)
      {
         count++;
         tastenstatusarray[i].tasten_history = (tastenstatusarray[i].tasten_history << 1);
         tastenstatusarray[i].tasten_history |= readTaste(tastenstatusarray[i].pin); // pin-nummer von $element i
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
   //return tastencode;
   return tipptastenstatus ;
}

uint8_t checkSPItasten() // MCP23S17 abrufen // Takt ca. 300us
{
   uint8_t count = 0; // Anzahl aktivierter Tasten
   uint8_t i=0;
   //uint8_t tastencode = 0;
   uint8_t check=0;
   //digitalWriteFast(OSZIB,LOW); // 
   //tastencode = 0xFF - mcp0.gpioReadPortB(); // 8 us active taste ist LO > invertieren
     
   //digitalWriteFast(OSZIB,HIGH);
   //controllooperrcounterC++;
   //controllooperrcounterD = tastencode;
   //digitalWriteFast(OSZIB,LOW);
   while (i<8) // 1us
   {
      uint8_t pressed = 0;
      if (tastenstatusarray[i].pin < 0xFF)
      {
         count++;
         tastenstatusarray[i].tasten_history = tastenstatusarray[i].tasten_history << 1;
      
         uint8_t pinnummer = tastenstatusarray[i].pin;
         tastenstatusarray[i].tasten_history |= ((tastencode & (1<<pinnummer)) > 0);
         if ((tastenstatusarray[i].tasten_history & 0b11000111) == 0b00000111)
         {
            pressed = 1;
            SPItastenstatus |= (1<<i);
            tastenbitstatus |= (1<<i);
            tastenstatusarray[i].tasten_history = 0b11111111;
            tastenstatusarray[i].pressed = pressed;
         }
      }// i < 0xFF
      i++;
   }
   //controllooperrcounterD = count;
   //digitalWriteFast(OSZIB,HIGH); // 9us
   return SPItastenstatus ;
}


void debounce_ISR(void)
{
   //  digitalWriteFast(OSZIA,LOW);
   uint8_t old_tipptastenstatus = tipptastenstatus;
   //tipptastenstatus = checktasten();
   
   //SPIcheck  = checkSPItasten(); 
   //debounced_state = checkSPItasten(); 
   //  digitalWriteFast(OSZIA,HIGH);
   
   analogWrite(I_OUT, get_analogresult(0));
   
   analogWrite(U_OUT, get_analogresult(1) * U_KORR); 
   analogWrite(POTENTIAL_OUT,(P_OBERGRENZE - potential) * P_KORR); // Potentialausgang an Buchse ausgeben
   
   //   analogWrite(U_OUT, 1000);
}


void prellcheck(void) // 30us debounce mit ganssle-funktion
{
   //digitalWriteFast(OSZIB,LOW);
      
    
   // MCP lesen
   
   regB = bereichpos;
//   digitalWriteFast(OSZIB,LOW);
   uint8_t regBB = (regB & 0x07)<< 5;
   mcp0.gpioWritePortA((regA | regBB)); // output
  //digitalWriteFast(OSZIB,HIGH);
   tastencode = 0xFF-mcp0.gpioReadPortB(); // input 8us . PORTB invertieren, 1 ist aktiv
    //digitalWriteFast(OSZIB,HIGH);
   
   //controllooperrcounterA =tastencode; //(1<<tastenstatusarray[2].pin) | (1<<tastenstatusarray[5].pin);
   
   //digitalWriteFast(OSZIB,LOW);
   
   //debounce_switch(tastencode); // 6us
   debounced_state = checkSPItasten();
   
   //mcp0.gpioWritePortA((regA | regBB)); // output
   
   //controllooperrcounterB = debounced_state;
      
   // test: Ausgang immer ON
   /*
   ausgangsspannung = get_targetvalue(1);
   ausgabestatus |= (1<<AUSGANG_BIT);
   ausgabestatus &= ~(1 << POTENTIAL_BIT);
   digitalWriteFast(OSZIB,HIGH);
    */
   // end test
   //   controllooperrcounterC = ausgabestatus;
   // Steuerung Anzeigeinstrumente   
   
  // Strom
   // since_I = 0; // Anzeigezeit resetten
   

   if (ausgabestatus & (1 << STROM_SETTING_BIT))
   {
      if (since_I < STROM_SETTING_ZEIT) //Strom-Setting anzeigen
      {
         analogWrite(I_OUT, get_targetvalue(0)* I_korr_array[bereichpos]); // Setting-Strom anzeigen)
      }
      else 
      {
         since_I = 0;// Anzeigezeit beenden
         // Instrumente dauernd anzeigen
         ausgabestatus &= ~(1 << STROM_SETTING_BIT); // Anzeige Setting beenden
         
      }
   }
   else // Ausgangsstrom anzeigen
   {
      since_I = 0;// Anzeigezeit beenden
      analogWrite(I_OUT, get_analogresult(0)* I_korr_array[bereichpos]); // IST-Strom anzeigen)
   }
   
   // Potential
   if (ausgabestatus & (1 << POTENTIAL_BIT))
   {
      if(since_P < POTENTIAL_ZEIT)
      {
         // analogWrite(U_OUT, potential  * U_KORR); // Potential auf Instrument anzeigen
         analogWrite(U_OUT, (P_OFFSET + (potential - P_OFFSET)* P_INSTRUMENTKORR) * U_KORR); // Potential auf Instrument anzeigen
      }
      else
      {
         since_P = 0; // Anzeigezeit beenden
         ausgabestatus &= ~(1 << POTENTIAL_BIT); // 
      }
   }
   else // Ausgangsspannung anzeigen
   {
      since_P = 0; // Potential anzeigen unterdrücken
      if (ausgabestatus & (1<<AUSGANG_BIT))
      {
         
         if (ausgangramp) // einschaltverzögerung
         {
            ausgangramp--;
         }
         else
          
         {
         analogWrite(U_OUT, get_analogresult(1) * U_KORR); // IST-Spannung
         }
      }
      else
      {
         analogWrite(U_OUT, get_targetvalue(1)* U_KORR); // SOLL-Spannung
      }
   }
   
   //analogWrite(U_OUT, get_analogresult(1) * U_KORR); // Analog-Spannung anzeigen
   
   analogWrite(POTENTIAL_OUT,(P_OBERGRENZE - potential) * P_KORR); // Potentialausgang an Buchse ausgeben
   
  // digitalWriteFast(OSZIB,HIGH);

}



#pragma mark prell_ISR
void prell_ISR(void)
{   
   //prellcounter++;
   
   ausgangsspannung = get_targetvalue(1);
 //  ausgabestatus |= (1<<AUSGANG_BIT);
  // ausgabestatus &= ~(1 << POTENTIAL_BIT);
   adc->startSynchronizedSingleRead(ADC_U, ADC_I); // 3 us
   
   if (abs(lastausgangspannung - get_analogresult(1)) > 500) // fehler
   {
      ausgangcheck++;
      lastausgangspannung = get_analogresult(1);
   }

  }

void DREHGEBER0_ISR(void) // I
{
   //digitalWriteFast(OSZIA,LOW);
   if (digitalReadFast(DREHGEBER0_B) == 0) //  Impuls B ist 0,  kommt spaeter: RI A
   {
      inc_targetvalue(0, 10);
   }
   else // //  Impuls B ist 1,  war frueher:RI B
   {
       dec_targetvalue(0, 10);
   }
   
   since_I = 0; // Anzeigezeit resetten
   ausgabestatus |= (1 << STROM_SETTING_BIT); // Auf Instrument_I_Setting anzeigen. Wird in debounce_ISR gecheckt
   //digitalWriteFast(OSZIA,HIGH);
}

void DREHGEBER1_ISR(void) // U
{
   //digitalWriteFast(OSZIA,LOW);
   if (digitalReadFast(DREHGEBER1_B) == 0) //  Impuls B ist 0,  kommt spaeter: RI A
   {
       inc_targetvalue(1, 10);
   }
   else // //  Impuls B ist 1,  war frueher:RI B
   {
       dec_targetvalue(1, 10);
   }
   //digitalWriteFast(OSZIA,HIGH);
   //ausgangsspannung = get_targetvalue(1);
}

void DREHGEBER2_ISR(void) // P wird geaendert
{
   //digitalWriteFast(OSZIA,LOW);
   if (digitalReadFast(DREHGEBER2_B) == 0) //  Impuls B ist 0,  kommt spaeter: RI A
   {
       if (potential < MAX_POT - 20)
      {
         potential += 20;
      }
   }
   else // //  Impuls B ist 1,  war frueher:RI B
   {
      if (potential > 20)
      {
         potential -= 20;
      }
      else
      {
         potential = 0;
      }
   }
   //controllooperrcounterC++;;
   since_P = 0; // Anzeigezeit resetten, 
   U_instrumentcounter = 0; // Anzeigezeit reset Anzeigezeit auf 0 setzen, wird in 
   ausgabestatus |= (1 << POTENTIAL_BIT); // Auf Instrument_U Potential anzeigen. Wird in debounce_ISR gecheckt
   //digitalWriteFast(OSZIA,HIGH);

}





void setup()
{
   Serial.begin(9600);
   pinMode(OSZIA,OUTPUT);
   digitalWriteFast(OSZIA,HIGH); // setup
   pinMode(OSZIB,OUTPUT);
   digitalWriteFast(OSZIB,HIGH); // setup
   loopLED = 18;
   // LCD
   pinMode(LCD_RSDS_PIN, OUTPUT);
   pinMode(LCD_ENABLE_PIN, OUTPUT);
   pinMode(LCD_CLOCK_PIN, OUTPUT);
   digitalWrite(LCD_RSDS_PIN,1);
   digitalWrite(LCD_ENABLE_PIN,1);
   digitalWrite(LCD_CLOCK_PIN,1);
   pinMode(loopLED, OUTPUT);
   analogReadResolution(12);
   analogWriteResolution(12);
   
   
   // Drehgeber
   pinMode(DREHGEBER0_A,INPUT); // Kanal A
   pinMode(DREHGEBER0_A,INPUT_PULLUP); // HI
   attachInterrupt(DREHGEBER0_A, DREHGEBER0_ISR, FALLING); //
   pinMode(DREHGEBER0_B,INPUT); // Kanal B
   pinMode(DREHGEBER0_B,INPUT_PULLUP); // HI
   
   
   pinMode(DREHGEBER1_A,INPUT); // Kanal A
   pinMode(DREHGEBER1_A,INPUT_PULLUP); // HI
   attachInterrupt(DREHGEBER1_A, DREHGEBER1_ISR, FALLING); //
   pinMode(DREHGEBER1_B,INPUT); // Kanal B
   pinMode(DREHGEBER1_B,INPUT_PULLUP); // HI

      
   pinMode(DREHGEBER2_A,INPUT); // Kanal A
   pinMode(DREHGEBER2_A,INPUT_PULLUP); // HI
   attachInterrupt(DREHGEBER2_A, DREHGEBER2_ISR, FALLING); //
   pinMode(DREHGEBER2_B,INPUT); // Kanal B
   pinMode(DREHGEBER2_B,INPUT_PULLUP); // HI

   pinMode(TONE, OUTPUT);
   
   pinMode(POTENTIAL_OUT, OUTPUT);
   
   
   // debounce
   
   for (uint8_t i= 0;i<8;i++)
   {
      tastenstatusarray[i].tasten_history = 0;
      tastenstatusarray[i].pressed = 0;
      tastenstatusarray[i].pin = 0xFF;
   }
   
   // Tasten
   /*
    // Taster
    
    // Umschalter Bereich
    #define BEREICH_UP      2
    #define BEREICH_DOWN    3
    
    // ON OFF 
    #define OUT_ON          4
    #define OUT_OFF         5
    
    #define LCD_RESET       0
    #define SAVE            1
    */
   tastenstatusarray[0].pin = LCD_RESET;
   tastenstatusarray[1].pin = SAVE;
   
   tastenstatusarray[2].pin = BEREICH_UP;
   tastenstatusarray[3].pin = BEREICH_DOWN;

   tastenstatusarray[4].pin = OUT_ON;
   tastenstatusarray[5].pin = OUT_OFF;

   //
   
   // SPI
    
   
   pinMode(U_OUT,OUTPUT);
   analogWriteFrequency(U_OUT, 10000);
   

   
   init_analog(); 
   
   pinMode(SPI_CS,OUTPUT);
   mcp0.begin();
   /*
    • PortA registeraddresses range from 00h–0Ah
    • PortB registeraddresses range from 10h–1Ah
    PortA output, PortB input: Direction 1 output: direction 0
    0x0F: A: out B: in
    */
   //mcp0.gpioPinMode(OUTPUT);
   mcp0.gpioPinMode(0x00FF);
   mcp0.portPullup(0x00FF);
   mcp0.gpioPort(0xFFFF);
   
   prelltimer.priority(0);
   prelltimer.begin(prell_ISR,40);
  
   U_soll = U_START;//  10V
   set_target_adc_val(1,U_soll);
   
   I_soll = I_START; // 100mA

   set_target_adc_val(0,I_soll);
   
   lcd_initialize(LCD_FUNCTION_8x2, LCD_CMD_ENTRY_INC, LCD_CMD_ON);
   _delay_ms(100);
   lcd_puts("Teensy");

}
uint8_t currentcontrolstatus=0;
void loop()
{
   int n;
   // Stromwarnung
   currentcontrolstatus = get_loopcontrol(); // von loopcontrol
//   controllooperrcounterA = currentcontrolstatus;
   if (currentcontrolstatus == 16) // 16, voltage control
   {
      currentstatus &= ~(1<<CURRENTWARNUNG); // Warnung off
      since_WARN=0;
      regA &= ~(1<< LED_I_WARRN);
    }
   else // current control
   {
      if (!(currentstatus & (1<<CURRENTWARNUNG)))
      {
         since_WARN=0;
         currentcontrolcounter = 0;
         currentstatus |= (1<<CURRENTWARNUNG); // Warnung on
      }
   }
    if (currentstatus & (1<<CURRENTWARNUNG))
   {
      
      if (since_WARN > CURRENTWARNUNG_DELAY) // Delay abgelaufen. Ton
      {
         since_WARN = 0;         
         if (currentcontrolcounter < CURRENTWARNUNG_ANZAHL)
         {
            currentcontrolcounter++;
            tone(TONE,400,300);
         
         }
         else
         {
            //currentstatus &= ~(1<<CURRENTWARNUNG); // Warnung off
            //currentcontrolcounter = 0;
         }
         // LED_I_WARRN
         regA ^= (1<< LED_I_WARRN);
      }
    }
   // end Stromwarnung
   
   if (sinceblink > 10) 
   {  
      
  //    tone(TONE,400,300);
      lcd_gotoxy(19,1);
      lcd_putc(' ');
      
    //  lcd_gotoxy(16,0);
    //  lcd_puthex(loopcontrol);
    //  loopcontrol = 0;
      // sine wave
         /*
         sinval = sin(phase) * 400.0 + 800.0;
         phase = phase + 0.2;
         if (phase >= twopi)
         {
            phase = 0;
         }
         //    set_target_adc_val(1,sinval);
         //    Serial.printf("sine wave: phase: \t%2.2f\t sin: \t%2.2f \t",phase, sin(phase));
         //   Serial.println(val);
         //mcp0.gpioPort(0xFFFF);
         // end sine wave
         */
         //tone(TONE,400,300);
      if (digitalRead(loopLED) == 1)
      {
         //Serial.printf("LED ON\n");
         digitalWriteFast(loopLED, 0);
         //digitalWriteFast(OSZIA,LOW);
         //digitalWriteFast(SPI_CLK,LOW);
         //mcp0.gpioDigitalWrite(1,LOW);
         //mcp0.gpioDigitalWrite((regA ),LOW);
         //mcp0.gpioPort(0xFFFF);
         /*
          for (int i=0;i<16;i++)
          {
          mcp0.gpioDigitalWrite(i,LOW);
          // delay(150);
          }
          */
         
      }
      else
      {
         //Serial.printf("LED OFF\n");
         digitalWriteFast(loopLED, 1);
         
      }
      
 /*      
  // regA als rundlauf
      if (regA < 0x10)
      {
         regA <<= 1;
      }
      else
      {
         regA = 1;
      }
  */
//      lcd_gotoxy(16,0);
 //     lcd_puthex(regA);
      sinceblink = 0;
      
      outbuffer[0] = 0;
      outbuffer[1] = (U_Pot & 0xFF00) >> 8;
      outbuffer[2] = U_Pot & 0x00FF;
      
      lcd_gotoxy(0,0);
      lcd_putc('U');
      
      uint16_t U = ausgangsspannung;
 //     lcd_putint12(U);
      lcd_putc(' ');
      uint16_t Udisp = adc_u_to_disp(U);
      
 //     int_to_dispstr(Udisp,buf,1);
      
      int_to_dispstr(U/2,buf,1);
      lcd_puts(buf);
      lcd_putc('V');
  //    lcd_putint12(adc_u_to_disp(U));
      lcd_putc('*');
      
#pragma mark Strom     
      //Strom 
      
      uint16_t Strom = get_analogresult(0); ///10 * 0.86;
      switch (bereichpos)
      {
         case 0: // 30mA
         {
            Strom /= 100;
            Strom *= 1.15;
            lcd_putint12(Strom);
         }break;
         case 1: // 100mA
         {
            Strom /= 10;
            Strom *= 0.37;
            lcd_putint12(Strom);
            lcd_puts("mA");
         }break;
         case 2: // 300mA
         {
            Strom /= 10;
            Strom *= 1.12;
            lcd_putint12(Strom);
            lcd_puts("mA");
         }break;
            
         case 3://1A
         {
            //Strom /= 10;
            Strom *= 0.33;
            lcd_putint12(Strom);
            lcd_puts("mA");
         }break;
         case  4: 
         {
            //Strom /= 10;
            Strom *= 0.285;
            int_to_dispstr(Strom,buf,1);
            lcd_puts(buf);
            lcd_puts("A");
         }break;
            
      }// switch
//      int_to_dispstr(Strom,buf,1);
//      lcd_puts(buf);
      
      //lcd_puts(Udisp);
      //lcd_putint(
 //     lcd_putint12(U_soll);
      //lcd_puthex(regB);
      //lcd_puts(buf);
      // "sincePrint" auto-increases
      

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
      
      
      //   Serial.printf("\n");
      
      
      
      //`
      adc->printError();
      adc->resetError();
      
      n = RawHID.send((void*)outbuffer, 100);
      
      if (n > 0) 
      {
         Serial.print(F("Transmit packet "));
         packetCount = packetCount + 1;
      } 
      else 
      {
         Serial.println(F("Unable to transmit packet"));
      }
      
      int outcontrol = (outbuffer[1]<<8) + outbuffer[2];
      //Serial.printf("outH: %02X outL: %02X wert: \t%d\n",outbuffer[1], outbuffer[2],outcontrol);
      
      
      
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
      uint16_t temp = 0;
      #pragma mark debounce
 //     lcd_gotoxy(12,1);
 //     lcd_puthex(SPItastenstatus);
      SPItastenstatus=0;
      
      tastenbitstatus = 0;
    } // if sinceblink
   
#pragma mark Tasten
      /*   
    tastenstatusarray[0].pin = LCD_RESET;
    tastenstatusarray[1].pin = SAVE;
    
    tastenstatusarray[2].pin = BEREICH_UP;
    tastenstatusarray[3].pin = BEREICH_DOWN;
    
    tastenstatusarray[4].pin = OUT_ON;
    tastenstatusarray[5].pin = OUT_OFF;
   */
   
   if (tastenstatusarray[LCD_RESET].pressed)
   {
//      lcd_gotoxy(19,1);
//      lcd_putc('A');
      lcd_initialize(LCD_FUNCTION_8x2, LCD_CMD_ENTRY_INC, LCD_CMD_ON);
      _delay_ms(100);
      tastenstatusarray[LCD_RESET].pressed = 0;
   }
   
   // Taste[1]
   if (tastenstatusarray[SAVE].pressed)
   {
//      lcd_gotoxy(19,1);
//      lcd_putc('B');
      tastenstatusarray[SAVE].pressed = 0;
   }
 
   if (tastenstatusarray[OUT_ON].pressed)
   {
 //     lcd_gotoxy(19,1);
 //     lcd_putc('E');
      
      ausgabestatus |= (1 << AUSGANG_BIT); // Ausgang ON 
      ausgangramp = AUSGANG_RAMP_MAX;
      regA |= (1<< LED_OUT);
      tastenstatusarray[OUT_ON].pressed = 0;
   }
   
   if (tastenstatusarray[OUT_OFF].pressed)
   {
 //     lcd_gotoxy(19,1);
 //     lcd_putc('F');
      
      ausgabestatus &= ~(1 << AUSGANG_BIT);// Ausgang OFF 
      regA &= ~(1<< LED_OUT);
      tastenstatusarray[OUT_OFF].pressed = 0;
   }

   //
   // Taste[2]
   if (tastenstatusarray[BEREICH_UP].pressed) // Taste gedrueckt ON
   {
  //    lcd_gotoxy(19,1);
  //    lcd_putc('C');
  //    loopcontrol |= (1<<6);
      if (bereichpos )
      {
         bereichpos--;
      }
      tastenstatusarray[BEREICH_UP].pressed = 0;
   }
   
   if (tastenstatusarray[BEREICH_DOWN].pressed) // Taste gedrueckt OFF
   {
      if (bereichpos < 4)
      {
         bereichpos++;
      }
 //     loopcontrol |= (1<<7);
  //    lcd_gotoxy(19,1);
//   lcd_putc('D');
      tastenstatusarray[BEREICH_DOWN].pressed = 0;
   }

   
   if (sincelcd > 10) // LCD aktualisieren
   {
      sincelcd = 0;
      
   //   lcd_gotoxy(9,0);
      //lcd_putint(lastausgangspannung & 0xFF);
   //   lcd_putint(ausgabestatus);
      
      lastausgangspannung = get_analogresult(1);

   //
   //controllooperrcounterC = bereichpos;
   currentcontrolstatus = is_current_limit();
 
      
   lcd_gotoxy(0, 1);
   lcd_puthex(ausgabestatus);
   lcd_putc(' ');
   lcd_putint(controllooperrcounterA);
   lcd_putc(' ');
   lcd_putint(controllooperrcounterB);
   lcd_putc(' ');
   lcd_putint(controllooperrcounterC);
   lcd_putc(' ');
   lcd_putint(controllooperrcounterD);

   
   lcd_gotoxy(0,2);
   lcd_putc('I');
   lcd_putc(' ');
   lcd_puts("a");
   lcd_putint12(get_analogresult(0));
   lcd_putc(' ');
   lcd_puts("t");
   lcd_putint12(get_targetvalue(0));
   
//   lcd_putc(' ');
//   lcd_putint(outbuffer[35]); // U low
   
   // lcd_putc(' ');
   lcd_gotoxy(0,3);
   lcd_putc('U');
   lcd_putc(' ');
   lcd_puts("a");
   lcd_putint12(get_analogresult(1));
   lcd_putc(' ');
   lcd_puts("t");
   lcd_putint12(get_targetvalue(1));
   //currentcontrolstatus = get_currentcontrol();

   
   lcd_gotoxy(14,3);
   //currentcontrolstatus = get_currentcontrol();
   //lcd_putc(' ');
   lcd_putint(loopcontrol);
   
   
   //lcd_putint2(get_currentcontrol());
   lcd_putc(' ');
   //lcd_putint2(currentcontrol_level);
   lcd_putint2(get_currentcontrol());

   lcd_gotoxy(15,0);
   lcd_putc('d');
   //lcd_putint12(get_dacval());
   lcd_putint12(potential);

//   loopcontrol = 0;
    
} // end sincelcd

#pragma mark USB   
//   if (sinceusb > 100)
   {
      //sinceusb = 0;
      //digitalWriteFast(OSZIA,LOW);
      r = RawHID.recv((void*)inbuffer, 0);
      //digitalWriteFast(OSZIA,HIGH);
      if (r > 0)
      {
         
         //Serial.println("Print every 2.5 seconds");
         //Serial.printf("usb_rawhid_recv: %x\n",r);
         U_soll = ((inbuffer[4]<<8) + inbuffer[5]) ;
         
         I_soll = (inbuffer[2]<<8) + inbuffer[3];
         
         set_target_adc_val(0,I_soll);
         set_target_adc_val(1,U_soll);
         /*
         uint8_t i=0;
         //Serial.printf("inbuffer:\t");
         while (i<10)
         {         
            //Serial.printf("%x\t",inbuffer[i]);
            i++;
         }
          */
         
         
      }
      //Serial.printf("U_soll:\t %d\n",U_soll);
      //analogWrite(9,U_soll);
      
      

      
   }
   
} // end loop
