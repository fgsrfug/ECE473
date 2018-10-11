Names:





//This code shows how debouncing methods and timing can effect
//the results of sampling mechanical switches. TCNT0 is used with 
//with different clock speeds to vary the sampling period of a 
//pushbutton switch on the AVR board.

//Try different pushbuttons and see which is worst. They can vary alot.
//Vary the sampling time and note any difference. Note the use of #ifdef 
//statements to enable or disable debouncing.

//R. Traylor 11.11.2016
                            
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <string.h>
#include <stdlib.h>
#include "hd44780_driver/hd44780.h"

//If the next line is commented out, debouncing is disabled
//#define DEBOUNCE

char lcd_str[16];                    //holds string to send to lcd
volatile uint16_t switch_count = 0;  //count switch closures
uint8_t i;                           //just a dumb counter

//******************************************************************************
//                            spi_init                               
//Initalizes the SPI port on the mega128. Does no further device specific inits
//******************************************************************************
void spi_init(void){
    DDRB |=  0x07;  //Turn on SS, MOSI, SCLK
    //mstr mode, sck=clk/2, cycle 1/2 phase, low polarity, MSB 1st, no interrupts 
    SPCR=(1<<SPE) | (1<<MSTR); //enable SPI, clk low initially, rising edge sample
    SPSR=(1<<SPI2X); //SPI at 2x speed (8 MHz)  
}//spi_init

//interrupt service routine runs at each TCNT0 interrupt
ISR(TIMER0_COMP_vect){
#ifdef DEBOUNCE 
  static uint16_t state = 0;        //"state" holds present state
  state = (state << 1) | (! bit_is_clear(PIND, 1)) | 0xE000;
  if (state == 0xF000) {switch_count++;}  //increment if pushed for 12 cycles 
#else 
  //3 state state machne makes a pushbutton give one pulse for each
  //pushbutton push and release
  static enum button_state_type{NOTSET, SET, WAIT} button_state;
  switch(button_state){
    case(NOTSET): 
      if(bit_is_clear(PIND,1)){ 
        button_state=SET;
      } break;
    case(SET): 
      switch_count++; 
      button_state=WAIT; 
      break;
    case(WAIT): 
      if(bit_is_set(PIND,1)){ 
        button_state=NOTSET;
      } 
      break;
    default: break;
  } //switch                  
#endif
} //ISR

int main(void){
  spi_init();      //initalize SPI 
  lcd_init();      //initalize LCD 
  clear_display(); //manually clear LCD display 
  cursor_off();    //keep LCD cursor off
 
  //Setup timer/counter TCNT0 to run in CTC mode
  TIMSK |= (1<<OCIE0);  //enable interrupts
  TCCR0 |= (1<<WGM01);  //CTC mode, no prescale yet

//some sample values
//OCR0  = 0x01;        //compare register small, counter interrupts quickly
//OCR0  = 0xFF;        //compare register big, long delay to interrupt 
//TCCR0 |= (1<<CS00);                     //no prescaling of clock to counter
//TCCR0 |= (1<<CS01)|(1<<CS00);           //prescale TCNT0 clock by 32 
//TCCR0 |= (1<<CS02)|(1<<CS00);           //prescale TCNT0 clock by 128
//TCCR0 |= (1<<CS02)|(1<<CS01)|(1<<CS00); //prescale TCNT0 clock by 1024

//try some of these values with and without debouncing
//interrupt period = 62.5ns * (OCR0+1) * prescale
//  OCR0=0xFF; TCCR0 |= (1<<CS02)|(1<<CS01)|(1<<CS00); //switch sample period=32.7ms
//  OCR0=0x3F; TCCR0 |= (1<<CS02)|(1<<CS00);           //switch sample period=1ms 
    OCR0=0x01; TCCR0 |= (1<<CS00);                     //switch sample period=125ns

  sei(); //enable global interrupts
  while(1){   //main while loop to make LCD visible
    for(i=0;i<5;i++){_delay_ms(2);} //0.01 second wait
    itoa(switch_count, lcd_str, 10);
    string2lcd(lcd_str);
    cursor_home();
  } //while
} //main
