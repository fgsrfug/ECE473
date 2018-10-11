// lab2_skel.c 
// R. Traylor
// 9.12.08

//  HARDWARE SETUP:
//  PORTA is connected to the segments of the LED display. and to the pushbuttons.
//  PORTA.0 corresponds to segment a, PORTA.1 corresponds to segement b, etc.
//  PORTB bits 4-6 go to a,b,c inputs of the 74HC138.
//  PORTB bit 7 goes to the PWM transistor base.

#define F_CPU 16000000 // cpu speed in hertz 
#define TRUE 1
#define FALSE 0
#include <avr/io.h>
#include <util/delay.h>

//holds data to be sent to the segments. logic zero turns segment on
uint8_t segment_data[4]; 

//decimal to 7-segment LED display encodings, logic "0" turns on segment
uint8_t dec_to_7seg[12]; 

//*******************************************************************************
//                            debounce_switch                                  
// Adapted from Ganssel's "Guide to Debouncing"            
// Checks the state of pushbutton S0 It shifts in ones till the button is pushed. 
// Function returns a 1 only once per debounced button push so a debounce and toggle 
// function can be implemented at the same time.  Expects active low pushbutton on 
// Port D bit zero.  Debounce time is determined by external loop delay times 12. 
//*******************************************************************************
int8_t debounce_switch(uint8_t button) {
    static uint16_t state [8] = {0}; //holds present state
    state[button] = (state[button] << 1) | (! bit_is_clear(PINA, button)) | 0xE000;
    if (state[button] == 0xF000) return 1;
  return 0;
  }

//******************************************************************************
//                            dec_2_bcd                                      
//Returns the hexadecimal value of a decimal value.
//Pass in the integer to be converted to hex and it returns the appropriate index in
//the array.
uint8_t dec_to_bcd(uint16_t num) {
    uint8_t bcd_array[10] = {0b11000000, 0b11111001, 0b10100100, 0b10110000, 0b10011001, 0b10010010, 0b10000010, 0b11111000, 0b10000000, 0b10011000};

return bcd_array[num];
}

//***********************************************************************************

//***********************************************************************************
//                                   segment_sum                                    
//takes a 16-bit binary input value and places the appropriate equivalent 4 digit 
//BCD segment code in the array segment_data for display.                       
//array is loaded at exit as:  |digit3|digit2|colon|digit1|digit0|
void segsum(uint16_t sum) {
  
  uint8_t result, digits, i;

  //determine how many digits there are
  if (sum >= 1000){
    digits = 4;
  }
  else if (sum >= 100){
    digits = 3;
  }  
  else if (sum >= 10){
    digits = 2;
  }
  else 
    digits = 1;

  for (i = 0; i < 4; i++){
    segment_data[i] = 0;
    }

  //break up decimal sum into 4 digit-segments
  for (i = 0; i < digits; i++){
    result = (sum % 10);
    segment_data[i] = result;
    sum = (sum / 10);
  }


    switch (digits){
        case 4: PORTB = 0x00;
                PORTA = dec_to_bcd(segment_data[0]);
                _delay_ms(2);
                PORTB = 0x10;
                PORTA = dec_to_bcd(segment_data[1]);
                _delay_ms(2);     
                PORTB = 0x30;
                PORTA = dec_to_bcd(segment_data[2]);
                _delay_ms(2);     
                PORTB = 0x40;
                PORTA = dec_to_bcd(segment_data[3]);
                _delay_ms(2);    
                break;
        case 3: PORTB = 0x00;
                PORTA = dec_to_bcd(segment_data[0]);
                _delay_ms(2); 
                PORTB = 0x10;
                PORTA = dec_to_bcd(segment_data[1]);
                _delay_ms(2);    
                PORTB = 0x30;
                PORTA = dec_to_bcd(segment_data[2]);
                _delay_ms(2);    
                break;
        case 2: PORTB = 0x00;
                PORTA = dec_to_bcd(segment_data[0]);
                _delay_ms(2);     
                PORTB = 0x10;
                PORTA = dec_to_bcd(segment_data[1]);
                _delay_ms(2);    
                break;
        case 1: PORTB = 0x00;
                PORTA = dec_to_bcd(segment_data[0]);
                _delay_ms(2);     
                break;
  }   
  //blank out leading zero digits 
  //now move data to right place for misplaced colon position
}//segment_sum
//***********************************************************************************

uint8_t main() {
//set port bits 4-7 B as outputs

uint16_t count = 0;

DDRB = 0xFF;
while(1){
  //insert loop delay for debounce
  _delay_ms(2);     
  
  //make PORTA an input port with pullups 
  DDRA = 0x00;
  PORTA = 0xFF;
  
  //enable tristate buffer for pushbutton switches
  uint8_t temp = PORTB;
  PORTB |= 0x70; 
  
  //now check each button and increment the count as needed
  for(int i = 0; i < 8; i++){
    if(debounce_switch(i)){
        count += (1 << i);
        }
    }
  //disable tristate buffer for pushbutton switches
  PORTB = temp;
  
  //bound the count to 0 - 1023
  if(count > 1023){
    count = 1;
  }

//count = 84;
  //break up the disp_value to 4, BCD digits in the array: call (segsum)
 DDRA = 0xFF;
 segsum(count);
  }//while
}//main
