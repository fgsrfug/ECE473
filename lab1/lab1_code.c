// lab1_code.c 
// R. Traylor
// 7.21.08

//Edited by Jesus Narvaez
//9.26.18

//This program increments a binary display of the number of button pushes on switch 
//S0 on the mega128 board.

#include <avr/io.h>
#include <util/delay.h>

//*******************************************************************************
//                            debounce_switch                                  
// Adapted from Ganssel's "Guide to Debouncing"            
// Checks the state of pushbutton S0 It shifts in ones till the button is pushed. 
// Function returns a 1 only once per debounced button push so a debounce and toggle 
// function can be implemented at the same time.  Expects active low pushbutton on 
// Port D bit zero.  Debounce time is determined by external loop delay times 12. 
//*******************************************************************************

int8_t debounce_switch() {
  static uint16_t state = 0; //holds present state
  state = (state << 1) | (! bit_is_clear(PINA, 2)) | 0xE000;
  if (state == 0xF000) return 1;
  return 0;
}

//*******************************************************************************
// Check switch S0.  When found low for 12 passes of "debounce_switch(), increment
// PORTB.  This will make an incrementing count on the port B LEDS. 
//*******************************************************************************
int main()
{
int8_t ones = 0;           //Assign counter for ones column
int8_t tens = 0;           //Assign counter for tens column
int8_t count;              //Declare variable holding both places 

DDRB = 0xFF;            //set port B to all outputs
PORTB = 0xFF;

while(1){               //do forever
 if(debounce_switch()) {
    ones++;             //Increment ones counter when debounce_switch returns 0
    
    if (ones > 9) {     //If the ones place overflows, set it to 0
        ones = 0; 
        tens++;         //Increment tens place if ones overflows
    }
    
    if (tens > 9) {     //If tens overflows, set it to 0
        tens = 0;
    }

    count = (ones | (tens << 4)); //Assign count to value of ones plus tens bitshifted by 4
    
        PORTB = (count);  //if switch true for 12 passes, increment port B

    }  //if switch true for 12 passes, increment port B
 
 _delay_ms(2);                    //keep in loop to debounce 24ms
  } //while 
} //main
