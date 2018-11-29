// lab5_48.c 
// Jesus Narvaez
// 11.13.18

//#define F_CPU 8000000 // cpu speed in hertz 
#define TRUE 1
#define FALSE 0
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stddef.h>
#include "hd44780.h"
#include "uart_functions_m48.h"
#include "lm73_functions.h"
#include "twi_master.h"
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>

//Read and write buffers for the LM73
extern uint8_t lm73_wr_buf[2]; 
extern uint8_t lm73_rd_buf[2];
//Temporary variable used to assemble the temperature readings
volatile static uint16_t lm73_temp;
//UART buffers for transmit and reciecve
char uart_rx_buf[40]; 
char uart_tx_buf[40]; 
//Dummy variable to read UDR0
uint16_t UDR_read;
//*******************************************************************************
//                            lm73_read                                  
// Read the LM73 from the mega48 board using TWI.
//*******************************************************************************
void lm73_read(){

    twi_start_rd(LM73_ADDRESS, lm73_rd_buf, 2); //read temperature data from LM73 (2 bytes) 
    _delay_ms(2);    //wait for it to finish
    lm73_temp = lm73_rd_buf[0]; //save high temperature byte into lm73_temp
    lm73_temp = lm73_temp << 8; //shift it into upper byte 
    lm73_temp |= lm73_rd_buf[1];//"OR" in the low temp byte to lm73_temp 
    
    //Left shift register to get accurate temp reading
    lm73_temp = lm73_temp >> 2;
    //multiply by .03125 to get temperature in celsius
    lm73_temp = (lm73_temp / 32);    
}
//*******************************************************************************
//***********************************************************************************
//                            ISR(USART_RX_vect)                                  
// Interrupts CPU whenever a byte is recieved from mega128
//*******************************************************************************
ISR(USART_RX_vect){                                  
        //Have dummy variable read register to clear flag
        UDR_read = UDR0;
        //read the temperature on the chip
        lm73_read();
        //send 2nd byte
        uart_putc(lm73_temp >> 8);
        //send 1st byte
        uart_putc(lm73_temp);

}
//***********************************************************************************
uint8_t main() {
    
    DDRD = (1<<PD3); 
    
    uart_init();
    init_twi();
    sei();
    //set LM73 mode for reading temperature by loading
    lm73_wr_buf[0] = LM73_PTR_TEMP;
    //load lm73_wr_buf[0] with temperature pointer
    twi_start_rd(LM73_PTR_TEMP, lm73_wr_buf, 1);
    //wait for the xfer to finish
    _delay_ms(2);

   while(1){    
    }
}

