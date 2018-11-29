// lab5.c 
// Jesus Narvaez
// 11.13.18

//  HARDWARE SETUP:
//  PORTA is connected to the segments of the LED display. and to the pushbuttons.
//  PORTA.0 corresponds to segment a, PORTA.1 corresponds to segement b, etc.
//  PORTB bits 4-6 go to a,b,c inputs of the 74HC138.
//  PORTB bit 7 goes to the PWM transistor base.

//#define F_CPU 16000000 // cpu speed in hertz 
#define TRUE 1
#define FALSE 0
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stddef.h>
#include "hd44780.h"
#include "lm73_functions.h"
#include "twi_master.h"
#include "uart_functions.h"
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>


//holds data to be sent to the segments. logic zero turns segment on
uint8_t segment_data[5]; 
//variable holding the current mode
enum mode {SET_ALARM, SET_CLOCK, SET_VOL, ALARM_HANDLER, TEMP};
static enum mode current_mode = SET_VOL;
//struct for storing time
struct time {
    uint8_t seconds;
    uint8_t minutes;
    uint8_t hours;
};    

//Declare 2 time structs to keep track of seperate times
volatile struct time alarm_clock = {0, 0, 0};
volatile struct time clock = {0, 0, 12};
//Declare booleans to tell us if the alarm going off, or if the alarm is set
static _Bool alarm_is_set = FALSE;
static _Bool alarm_sounding = FALSE;
//Boolean that determines if we're in celsius or not
static _Bool celsius_mode = TRUE;
//Holds a string to refresh the LCD
char lcd_string_array[16];
//Read and write buffers for the LM73
extern uint8_t lm73_wr_buf[2]; 
extern uint8_t lm73_rd_buf[2];
//Temporary variable used to assemble the temperature readings
volatile static uint16_t lm73_temp;
//UART buffers for transmit and reciecve
char uart_rx_buf[40]; 
char uart_tx_buf[40]; 
volatile static uint16_t remote_read = 97;
//*******************************************************************************
//                            timer0_init                                  
// Initialize the timer to be used to count up 1 second and run off the external
// 32kHz oscillator.
//*******************************************************************************
void timer0_init(){
    //Initialization of timer 0 using ext. oscillator. Used to count seconds.
    ////enable overflow interrupt
    //normal mode, 128 prescale
    //Use external oscillator
    TIMSK |= (1<<TOIE0);             
    TCCR0 |= (1<<CS00) | (1<<CS02);  
    ASSR |= (1<<AS0);                
}
//*******************************************************************************
//*******************************************************************************
//                            timer1_init                                  
// Oscillator used to make alarm sound. Sent out to PC0 to OP amp to eventually be
// audio output. Uses internal I/O clock with no prescale.
//*******************************************************************************
void timer1_init(){
    //Keep timer masked until the alarm needs to go off
    //Set to fast PWM, 64 prescale
    //No force compare
    TCCR1A |= (1<<WGM11) | (1<<WGM10);   
    TCCR1B |= (1<<WGM12) | (1<<CS10) | (1<<CS11);              
    TCCR1C = 0x00;                      
}
//*******************************************************************************
//*******************************************************************************
//                            timer2_init                                  
// Used in PWM mode to adjust brightness of display. Reads in ADCH to determine
// duty cycle of PB7.
//*******************************************************************************
void timer2_init(){
    //Set to fast PWM, no prescale, drive OC2 pin
    TCCR2 |= (1<<WGM20) |(1<<WGM21) | (1<<CS20) | (1<<COM21) | (1<<COM20); 
}
//*******************************************************************************
//*******************************************************************************
//                            timer3_init                                  
// Controls volume of audio output. Connected to VOLUME pin on audio amplifier
// and varies a voltage on the pin to increase or decrease the volume.
//*******************************************************************************
void timer3_init(){
    //Set to fast PWM, no prescale
    //No force compare
    TCCR3A |= (1<<COM3A1) | (1<<COM3A0) | (1<<WGM31) | (1<<WGM30);   
    TCCR3B |= (1<<WGM32) | (1<<CS30);              
    TCCR3C = 0x00;                      
}
//*******************************************************************************
/********************************************************************************/
//                            spi_init                               
//Initalizes the SPI port on the mega128. Does not do any further   
//external device specific initalizations.  Sets up SPI to be:                        
//master mode, clock=clk/2, cycle half phase, low polarity, MSB first
//interrupts disabled, poll SPIF bit in SPSR to check xmit completion
//********************************************************************************
void spi_init(){
    //Set SH/LD_N low to load in data
    DDRE  |= (1<<PE6);                
    //Turn on REGCLK
    DDRD  |= (1<<PD2);                
    //enable SPI, master mode 
    SPCR  |= (1<<SPE) | (1<<MSTR);    
    //double speed operation
    SPSR  |= (1<<SPI2X);              

}//spi_init
/********************************************************************************/
//*******************************************************************************
//                            adc_init()                                  
// Initialize the analog to digital converter so we receive input on PF0 to send to 
// TCNT0.
//*******************************************************************************
void adc_init(){
    //Use the internal 2.56V as AREF and left adjust
    ADMUX |= (1<<REFS1) | (1<<REFS0);   
    //Enable ADC and ADC interrupts
    //Start the conversion and prescale by 128
    ADCSRA |= (1<<ADEN) | (1<<ADIE) | (1<<ADSC) | (1<<ADPS2) | (1<<ADPS0) | (1<<ADPS0);                       
}
/********************************************************************************/
//*******************************************************************************
//                            debounce_switch                                  
// Adapted from Ganssel's "Guide to Debouncing"            
// Checks the state of pushbutton S0 It shifts in ones till the button is pushed. 
// Function returns a 1 only once per debounced button push so a debounce and toggle 
// function can be implemented at the same time.  Expects active low pushbutton on 
// Port D bit zero.  Debounce time is determined by external loop delay times 12. 
//*******************************************************************************
int8_t debounce_switch(uint8_t button) {
    volatile static uint16_t state [4] = {0}; //holds present state
    state[button] = (state[button] << 1) | (! bit_is_clear(PINA, button)) | 0xE000;
    if (state[button] == 0xF000) return 1;
    return 0;
}
/********************************************************************************/
//******************************************************************************
//                            dec_2_bcd                                      
//Returns the hexadecimal value of a decimal value.
//Pass in the integer to be converted to hex and it returns the appropriate value in
//the array.
uint8_t dec_to_bcd(uint16_t num) {
    uint8_t bcd_array[12] = {0b11000000, 0b11111001, 0b10100100, 0b10110000, 0b10011001, 0b10010010, 0b10000010, 0b11111000, 0b10000000, 0b10011000, 0b11111111, 0b11111100};

    return bcd_array[num];
}

//***********************************************************************************
//*******************************************************************************
//                            blink_colon                                  
// Sends values to the colon to blink at 1 second intervals. Called in
// timer 0 overflow ISR.
//*******************************************************************************
int8_t blink_colon(struct time *t) {
    
    
    //Check to see if the colon is on and either turn on segments
    //or blank segments.
    //Toggle colon_on before leaving function
    if (t->seconds % 2 == 0){
        return segment_data[2] = 11;
    }
    else if (t->seconds % 2 == 1){
        return segment_data[2] = 10;
    }
}
//***********************************************************************************
//*******************************************************************************
//                            display_sum                                  
// Takes the number of digits found in segsum and the array of data and outputs it
// onto the display.
// Does not return anything.
//*******************************************************************************
void display_sum() {
    
    //Set digit_select to MSB
    int8_t digit_select = 0x40;
    
    //Iterate through segment array
    for (int i = 4; i >= 0; i--) {
    
        //Select digit to turn on and send PORTA the BCD
        PORTB = digit_select;
        PORTA = dec_to_bcd(segment_data[i]);
        _delay_ms(1);
        PORTA = 0xFF;
        
        //Right shift digit_select and subtract 1 
        digit_select = digit_select >> 4;
        digit_select--;
        
        //Right shift digit_select back
        digit_select = digit_select << 4;
    }                    
}
//***********************************************************************************
//                                   segsum                                    
//takes a 16-bit binary input value and places the appropriate equivalent 4 digit 
//BCD segment code in the array segment_data for display.                       
//array is loaded at exit as:  |digit3|digit2|colon|digit1|digit0|
//***********************************************************************************
void segsum(int8_t minutes, int8_t hours) {

    uint8_t result, i;

    //break up decimal sum into 4 digit-segments
    for (i = 0; i < 5; i++){
        if (i < 2){
            //get the last digit of the current
            result = (minutes % 10);
            //place that digit into the segment array
            segment_data[i] = result;
            //divide sum by 10 to get to next one's digit
           minutes = (minutes / 10);
        }
        else if (i > 2){
            //get the last digit of the current
            result = (hours % 10);
            //place that digit into the segment array
            segment_data[i] = result;
            //divide sum by 10 to get to next one's digit
            hours = (hours / 10);
        }
    }
    display_sum();

}
//***********************************************************************************
//***********************************************************************************//                                   
//                                 spi_action                                    
// Takes in mode_multiplier and sends it out to diplay to the bargraph. Also reads
// in data from the encoders. The return value is passed into encoder_adjuster to
// adjust the data to send to segsum.
//***********************************************************************************
uint8_t spi_action(uint8_t mode_disp){
    //Set SH/LD_N high to not read encoder values
    PORTE |= (1 << PE6);                    
    //Place mode_disp out to bar graph
    SPDR = (mode_disp);                     
    //Wait for trasmission to complete
    while (bit_is_clear(SPSR, SPIF));       
    //Deselect bar graph 
    PORTD |= (1 << PD2);                         
    //Set low SH/LD_N to read encoder values
    PORTE |= (0 << PE6);                    
    _delay_us(1);
    //Ensure bar graph is low as to read encoder values
    PORTD &= ~(1 << PD2);                         
    //Ensure PORTE is set to read encoder values
    PORTE &= ~(1 << PE6);                         
    _delay_us(1);
    
    //Return regster with encoder values
    return SPDR;
}
//***********************************************************************************//                                   
//                                 encoder_adjuster                                    
// Takes the values recieved from the encoders and returns counter, which is +- 1,
// as well as mode_multiplier. This return value is passed directly into segsum.
//***********************************************************************************
int8_t encoder_adjuster(uint8_t enc_val){

    //create a variable to store previous encoder values
    static uint8_t prev_enc_val = 0;
    //save state of enc_val
    uint8_t temp;
    temp = enc_val;
    //create counter to increment or decrement
    int8_t counter = 0;
    static uint8_t CW = TRUE;
    static uint8_t dir_count = 0;

    //for loop checks both pairs of bits in the nibble for the encoders
    for (int i = 0; i < 2; i++){
        //get either the high or low bits from the encoder
        uint8_t current_bits = ((enc_val >> 2*i) & 0x03);
        switch((prev_enc_val >> 2*i) & 0x03){
            //compare the previous bits to the current bits to
            //see if we are incrementing or decrementing

            //Check the previous bits with the current ones
            case 0b00: if (current_bits == 0b01){ 
                           //Check if we were and still are going clockwise
                           if(CW == TRUE){
                               //Increment dir_count
                               dir_count++;
                               //If we've been going in the same direction for 4 turns
                               //we can increment or decrement the counter
                               //and reset the direction counter
                               if (dir_count == 4){
                                   counter --;
                                   dir_count = 0;
                               }
                           }
                           else {
                               dir_count = 0;
                               CW = FALSE;
                           }
                       }  
                       if (current_bits == 0b10){
                           if(CW == TRUE){
                               dir_count++;
                               if (dir_count == 4){
                                   counter ++;
                                   dir_count = 0;
                               }
                           }
                           else {
                               dir_count = 0;
                               CW = FALSE;
                           }
                       }
                       break;
            
            case 0b01: if (current_bits == 0b11){ 
                           if(CW == TRUE){
                               dir_count++;
                               if (dir_count == 4){
                                   counter --;
                                   dir_count = 0;
                               }
                           }
                           else {
                               dir_count = 0;
                               CW = FALSE;
                           }
                       }  
                       if (current_bits == 0b00){
                           if(CW == TRUE){
                               dir_count++;
                               if (dir_count == 4){
                                   counter ++;
                                   dir_count = 0;
                               }
                           }
                           else {
                               dir_count = 0;
                               CW = FALSE;
                           }
                       }
                       break;
            
            case 0b11: if (current_bits == 0b10){ 
                           if(CW == TRUE){
                               dir_count++;
                               if (dir_count == 4){
                                   counter --;
                                   dir_count = 0;
                               }
                           }
                           else {
                               dir_count = 0;
                               CW = FALSE;
                           }
                       }  
                       if (current_bits == 0b01){
                           if(CW == TRUE){
                               dir_count++;
                               if (dir_count == 4){
                                   counter ++;
                                   dir_count = 0;
                               }
                           }
                           else {
                               dir_count = 0;
                               CW = FALSE;
                           }
                       }
                       break;
            
            case 0b10: if (current_bits == 0b00){ 
                           if(CW == TRUE){
                               dir_count++;
                               if (dir_count == 4){
                                   counter --;
                                   dir_count = 0;
                               }
                           }
                           else {
                               dir_count = 0;
                               CW = FALSE;
                           }
                       }  
                       if (current_bits == 0b11){
                           if(CW == TRUE){
                               dir_count++;
                               if (dir_count == 4){
                                   counter ++;
                                   dir_count = 0;
                               }
                           }
                           else {
                               dir_count = 0;
                               CW = FALSE;
                           }
                       }
                       break;
            
            default:   break;
        }
    }

    //set the soon-to-be previous encoder to temp, which held the current
    //encoder values.
    prev_enc_val = temp;
    return counter;

}

//*******************************************************************************
//                            mode_select                                  
// Determines the current mode by checking which button was pressed recently pressed.
// Returns an enum.
//*******************************************************************************
void mode_select(int button, enum mode cur_mode, struct time *alarm){

    //Test to see what button was pressed
    switch (button) {
        //S0 was pressed
        case 0: //SET_CLOCK handler 
            //if we want to set the clock, we should stop it from counting up
            if (cur_mode == SET_VOL){            
                TIMSK &= ~(1<<TOIE0);
                current_mode = SET_CLOCK;
            } 
            //if we're done setting the clock, we can start it
            else if (cur_mode == SET_CLOCK){            
                TIMSK |= (1<<TOIE0);
                current_mode = SET_VOL;
            }
            break;
        //S1 was pressed
        case 1: //SET_ALARM handler
            if (cur_mode == SET_VOL){            
                current_mode = SET_ALARM;
            } 
            else if (cur_mode == SET_ALARM){            
                current_mode = SET_VOL;
                clear_display();
                //string2lcd("ALARM SET");
                alarm_is_set = TRUE;
            } 
            break;
        //S2 was pressed
        case 2: //snooze handler 
            if (alarm_sounding){
                //set alarm alarm time to 10 seconds
                alarm->seconds = clock.seconds + 10;
                alarm->minutes = clock.minutes;
                alarm->hours = clock.hours;
                TIMSK &= ~(1<<TOIE1);
                alarm_sounding = FALSE;
                ////alarm_set(alarm_seconds, alarm_minutes, alarm_hours);
                //continue to ALARM_HANDLER to disable alarm
                current_mode = SET_VOL;
           }
           else if (alarm_sounding == FALSE)
                current_mode = SET_VOL;
            break;
        //S3 was pressed
        case 3: //alarm off handler
            if (alarm_sounding){
                TIMSK &= ~(1<<TOIE1);
                alarm_sounding = FALSE;
                alarm_is_set = FALSE;
                current_mode = SET_VOL;
            }
            else if (alarm_sounding == FALSE) 
                current_mode = SET_VOL;
            break;
        
        case 4:
            celsius_mode ^= TRUE;
            break;
       
       default: //SET_VOL is the default setting
            current_mode = SET_VOL;
            break;
    }
}

//*******************************************************************************
//                            clock_set                                  
// Takes in the value from the encoders and use them to set the time struct
//*******************************************************************************

void clock_set (struct time *t, uint8_t enc_val){
    //Allow the encoders to adjust the minutes
    t->minutes += enc_val;
    //Set bounds for minutes and set hours appropriately
    if (t->minutes > 59){
        t->hours++;
        t->minutes = 0;
    }
    if (t->minutes < 0){
        t->hours--;
        t->minutes = 59;
    }

    if (t->hours > 23)
        t->hours = 0;

    if (t->hours < 0)
        t->hours = 23;

}

//***********************************************************************************
//                            ISR(TIMER0_OVF_vect)                                  
// ISR performed when timer 0 overflows. Determines when 1 second has
// passed. Responsible for blinking colon after every second and keeping
// track of the 24hr clock. 
//*******************************************************************************
ISR(TIMER0_OVF_vect){
    
    clock.seconds++;
    //Increment seconds, minutes and hours when appropriate
    //clear each when they reach 60, 60 and 24 respectively.
    if (clock.seconds > 59){
        clock.minutes++;
        clock.seconds = 0;
    }

    if (clock.minutes > 59){
        clock.hours++;
        clock.minutes = 0;
    }

    if (clock.hours > 23){
        clock.hours = 0;
    }
    
    //blink the colon
    blink_colon(&clock);
    
    //compare the current time to the set alarm
    if((clock.hours == alarm_clock.hours) && (clock.minutes == alarm_clock.minutes) && (clock.seconds == alarm_clock.seconds)){
        TIMSK |= (1<<TOIE1);
        alarm_sounding = TRUE;
    }
    
    //Toggle between celsius and fahrenheit every 5 seconds
    if(clock.seconds % 5 == 0){
       celsius_mode ^= TRUE;
    }
    
    //Send control signal to m48 
    uart_send();

}
//***********************************************************************************
//                            ISR(TIMER1_OVF_vect)                                  
// Used in fast PWM mode to oscillate PC0 to sent alarm tone. 
//*******************************************************************************
ISR(TIMER1_OVF_vect){
        PORTC ^= (1<<PC0);
}
//***********************************************************************************
//                            ISR(ADC_vect)                                  
// Used to extract ADC result and store in adc_result. 
//*******************************************************************************
ISR(ADC_vect){
    //Set OCR2 to the ADC value 
    OCR2 =(ADC / 8) + 220;
    //Enable the next conversion
    ADCSRA |= (1<<ADSC);
}
//***********************************************************************************
//                            vol_adjust()                                  
// Adjust the volume of the alarm using the encoders.
//*******************************************************************************
void vol_adjust(int8_t encoder_change){
    
    //values to hold the return values of spi_action and encoder_adjuster, respectively.
    uint16_t upper_limit = 1020;       
    uint16_t lower_limit = 20;       
    
    //multiply encoder_change by 20 to match the voltage steps for the
    //audio amp.
    encoder_change = encoder_change * 20; 
    
    //set upper and lower bounds for volume
    if(OCR3A + encoder_change > upper_limit) {
        OCR3A = upper_limit;
    }
    else if(OCR3A + encoder_change < lower_limit) {
        OCR3A = lower_limit;
    }
    
    else
        OCR3A += encoder_change;
}
//***********************************************************************************
//                            temp_conversion()                                  
// Convert temperature from celsius to farenheit 
//*******************************************************************************
uint16_t temp_conversion(uint16_t temp, _Bool c_mode){
    //Check if the user wants temperature in celsius
    if (c_mode){
        return temp;
    }
    //If not, convert to fahrenheit using formula 
    else
        temp = (((temp * 9) / 5) + 32);
        return temp;
}
//*******************************************************************************
//                            lm73_read                                  
// Read the LM73 from the mega128 board using TWI.
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
    //Convert to farenheit if in correct mode
    lm73_temp =  temp_conversion(lm73_temp, celsius_mode); 
    TWCR = (1<<TWINT) | (1<<TWSTO);
}

//***********************************************************************************
//                            temp_to_lcd                                  
// Send lm73_temp to be written on the lcd 
//*******************************************************************************
void temp_to_lcd(uint16_t temp, uint16_t rem_temp){
    
    //Declare a buffer to send to LCD 
    char buf_to_lcd[32] = " ";
    //Write temperatures to buffer
    sprintf(buf_to_lcd, "Remote: %d      Local: %d        ",rem_temp, temp);
    //Send buffer to LCD
    refresh_lcd(buf_to_lcd);

    return;
}
//***********************************************************************************
//                            mode_action()                                  
// Depending on what the current mode is, we do the appropriate action.
//*******************************************************************************
void mode_action(enum mode cur_mode, uint8_t encoder_change, struct time *t, struct time *alarm){
    switch (cur_mode){
        case 0: //setting an alarm
            clock_set(alarm, encoder_change);
            //segsum(alarm_clock.minutes, alarm_clock.hours);
            break;
        case 1: //setting a new time
            clock_set(t, encoder_change);
            //segsum(t);
            break;
        case 2: //adjusting the volume
            vol_adjust(encoder_change);
            break;
        case 3: //alarm handling
            break;
    }
}
//***********************************************************************************
//***********************************************************************************
//                            uart_send()                                  
// Send a byte of data through UART
//*******************************************************************************
void uart_send(){
    //Send character to trigger m48 recieve interrupt
    uart_putc('r');
}
//***********************************************************************************
//***********************************************************************************
//                            ISR(USART0_RX_vect)                                  
// Interrupts CPU whenever a byte is recieved from mega48
//*******************************************************************************
ISR(USART0_RX_vect){                                  
    //Get first byte from m48
    remote_read = uart_getc();
    //Make room to get the next byte
    remote_read = remote_read << 8;
    //Get said byte
    remote_read |= uart_getc();
    //Convert the temp if neccessary
    remote_read = temp_conversion(remote_read, celsius_mode);
}
//***********************************************************************************
uint8_t main() {

    //set port bits 4-7 B as outputs
    DDRB = 0xF7;
    //Set PC0 to output
    DDRC = (1<<PC0); 
    //Enable the volume PWM pin    
    DDRE = (1<<PE3);
    PORTE = (1<<PE3);
   
   //Initialize SPI
    spi_init();
    //Initialize the timers
    timer0_init();
    timer1_init();
    timer2_init();
    timer3_init();
    adc_init();
    lcd_init();
    init_twi();
    uart_init();

    //Enable the interrupts
    sei();
    
    //set LM73 mode for reading temperature by loading
    lm73_wr_buf[0] = LM73_PTR_TEMP;
    //load lm73_wr_buf[0] with temperature pointer address
    twi_start_rd(LM73_PTR_TEMP, lm73_wr_buf, 1);
    //wait for the xfer to finish
    _delay_ms(2);
    
    while(1){

        //values to hold the return values of spi_action and encoder_adjuster, respectively.
        int8_t SPDR_val;
        int8_t SPDR_adj;
        
        //alarm_time = 5;
        //make PORTA an input port with pullups 
        DDRA = 0xFC;
        PORTA = 0xFF;

        //enable tristate buffer for pushbutton switches
        uint8_t temp = PORTB;
        PORTB |= 0x70;


        //now check each button and pass that information to mode_select
        for(int i = 0; i < 5; i++){
            if(debounce_switch(i)){
               mode_select(i, current_mode, &alarm_clock);
            }
        }

        //disable tristate buffer for pushbutton switches
        PORTB = temp;

        //break up the disp_value to 4, BCD digits in the array: call (segsum)
        DDRA = 0xFF;
       
        if (alarm_is_set){
            //store values of spi_action and encoder_adjuster
            SPDR_val = spi_action(clock.seconds | (1<<7));
        }
        else {
            SPDR_val = spi_action(clock.seconds);
        }
        
        //send encoder data to be less sensitive
        SPDR_adj = encoder_adjuster(SPDR_val);
        
        //Based on what mode we are in, do an action
        mode_action(current_mode, SPDR_adj, &clock, &alarm_clock);
        
        //Read the current temperature from the local LM73
        lm73_read();
        
        //Call function to display temperature 
        temp_to_lcd(lm73_temp, remote_read);
        
        //send data out to display
        if (current_mode == SET_CLOCK) 
            segsum(clock.minutes, clock.hours);
        else if (current_mode == SET_VOL) 
            segsum(clock.minutes, clock.hours);
        else if (current_mode == SET_ALARM) 
            segsum(alarm_clock.minutes, alarm_clock.hours);
    }
}
