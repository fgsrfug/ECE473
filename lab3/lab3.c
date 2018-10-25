// lab3.c 
// Jesus Narvaez
// 10.25.18

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

//holds data to be sent to the segments. logic zero turns segment on
uint8_t segment_data[4]; 
static volatile uint8_t mode_multiplier = 1;

//*******************************************************************************
//                            timer_init                                  
// Initializes the timer to enable interrupts and to run off the I/O clock,
// prescaled by 128.
//*******************************************************************************
void timer_init(){
    //timer counter 0 setup, running off i/o clock
    TIMSK |= (1<<TOIE0);             //enable interrupts
    TCCR0 |= (1<<CS02) | (1<<CS00);  //normal mode, prescale by 128
}
//*******************************************************************************
/***********************************************************************/
//                            spi_init                               
//Initalizes the SPI port on the mega128. Does not do any further   
//external device specific initalizations.  Sets up SPI to be:                        
//master mode, clock=clk/2, cycle half phase, low polarity, MSB first
//interrupts disabled, poll SPIF bit in SPSR to check xmit completion
//************************************************

void spi_init(){
    //DDRB  |= (1<<PB1) | (1<<PB2);     //Turn on SS, MOSI 
    DDRE  |= (1<<PE6);                //Set SH/LD_N low to load in data
    DDRD  |= (1<<PD2);                //Turn on REGCLK
    SPCR  |= (1<<SPE) | (1<<MSTR);    //enable SPI, master mode 
    SPSR  |= (1<<SPI2X);              // double speed operation

}//spi_init


//*******************************************************************************
//                            debounce_switch                                  
// Adapted from Ganssel's "Guide to Debouncing"            
// Checks the state of pushbutton S0 It shifts in ones till the button is pushed. 
// Function returns a 1 only once per debounced button push so a debounce and toggle 
// function can be implemented at the same time.  Expects active low pushbutton on 
// Port D bit zero.  Debounce time is determined by external loop delay times 12. 
//*******************************************************************************
int8_t debounce_switch(uint8_t button) {
    volatile static uint16_t state [2] = {0}; //holds present state
    state[button] = (state[button] << 1) | (! bit_is_clear(PINA, button)) | 0xE000;
    if (state[button] == 0xF000) return 1;
    return 0;
}

//******************************************************************************
//                            dec_2_bcd                                      
//Returns the hexadecimal value of a decimal value.
//Pass in the integer to be converted to hex and it returns the appropriate value in
//the array.
uint8_t dec_to_bcd(uint16_t num) {
    uint8_t bcd_array[10] = {0b11000000, 0b11111001, 0b10100100, 0b10110000, 0b10011001, 0b10010010, 0b10000010, 0b11111000, 0b10000000, 0b10011000};

    return bcd_array[num];
}

//***********************************************************************************

//*******************************************************************************
//                            display_sum                                  
// Takes the number of digits found in segsum and the array of data and outputs it
// onto the display.
// Does not return anything.
//*******************************************************************************
void display_sum(int digits) {

    //Determines how many digits to turn on
    switch (digits){
        case 4: PORTB = 0x00;
                PORTA = dec_to_bcd(segment_data[0]);
                _delay_ms(1);
                PORTA = 0xFF;
                PORTB = 0x10;
                PORTA = dec_to_bcd(segment_data[1]);
                _delay_ms(1);     
                PORTA = 0xFF;
                PORTB = 0x30;
                PORTA = dec_to_bcd(segment_data[2]);
                _delay_ms(1);     
                PORTA = 0xFF;
                PORTB = 0x40;
                PORTA = dec_to_bcd(segment_data[3]);
                _delay_ms(1);    
                break;
        case 3: PORTB = 0x00;
                PORTA = dec_to_bcd(segment_data[0]);
                _delay_ms(1); 
                PORTA = 0xFF;
                PORTB = 0x10;
                PORTA = dec_to_bcd(segment_data[1]);
                _delay_ms(1);    
                PORTA = 0xFF;
                PORTB = 0x30;
                PORTA = dec_to_bcd(segment_data[2]);
                _delay_ms(1);    
                break;
        case 2: PORTB = 0x00;
                PORTA = dec_to_bcd(segment_data[0]);
                _delay_ms(1);     
                PORTA = 0xFF;
                PORTB = 0x10;
                PORTA = dec_to_bcd(segment_data[1]);
                _delay_ms(1);    
                break;
        case 1: PORTB = 0x00;
                PORTA = dec_to_bcd(segment_data[0]);
                _delay_ms(2);     
                break;
    }
}

//***********************************************************************************
//                                   segsum                                    
//takes a 16-bit binary input value and places the appropriate equivalent 4 digit 
//BCD segment code in the array segment_data for display.                       
//array is loaded at exit as:  |digit3|digit2|colon|digit1|digit0|
//***********************************************************************************
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

    display_sum(digits);

}
//***********************************************************************************
//***********************************************************************************//                                   
//                                 spi_action                                    
// Takes in mode_multiplier and sends it out to diplay to the bargraph. Also reads
// in data from the encoders. The return value is passed into encoder_adjuster to
// adjust the data to send to segsum.
//***********************************************************************************
uint8_t spi_action(uint8_t mode_disp){

    PORTE |= (1 << PE6);                    //Set SH/LD_N high to not read encoder values
    SPDR = (mode_disp);                     //Place mode_disp out to bar graph
    while (bit_is_clear(SPSR, SPIF));       //Wait for trasmission to complete
    //_delay_ms(100);
    PORTD |= (1 << PD2);                    //Deselect bar graph      
    PORTE |= (0 << PE6);                    //Set low SH/LD_N to read encoder values
    _delay_us(1);
    PORTD &= ~(1 << PD2);                         
    PORTE &= ~(1 << PE6);                         
    _delay_us(1);

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

    //for loop checks both pairs of bits in the nibble for the encoders
    for (int i = 0; i < 2; i++){
       //get either the high or low bits from the encoder
       uint8_t current_bits = ((enc_val >> 2*i) & 0x03);
            switch((prev_enc_val >> 2*i) & 0x03){
                //compare the previous bits to the current bits to
                //see if we are incrementing or decrementing
                case 0b00: if (current_bits == 0b01) 
                            counter -= mode_multiplier;
                        if (current_bits == 0b10) 
                            counter += mode_multiplier;
                        break;
                case 0b01: if (current_bits == 0b11) 
                            counter -= mode_multiplier;
                        if (current_bits == 0b00) 
                            counter += mode_multiplier;
                        break;
                case 0b11: if (current_bits == 0b10) 
                            counter -= mode_multiplier;
                        if (current_bits == 0b01) 
                            counter += mode_multiplier;
                        break;
                case 0b10: if (current_bits == 0b00) 
                            counter -= mode_multiplier;
                        if (current_bits == 0b11) 
                            counter += mode_multiplier;
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
// Selects the which mode multiplier to use by determining if a button got pressed,
// and if so, we toggle the switch state, and use the states for both buttons to 
// determine the mode_multiplier.
//*******************************************************************************
void mode_select(int button){

    volatile static char szero_on = FALSE;
    volatile static char sone_on = FALSE;

    if (button == 0) {                  //Checks S0 switch and toggle it on and off
        szero_on ^= 1;
    }

    if (button == 1) {                  //Checks S0 switch and toggle it on and off
        sone_on ^= 1;
    }

    if (szero_on && sone_on)                  //Have mode_total keep track of what buttons
        mode_multiplier = 0;                //have been pressed.
    else if (sone_on)
        mode_multiplier = 4;                //Add or subtract to mode_total depending on if 
    else if (szero_on)
        mode_multiplier = 2;
    else 
        mode_multiplier = 1;

}


//*******************************************************************************
//***********************************************************************************
//                            ISR(TIMER0_OVF_vect)                                  
// ISR performed when timer 0 overflows. Checks the buttons and the encoders
// when the timer overflows. It then sends to count to the segments
//*******************************************************************************
ISR(TIMER0_OVF_vect){

    static uint16_t count = 1;

    //values to hold the return values of spi_action and encoder_adjuster, respectively.
    uint8_t SPDR_val;
    uint16_t SPDR_adj;

    //make PORTA an input port with pullups 
    DDRA = 0xFC;
    PORTA = 0xFF;

    //enable tristate buffer for pushbutton switches
    uint8_t temp = PORTB;
    PORTB |= 0x70; 

    //now check each button and pass that information to mode_select
    for(int i = 0; i < 2; i++){
        if(debounce_switch(i)){
            mode_select(i);
        }
    }

    //disable tristate buffer for pushbutton switches
    PORTB = temp;

    //bound the count to 0 - 1023
    if(count > 1023){
        count = 1;
    }

    //break up the disp_value to 4, BCD digits in the array: call (segsum)
    DDRA = 0xFF;
    
    //store values of spi_action and encoder_adjuster
    SPDR_val = spi_action(mode_multiplier);
    SPDR_adj = encoder_adjuster(SPDR_val);
    //set count equal to SPDR_adj, which is adjusted with mode_multiplier
    count += SPDR_adj;
    //bound the count to 0 - 1023
    if(count > 1023){
        count = 1;
    }
    //send data out to display
    segsum(count);
}
//*******************************************************************************
uint8_t main() {

    //set port bits 4-7 B as outputs
    DDRB = 0xF7;
    //Initialize SPI
    spi_init();
    //Initialize the timers
    timer_init();
    //Enable the interrupts
    sei();

    while(1){}
}
