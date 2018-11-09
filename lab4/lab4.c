// lab4.c 
// Jesus Narvaez
// 11.8.18

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

//holds data to be sent to the segments. logic zero turns segment on
uint8_t segment_data[5]; 
//variable holding the current mode
enum mode {SET_ALARM, SET_CLOCK, SET_VOL, ALARM_HANDLER};
static enum mode current_mode = SET_VOL;
//variables for storing time
static uint16_t seconds = 1;
static uint16_t minutes = 0;
static uint16_t hours = 0;
static uint8_t  alarm_sounding = FALSE;
static int8_t alarm_seconds = 10;
static int8_t alarm_minutes = 10;
static int8_t alarm_hours = 10;
static int8_t clock_minutes = 0;
static int8_t clock_hours = 0;
static int8_t alarm_is_set = 0;
//*******************************************************************************
//                            timer0_init                                  
// Initialize the timer to be used to count up 1 second and run off the external
// 32kHz oscillator.
//*******************************************************************************
void timer0_init(){
    //Initialization of timer 0 using ext. oscillator. Used to count seconds.
    TIMSK |= (1<<TOIE0);             //enable overflow interrupt
    TCCR0 |= (1<<CS00) | (1<<CS02);  //normal mode, 128 prescale
    ASSR |= (1<<AS0);                //Use external oscillator
}
//*******************************************************************************
//*******************************************************************************
//                            timer1_init                                  
// Oscillator used to make alarm sound. Sent out to PC0 to OP amp to eventually be
// audio output. Uses internal I/O clock with no prescale.
//*******************************************************************************
void timer1_init(){
    TIMSK |= (1<<TOIE1);  //enable timer overflow and enable output compare A interrupts
    TCCR1A |= (1<<WGM11) | (1<<WGM10);  //Set to fast PWM, 64 prescale 
    TCCR1B |= (1<<WGM12) | (1<<CS10) | (1<<CS11);              
    TCCR1C = 0x00;                      //No force compare
}
//*******************************************************************************
//*******************************************************************************
//                            timer2_init                                  
// Used in PWM mode to adjust brightness of display. Reads in ADCH to determine
// duty cycle of PB7.
//*******************************************************************************
void timer2_init(){
    //TIMSK |= (1<<OCIE2);             //output compare interrupt enabled
    TCCR2 |= (1<<WGM20) |(1<<WGM21) | (1<<CS20) | (1<<COM21);  //Set to fast PWM, 1024 prescale
}
//*******************************************************************************
//*******************************************************************************
//                            timer3_init                                  
// Controls volume of audio output. Connected to VOLUME pin on audio amplifier
// and varies a voltage on the pin to increase or decrease the volume.
//*******************************************************************************
void timer3_init(){
    //TIMSK |= (1<<TOIE1) | (1<<OCIE1A);  //enable timer overflow and enable output compare A interrupts
    TCCR3A |= (1<<COM3A1) | (1<<COM3A0) | (1<<WGM31) | (1<<WGM30);  //Set to fast PWM, no prescale 
    TCCR3B |= (1<<WGM32) | (1<<CS30);              
    TCCR3C = 0x00;                      //No force compare
    OCR3A = 9727; 
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
//                            adc_init()                                  
// Initialize the analog to digital converter so we receive input on PF0 to send to 
// TCNT0.
//*******************************************************************************
void adc_init(){
    ADMUX |= (1<<REFS1) | (1<<REFS0);   //Use the internal 2.56V as AREF and left adjust
    ADCSRA |= (1<<ADEN) | (1<<ADIE) | (1<<ADSC) | (1<<ADPS2) | (1<<ADPS0) | (1<<ADPS0);   //Enable ADC and ADC interrupts
                                                                //Start the conversion and prescale by 128                    
}
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
int8_t blink_colon() {
    
    //Declare boolean to determine whether the colon is on
    
    //Check to see if the colon is on and either turn on segments
    //or blank segments.
    //Toggle colon_on before leaving function
    if (seconds % 2 == 0){
        return segment_data[2] = 11;
    }
    else if (seconds % 2 == 1){
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
void segsum(uint8_t seconds, uint8_t minutes) {

    uint8_t result, i;

    //break up decimal sum into 4 digit-segments
    for (i = 0; i < 5; i++){
        if (i < 2){
            //get the last digit of the current
            result = (seconds % 10);
            //place that digit into the segment array
            segment_data[i] = result;
            //divide sum by 10 to get to next one's digit
            seconds = (seconds / 10);
        }
        else if (i > 2){
            //get the last digit of the current
            result = (minutes % 10);
            //place that digit into the segment array
            segment_data[i] = result;
            //divide sum by 10 to get to next one's digit
            minutes = (minutes / 10);
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
    static uint8_t CW = TRUE;
    static uint8_t dir_count = 0;

    //for loop checks both pairs of bits in the nibble for the encoders
    for (int i = 0; i < 2; i++){
        //get either the high or low bits from the encoder
        uint8_t current_bits = ((enc_val >> 2*i) & 0x03);
        switch((prev_enc_val >> 2*i) & 0x03){
            //compare the previous bits to the current bits to
            //see if we are incrementing or decrementing

            case 0b00: if (current_bits == 0b01){ 
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
//                            alarm_set                                  
// Takes in a time, in seconds, that will pass until the alarm will go off..
//*******************************************************************************

void alarm_set (uint8_t secs, uint8_t mins, uint8_t hrs){
    static int8_t count = 0;
    
    for (int i = 0; i < mins; i++){
        secs += 60;
    }

    for (int j = 0; j < hrs; j++){
        secs += 3600;
    }
    
    secs--;
    alarm_is_set = secs;
}

//*******************************************************************************
//                            mode_select                                  
// Determines the current mode by checking which button was pressed recently pressed.
// Returns an enum.
//*******************************************************************************
void mode_select(int button, enum mode cur_mode){

    
    //Test to see what button was pressed
    switch (button) {
        
        case 0: //SET_CLOCK handler 
            if (cur_mode == SET_VOL){            
                TIMSK &= ~(1<<TOIE0);
                current_mode = SET_CLOCK;
            } 
            else if (cur_mode == SET_CLOCK){            
                TIMSK |= (1<<TOIE0);
                current_mode = SET_VOL;
            }
            break;
        case 1: //SET_ALARM handler
            if (cur_mode == SET_VOL){            
                current_mode = SET_ALARM;
            } 
            else if (cur_mode == SET_ALARM){            
                current_mode = SET_VOL;
            } 
            break;
        case 2: //snooze handler 
            if (cur_mode == ALARM_HANDLER){
                //set alarm alarm time to 10 seconds
                alarm_seconds = seconds + 10;
                alarm_minutes = minutes;
                alarm_hours = hours;
                ////alarm_set(alarm_seconds, alarm_minutes, alarm_hours);
                //continue to ALARM_HANDLER to disable alarm
                current_mode = ALARM_HANDLER;
           }
           else if (cur_mode != ALARM_HANDLER)
                current_mode = SET_VOL;
            break;
        case 3: //alarm off handler
            if (cur_mode == ALARM_HANDLER){
                current_mode = ALARM_HANDLER;
            }
            else if (cur_mode != ALARM_HANDLER) 
                current_mode = SET_VOL;
            break;
       
       default: //SET_VOL is the default setting
            current_mode = SET_VOL;
            break;
    }
}

//*******************************************************************************
//                            clock_set                                  
// Takes in a time, in seconds, that will be sent off to become the displayed time
//*******************************************************************************

void clock_set (uint8_t mins, uint8_t hrs){
    
    minutes = mins;
    hours = hrs;

}
//***********************************************************************************
//                            ISR(TIMER0_OVF_vect)                                  
// ISR performed when timer 0 overflows. Determines when 1 second has
// passed. Responsible for blinking colon after every second and keeping
// track of the 24hr clock. 
//*******************************************************************************
ISR(TIMER0_OVF_vect){

    seconds++;
    
    //Increment seconds, minutes and hours when appropriate
    //clear each when they reach 60, 60 and 24 respectively.
    if (seconds > 59){
        minutes++;
        seconds = 0;
    }

    if (minutes > 59){
        hours++;
        minutes = 0;
    }

    if (hours > 23){
        hours = 0;
    }
    
    //blink the colon
    blink_colon();
    
    //compare the current time to the set alarm
    if((alarm_hours == hours) && (alarm_minutes == minutes) && (alarm_seconds == seconds));
        alarm_sounding = TRUE;
}
//***********************************************************************************
//                            ISR(TIMER1_OVF_vect)                                  
// Used in fast PWM mode to oscillate PC0 to sent alarm tone. 
//*******************************************************************************
ISR(TIMER1_OVF_vect){
   if (alarm_sounding){ 
        PORTC ^= (1<<PC0);
    }
    else{
        PORTC &= ~(1 <<PC0);
        }
}
//***********************************************************************************
//                            ISR(ADC_vect)                                  
// Used to extract ADC result and store in adc_result. 
//*******************************************************************************
ISR(ADC_vect){
    //Set OCR2 to the ADC value divided by 4 to get more precision
    OCR2 = (ADC / 4);
    //Enable the next conversion
    ADCSRA |= (1<<ADSC);
}
//***********************************************************************************
//                            vol_adjust()                                  
// Adjust the volume of the alarm using the encoders.
//*******************************************************************************
void vol_adjust(int8_t encoder_change){
    
    //values to hold the return values of spi_action and encoder_adjuster, respectively.
    //uint16_t upper_limit = 10251;       
    //uint16_t lower_limit = 100;       
    //uint16_t fifty_duty = 9727;       
    
    //multiply encoder_change by 32.75 to match the voltage steps for the
    //audio amp.
    encoder_change = encoder_change * 33; 
   /* 
    if(OCR3A + encoder_change > upper_limit) {
        OCR3A = upper_limit;
    }
    else if(OCR3A + encoder_change < lower_limit) {
        OCR3A = lower_limit;
    }
    */
    OCR3A += encoder_change;
    /*
    if(OCR3A > upper_limit){
        OCR3A = upper_limit;
    }
    else if (OCR3A < lower_limit){
        OCR3A = lower_limit;
    }

   else 
        OCR3A += (encoder_change);
        */
}
//***********************************************************************************
//                            mode_action()                                  
// Depending on what the current mode is, we do the appropriate action.
//*******************************************************************************
void mode_action(enum mode cur_mode, uint8_t encoder_change){
    switch (cur_mode){
        case 0: //setting an alarm
            alarm_minutes += encoder_change;
            if (alarm_minutes > 59){
                alarm_hours++;
                alarm_minutes = alarm_minutes % 60; 
            }
            clear_display();
            string2lcd("ALARM SET");
           // segsum(alarm_minutes, alarm_hours);
            break;
        case 1: //setting a new time
            clock_minutes += encoder_change;
            if (clock_minutes > 59){
                clock_hours++;
                clock_minutes = clock_minutes % 60;
            }
            clear_display();
            string2lcd("TIME SET");
            clock_set(clock_minutes, clock_hours);
           // segsum(alarm_seconds, alarm_minutes);
            break;
        case 2: //adjusting the volume
            vol_adjust(encoder_change);
            clear_display();
            string2lcd("VOLUME ADJUST");
            break;
        case 3: //alarm handling
            TIMSK &= ~(1<<TOIE1);
            clear_display();
            string2lcd("ALARMED DISABLED");
    }
}
//***********************************************************************************
uint8_t main() {

    //set port bits 4-7 B as outputs
    DDRB = 0xF7;
    //Set PC0 to output
    DDRC = (1<<PC0); 
        
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
    //Enable the interrupts
    sei();

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
        for(int i = 0; i < 4; i++){
            if(debounce_switch(i)){
               mode_select(i, current_mode);
            }
        }

        //disable tristate buffer for pushbutton switches
        PORTB = temp;

        //break up the disp_value to 4, BCD digits in the array: call (segsum)
        DDRA = 0xFF;

        //store values of spi_action and encoder_adjuster
        SPDR_val = spi_action(current_mode);
        SPDR_adj = encoder_adjuster(SPDR_val);
        
        
        mode_action(current_mode, SPDR_adj);

        //send data out to display
        if (current_mode == SET_CLOCK) 
            segsum(clock_minutes, clock_hours);
        else if (current_mode == SET_VOL) 
            segsum(minutes, hours);
        else if (current_mode == SET_ALARM) 
            segsum(alarm_minutes, alarm_hours);
    }
}
