// mykeyer.c    
// R. Traylor
// 12.19.2010
// iambic keyer      

#define F_CPU 8000000UL     //8Mhz clock
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>

//Note: ATmega48 is supplied with 8 Mhz RC internal clock enabled with
//clkdiv programmed which sets internal clock to 1Mhz. 

// I/O allocation 
//ADC  analog pin to read the speed value 
//PC0   when low, dot paddle closed
//PC1   when low, dash paddle closed 
//PC2   drives the driver xistor that closes the key connection
//PC3   mute2, relay output, when high, xmitter is connected to antenna
//PC4   mute1, mute output, when high, reciever is muted
//PC5   spare, unused

//PD0   encoder a pin 
//PD1   encoder b pin 
//PD3   pin 1, OC2B compare output for sidetone

//main keyer state machine states
#define IDLE     0  //waiting for a paddle closure 
#define DIT      1  //making a dit 
#define DAH      2  //making a dah  
#define DIT_DLY  3  //intersymbol delay, one dot time
#define DAH_DLY  4  //intersymbol delay, one dot time
//
#define FALSE    0
#define TRUE     1
//

//define EEPROM storage areas
uint8_t EEMEM eeprom_wds_per_min = 20;    //this value only assigned during programming

volatile uint8_t  keyer_state   = IDLE;   //the keyer state 
volatile uint8_t  timer_ena     = 0;      //the timer enable 
volatile uint8_t  dit_pending   = FALSE;  //memory for dits  
volatile uint8_t  dah_pending   = FALSE;  //memory for dahs  
volatile uint16_t timeout       ;         //one dot interval  
volatile uint16_t half_timeout  ;         //one half dot interval 
volatile uint8_t  key           = FALSE;  //internal keyer output 
volatile int16_t  ee_wait_cnt   = -1;     //countdown to save new setting to eeprom

volatile uint8_t  wds_per_min = 20;       //words per minute (still need to intalize if in EEPROM?)

//output state machine states 
#define IDLE   0  //
#define A      1  //
#define B      2  //
#define C      3  //
#define D      4  //
#define E      5  //
#define F      6  //
#define G      7  //
#define H      8  //
#define I      9  //

//output state machine variables
volatile uint8_t output_state  = IDLE;
volatile uint8_t tx_dly        = 0x00;
volatile uint8_t mute1_timeout    = 4;  //500uS for audio mute to engage
volatile uint8_t relay_timeout    = 31; //4mS for relay to actuate
volatile uint8_t tx_decay         = 39; //5mS for xmit envelope to decay

//functions
void    tx_on()  {PORTC &= ~0b00000100;}           //asserts key output
vOID    TX_OFF() {porTC |=  0b00000100;}           //deasserts key output
uint8_t dit_on() {return(bit_is_clear(PINC, 0));}  //returns non-zero if dit paddle on 
uint8_t dah_on() {return(bit_is_clear(PINC, 1));}  //returns non-zero if dah paddle on 

/*****************************  mute1  ****************************************/
void mute1(state){if(state==TRUE){PORTC |=  (1<<PC4);}
                  else           {PORTC &= ~(1<<PC4);}
}
/******************************************************************************/

/****************************  mute2  *****************************************/
void mute2(state){if(state==TRUE){PORTC |=  (1<<PC3);}
                  else           {PORTC &= ~(1<<PC3);}
}
/******************************************************************************/

/******************************************************************************/
//                             encoder_chk                                   
//Checks for encoder movement.  Debounces encoder and returns a one exactly once.
//for each falling edge detected on PD0.  Takes 12 cycles to determine movement. 
//Note: Returns signed value.
//
Int8_t encoder_chk() {

static uint16_t state = 0;  //holds shifted in bits from encoder

  state = (state << 1) | (! bit_is_clear(PIND, PD0)) | 0xE000;     
  if(state == 0xF000){                     //if a falling edge is detected on A output
    //get state of "B" to determine direction
    if (bit_is_set(PIND, PD1)){return 1;} //detected CW rotation
    else                      {return 0;} //detected CCW rotation
  }
  else{return -1;}                        //no movement detected
}//encoder_chk
/******************************************************************************/

//*****************************************************************************/
//                  Interrupt service routine for timer 0 
//worst case is about 18uS to run ISR

ISR(TIMER0_COMPA_vect){

static uint16_t timer;

  timer--;  //decrement clock 

//keyer main state machine   
  switch(keyer_state){ //see if user changed the minutes setting
    case (IDLE) : 
      key = FALSE;
      if     (dit_on()){timer = timeout;   keyer_state = DIT;}
      else if(dah_on()){timer = timeout*3; keyer_state = DAH;}       
      break;
    case (DIT) :
      key = TRUE; 
      if(!timer){timer = timeout; keyer_state = DIT_DLY;}  
      break;
    case (DAH) : 
      key = TRUE; 
      if(!timer){timer = timeout; keyer_state = DAH_DLY;}  
      break;
    case (DIT_DLY) :
      key = FALSE;  
      if(!timer){
        if(dah_pending == TRUE) {timer=timeout*3; keyer_state = DAH;}
        else                    {keyer_state = IDLE;}
      }
      break; 
    case (DAH_DLY) : 
      key = FALSE; 
      if(!timer){
        if(dit_pending == TRUE) {timer=timeout; keyer_state = DIT;}
        else                    {keyer_state = IDLE;}
      }
      break; 
  }//switch keyer state

//*****************  dit pending main state machine   *********************
  switch(dit_pending){ //see if a dot is pending 
    case (FALSE) : 
      if( (dit_on() && (keyer_state == DAH)     & (timer < timeout / 3))   ||
          (dit_on() && (keyer_state == DAH_DLY) & (timer > half_timeout))) 
        { dit_pending = TRUE;}
      break;
    case (TRUE) : 
      if(keyer_state == DIT){dit_pending = FALSE;}
      break;
  }//switch dit_pending
         
//******************  dah pending main state machine   *********************
  switch(dah_pending){ //see if a dah is pending 
    case (FALSE) : 
      if( (dah_on() && (keyer_state == DIT)     & (timer < half_timeout))  ||
          (dah_on() && (keyer_state == DIT_DLY) & (timer > half_timeout)))
         {dah_pending = TRUE;}
      break;
    case (TRUE) : 
      if(keyer_state == DAH){dah_pending = FALSE;}
      break;
  }//switch dah_pending


//****************** state machine for the output sequencer *********
  tx_dly--;  //decrement the delay counter for the output sequencer
  switch(output_state){ 
    case (IDLE) : 
               if(key==TRUE){ tx_dly = mute1_timeout; output_state=A;} //delay from mute1 to relay on
               break; 
    case (A) : mute1(TRUE); 
               if(!tx_dly){tx_dly=relay_timeout; output_state=B;} //delay from mute2 to relay on
               break; 
    case (B) : mute2(TRUE); 
               if(!tx_dly){output_state=C;} //wait for relay
               break; 
    case (C) : if(key==FALSE){tx_dly=mute1_timeout+relay_timeout+1; output_state=D;} //equalize key low 
               else{tx_on(); TCCR2B = (1<<CS22) | (1<<CS20);}  //tx_on and sidetone on 
               break;
    case (D) : if(!tx_dly){tx_dly=tx_decay; output_state=E;}  
               break;
    case (E) : tx_off(); TCCR2B = 0x00;  //tx off and sidetone off 
               if(!tx_dly){tx_dly=mute1_timeout; output_state = F;}
               break; 
    case (F) : mute2(FALSE);
               if(!tx_dly){output_state=G;}//let transmitter die out 
               break;
    case (G) : mute1(FALSE); 
               output_state = IDLE;   //unmute audio
               break; 
  }//switch output state  

//********** check encoder for movement and adjust speed accordingly  *****************
  switch(encoder_chk()){                              //see if user changed the speed setting
    case (0) : if(wds_per_min > 5){wds_per_min--;    }//check, bound and possibly decrement speed
               else               {wds_per_min = 5;  }
               ee_wait_cnt=32760;                     //countdown to save new setting to eeprom
               break;  //decrease speed
    case (1) : if(wds_per_min < 60){wds_per_min++;   }//check, bound and possibly increment speed
               else                {wds_per_min = 60;} 
               ee_wait_cnt=32760;                     //countdown to save new setting to eeprom
               break;                                 //increase speed
    case (-1): break;                                 //no change
    } //switch 

//*************** see if we need to save the present speed setting **************
  if(ee_wait_cnt>0){ee_wait_cnt--;}  //keep counting down till its time to write
  else if(ee_wait_cnt==0){           //upon reaching zero, we save the setting
    eeprom_write_byte(&eeprom_wds_per_min, wds_per_min);  //save the setting 
    ee_wait_cnt--;                   //decrement to -1 to prevent more writes 
  }
} //**************** end of ISR for TIMER1_COMPA_vect ***************************


//********************** startup initalization code ****************************
void setup()                    // run once, at startup 
{
//initalize the pins
  DDRC  |= (1<<PC2) | (1<<PC3) | (1<<PC4); //keyout, mute, relay outputs
  PORTC |= (1<<PC0) | (1<<PC1);            //dot and dash inputs have pullups on

  DDRD |= 0x48; //PD6 is the debug output pin, PD3 is the sidetone pin
  PORTD = 0x03; //PD0 and 1 are encoder input pins, turn pullups on

//Initalize TCNT2 to generate an 800hz sidetone 
  TCCR2A = (1<<COM2B0) | (1<<WGM21); //CTC, wiggle OC2B (PD3, pin 1), TOP=OCR2A 
  TCCR2B = 0x00; //to enable tone, set CS22, CS20 to one; divide by 128 
  OCR2A   = 0x26;                    //tone is about 800Hz 

//Initalize TCNT0 to generate the state machine interrupts 
  TCCR0A = (1<<WGM01);               //CTC mode, no pins wiggle, TOP in OCR0A 
  TCCR0B = (1<<CS01) | (1<<CS00);    //divide clk by 64
  OCR0A   = 0x0F;                    //interrupt every 128uS 
  TIMSK0 = (1<<OCIE0A);              //interrupt on OCR1A match 

wds_per_min = eeprom_read_byte(&eeprom_wds_per_min); //restore previous speed setting 
sei(); //enable interrupts
}//setup
//********************** startup initalization code ****************************

//****************************  main code loop  ********************************
int main(void) { 
  setup();   //do setup of timers, ports interrupts
  tx_off();  //make sure xmitter is off
  while(1){  //spin forever and update dot period blindly
    timeout = (9375/wds_per_min); //1 dot time = ((1200/wds_per_min)*1000)/128
    half_timeout = (timeout >> 1);
  } //do forever
}//main	

