#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include <string.h>

#define pi 3.141592
#define CAPACITANCE 0.00000000191
//LCD Interface
#define DIGIT_SIZE	4
#define lcd_D7_port	PORTA
#define lcd_D7_bit	PA6
#define lcd_D7_ddr	DDRA

#define lcd_D6_port	PORTA
#define lcd_D6_bit	PA5
#define lcd_D6_ddr	DDRA

#define lcd_D5_port	PORTA
#define lcd_D5_bit	PA4
#define lcd_D5_ddr	DDRA

#define lcd_D4_port	PORTA
#define lcd_D4_bit	PA3
#define lcd_D4_ddr	DDRA

#define lcd_D3_port	PORTA
#define lcd_D3_bit	PA2
#define lcd_D3_ddr	DDRA

#define lcd_D2_port	PORTA
#define lcd_D2_bit	PA1
#define lcd_D2_ddr	DDRA

#define lcd_D1_port	PORTA
#define lcd_D1_bit	PA0
#define lcd_D1_ddr	DDRA

#define lcd_D0_port	PORTB
#define lcd_D0_bit	PB0
#define lcd_D0_ddr	DDRB

#define lcd_E_port	PORTB
#define lcd_E_bit	PB1
#define lcd_E_ddr	DDRB

#define lcd_RS_port	PORTB
#define lcd_RS_bit	PB2
#define lcd_RS_ddr	DDRB

// LCD module information
#define lcd_LineOne     0x00                    // start of line 1
#define lcd_LineTwo     0x40                    // start of line 2
//#define   lcd_LineThree   0x14                  // start of line 3 (20x4)
//#define   lcd_lineFour    0x54                  // start of line 4 (20x4)
//#define   lcd_LineThree   0x10                  // start of line 3 (16x4)
//#define   lcd_lineFour    0x50                  // start of line 4 (16x4)

// LCD instructions
#define lcd_Clear           0b00000001          // replace all characters with ASCII 'space'
#define lcd_Home            0b00000010          // return cursor to first position on first line
#define lcd_EntryMode       0b00000110          // shift cursor from left to right on read/write
#define lcd_DisplayOff      0b00001000          // turn display off
#define lcd_DisplayOn       0b00001100          // display on, cursor off, don't blink character
#define lcd_FunctionReset   0b00110000          // reset the LCD
#define lcd_FunctionSet8bit 0b00111000          // 8-bit data, 2-line display, 5 x 7 font
#define lcd_SetCursor       0b10000000          // set cursor position

// Program ID
volatile uint16_t period = 0;
volatile uint8_t new_period = 0;
volatile uint16_t prev_period = 290;
volatile uint16_t time_1;
volatile uint16_t time_2;
volatile uint8_t captures;

// Function Prototypes
void lcd_write_8(uint8_t);
void lcd_write_instruction_8d(uint8_t);
void lcd_write_character_8d(uint8_t);
void lcd_write_string_8d(uint8_t *);
void lcd_init_8d(void);
void lcd_datalines_init(void);
void tc_init();
char* itoa(int value, char* result, int base);
uint32_t get_period();
/******************************* Main Program Code *************************/
int main(void)
{
   lcd_datalines_init();
// initialize the LCD controller as determined by the defines (LCD instructions)
    lcd_init_8d();                                  // initialize the LCD display for an 8-bit interface

    tc_init();

    while(1)
    {

	    char period_buf[7];
	    //max inductance measured is 18000uH
	if(new_period && period < 290 && period > 1 && (abs(period - prev_period) < (2*prev_period)) )
	{
	    new_period = 0;
	    prev_period = period;
	    double frequency = (1000000.0/period); // Period is in us so frequency is in Hz don't forget *8 for the scaling factor.
	    frequency *= 8;

	    double inductance = 1000000.0/((2*2*pi*pi*frequency*frequency)*CAPACITANCE); // in uH
	    frequency = (uint32_t)frequency;
	    inductance = (uint32_t)inductance;
	    utoa(inductance,period_buf,10);

	    //OUTPUT can't go over 65535
	    // display the first line of information
	    lcd_write_instruction_8d(lcd_SetCursor | lcd_LineOne);

            // set cursor to start of second line
	    lcd_write_instruction_8d(lcd_SetCursor | lcd_LineTwo);
	    _delay_us(80);                                  // 40 uS delay (min)

            // display the second line of information
            lcd_write_instruction_8d(lcd_Clear);            // clear display RAM
            _delay_ms(4);                                   // 1.64 mS delay (min)
	    //concatenate the strings
	    char units[] = " uH";
	    strcat(period_buf, units);
            lcd_write_string_8d(period_buf);
	    _delay_us(80);                                  // 40 uS delay (min)

	    _delay_ms(200);
	}

    }
    return 0;
}
/******************************* End of Main Program Code ******************/

/*============================== 8-bit LCD Functions ======================*/
/*
  Name:     lcd_datalines_init
  Purpose:  initialize the data direction of I/O pins for LCD module for a 8-bit data interface
  Entry:    no parameters
  Exit:     no parameters
*/
void lcd_datalines_init(void)
{
// configure the microprocessor pins for the data lines
    lcd_D7_ddr |= (1<<lcd_D7_bit);                  // 8 data lines - output
    lcd_D6_ddr |= (1<<lcd_D6_bit);
    lcd_D5_ddr |= (1<<lcd_D5_bit);
    lcd_D4_ddr |= (1<<lcd_D4_bit);
    lcd_D3_ddr |= (1<<lcd_D3_bit);
    lcd_D2_ddr |= (1<<lcd_D2_bit);
    lcd_D1_ddr |= (1<<lcd_D1_bit);
    lcd_D0_ddr |= (1<<lcd_D0_bit);

// configure the microprocessor pins for the control lines
    lcd_E_ddr |= (1<<lcd_E_bit);                    // E line - output
    lcd_RS_ddr |= (1<<lcd_RS_bit);                  // RS line - output
}
/*
  Name:     lcd_init_8d
  Purpose:  initialize the LCD module for a 8-bit data interface
  Entry:    equates (LCD instructions) set up for the desired operation
  Exit:     no parameters
  Notes:    uses time delays rather than checking the busy flag
*/
void lcd_init_8d(void)
{
// Power-up delay
    _delay_ms(100);                                 // initial 40 mSec delay

// Reset the LCD controller
    lcd_write_instruction_8d(lcd_FunctionReset);    // first part of reset sequence
    _delay_ms(10);                                  // 4.1 mS delay (min)

    lcd_write_instruction_8d(lcd_FunctionReset);    // second part of reset sequence
    _delay_us(200);                                 // 100uS delay (min)

    lcd_write_instruction_8d(lcd_FunctionReset);    // third part of reset sequence
    _delay_us(200);                                 // this delay is omitted in the data sheet

// Function Set instruction
    lcd_write_instruction_8d(lcd_FunctionSet8bit);  // set mode, lines, and font
    _delay_us(80);                                  // 40uS delay (min)

// The next three instructions are specified in the data sheet as part of the initialization routine,
//  so it is a good idea (but probably not necessary) to do them just as specified and then redo them
//  later if the application requires a different configuration.

// Display On/Off Control instruction
    lcd_write_instruction_8d(lcd_DisplayOff);       // turn display OFF
    _delay_us(80);                                  // 40 uS delay (min)

// Clear Display instruction
    lcd_write_instruction_8d(lcd_Clear);            // clear display RAM
    _delay_ms(4);                                   // 1.64 mS delay (min)

// ; Entry Mode Set instruction
    lcd_write_instruction_8d(lcd_EntryMode);        // set desired shift characteristics
    _delay_us(80);                                  // 40 uS delay (min)

// This is the end of the LCD controller initialization as specified in the data sheet, but the display
//  has been left in the OFF condition.  This is a good time to turn the display back ON.

// Display On/Off Control instruction
    lcd_write_instruction_8d(lcd_DisplayOn);        // turn the display ON
    _delay_us(80);                                  // 40 uS delay (min)
}

/*...........................................................................
  Name:     lcd_write_string_8d
; Purpose:  display a string of characters on the LCD
  Entry:    (theString) is the string to be displayed
  Exit:     no parameters
  Notes:    uses time delays rather than checking the busy flag
*/
void lcd_write_string_8d(uint8_t theString[])
{
    volatile int i = 0;                             // character counter*/
    while (theString[i] != 0)
    {
        lcd_write_character_8d(theString[i]);
        i++;
        _delay_us(80);                              // 40 uS delay (min)
    }
}

/*...........................................................................
  Name:     lcd_write_character_8d
  Purpose:  send a byte of information to the LCD data register
  Entry:    (theData) is the information to be sent to the data register
  Exit:     no parameters
  Notes:    does not deal with RW (busy flag is not implemented)
*/
void lcd_write_character_8d(uint8_t theData)
{
    lcd_RS_port |= (1<<lcd_RS_bit);                 // select the Data Register (RS high)
    lcd_E_port &= ~(1<<lcd_E_bit);                  // make sure E is initially low
    lcd_write_8(theData);                           // write the data
}

/*...........................................................................
  Name:     lcd_write_instruction_8d
  Purpose:  send a byte of information to the LCD instruction register
  Entry:    (theInstruction) is the information to be sent to the instruction register
  Exit:     no parameters
  Notes:    does not deal with RW (busy flag is not implemented)
*/
void lcd_write_instruction_8d(uint8_t theInstruction)
{
    lcd_RS_port &= ~(1<<lcd_RS_bit);                // select the Instruction Register (RS low)
    lcd_E_port &= ~(1<<lcd_E_bit);                  // make sure E is initially low
    lcd_write_8(theInstruction);                    // write the instruction
}

/*...........................................................................
  Name:     lcd_write_8
  Purpose:  send a byte of information to the LCD module
  Entry:    (theByte) is the information to be sent to the desired LCD register
            RS is configured for the desired LCD register
            E is low
            RW is low
  Exit:     no parameters
  Notes:    use either time delays or the busy flag
*/
void lcd_write_8(uint8_t theByte)
{
    lcd_D7_port &= ~(1<<lcd_D7_bit);                        // assume that data is '0'
    if (theByte & 1<<7) lcd_D7_port |= (1<<lcd_D7_bit);     // make data = '1' if necessary

    lcd_D6_port &= ~(1<<lcd_D6_bit);                        // repeat for each data bit
    if (theByte & 1<<6) lcd_D6_port |= (1<<lcd_D6_bit);

    lcd_D5_port &= ~(1<<lcd_D5_bit);
    if (theByte & 1<<5) lcd_D5_port |= (1<<lcd_D5_bit);

    lcd_D4_port &= ~(1<<lcd_D4_bit);
    if (theByte & 1<<4) lcd_D4_port |= (1<<lcd_D4_bit);

    lcd_D3_port &= ~(1<<lcd_D3_bit);
    if (theByte & 1<<3) lcd_D3_port |= (1<<lcd_D3_bit);

    lcd_D2_port &= ~(1<<lcd_D2_bit);
    if (theByte & 1<<2) lcd_D2_port |= (1<<lcd_D2_bit);

    lcd_D1_port &= ~(1<<lcd_D1_bit);
    if (theByte & 1<<1) lcd_D1_port |= (1<<lcd_D1_bit);

    lcd_D0_port &= ~(1<<lcd_D0_bit);
    if (theByte & 1<<0) lcd_D0_port |= (1<<lcd_D0_bit);

// write the data
                                                    // 'Address set-up time' (40 nS)
    lcd_E_port |= (1<<lcd_E_bit);                   // Enable pin high
    _delay_us(1);                                   // implement 'Data set-up time' (80 nS) and 'Enable pulse width' (230 nS)
    lcd_E_port &= ~(1<<lcd_E_bit);                  // Enable pin low
    _delay_us(1);                                   // implement 'Data hold time' (10 nS) and 'Enable cycle time' (500 nS)
}

/*===========================End of 8-bit LCD Functions ======================*/

/*========================== Interrupt Service Routines ======================*/
//ISR variables

ISR(TIM1_CAPT_vect)
{
	time_2 = ICR1;
	period = time_2 - time_1;
	new_period = 1; //set flag to indicate new period
	time_1 = time_2;
}


/*==================== End of Interrupt Service Routines ======================*/
/*
  Name:     input_capture_init()
  Purpose:  initialize the input capture pin
  Entry:    no parameters
  Exit:     no parameters
*/

/*
  Name:     get_period()
  Purpose:  gets the period of the waveform on Input Capture Pin PA7
  Entry:    no parameters
  Exit:     period
*/

/*
  Name:     tc_init()
  Purpose:  gets the period of the waveform on Input Capture Pin PA7
  Entry:    no parameters
  Exit:     period
*/
void tc_init()
{
//	DDRA &= (~(1<<DDA7)); //set as output for detection?
	//Set up Timer Counter 1
	TCCR1A = 0<<WGM10;
	TCCR1B = (1<<ICES1) | (0<<CS10); //Normal mode, rising edge, clock off
	SREG |= 0x80; //Global interrupts
	TIMSK1 = (1<<ICIE1);	//Interrupts on
	//sei();			//global interrupts
	TCCR1B |= (1<<CS10); //start the timer

	ICR1 = 0;
	TCNT1 = 0;

}

