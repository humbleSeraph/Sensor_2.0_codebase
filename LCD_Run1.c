/************************

This is a module to initialize the PIC16LF1827 to communicate with
the Nokia 5110 graphical LCD screen. I'm going to draw something. 

author: Osagie Igbeare

8/7/2014

*************************/

/**************** Header Files *********************/

#include "BitDefs.h"
#include <htc.h>
#include "pic.h"
#include "chip_select.h"
#include "TIMERS12.h"				// from JEC, prof of ME218


/***************** Configuration Macros ***************/

__CONFIG(FCMEN_OFF & IESO_OFF & FOSC_XT & WDTE_OFF & MCLRE_ON & PWRTE_OFF & BOREN_OFF
		& LVP_ON & WRT_OFF & CPD_OFF & CP_OFF);


/***************** # Defines *****************/
#define lcd_data BIT3HI
#define lcd_command BIT3LO

/*************** module level variables ************/

static char counter; 
int currentTime;
int newTime;


/******* Function Prototypes ***************/

void InitPorts(void);
void InitTimers(void);
void InitInterrupts(void);
void InitComm(void);
void NokiaInit(void);
void Delay(int pause); 


/******* Acutal Functions ****************/

void InitPorts()
{

	ANSELA = 0x00;			// Port A pins are digital
	ANSELB = 0x00;			// Port B pins are digital

	TRISA = 0b11111010;		// 1 - input, 0 - output, RA2, RA0 are outputs
	TRISB = 0b10000011;		// 1 - input, 0 - output, RB2 - RB6 is an output

	PORTA = 0b11111111;			// initialize LED to OFF
	PORTB = 0b11111111;

}

void InitTimers()
{

	T2CON = 0b01111110;		// Fosc / (4 instruct * 16 prescale * 16 postscale * 60 PR2) = 65 Hz
	PR2 = 250;

	TMRS12_Init(TMRS12_RATE_2MS);
}

void InitInterrupts()
{

	PIE1 = 0b00001010; 		// Enable TMR2IE, interrupt when Timer 2 matches PR2
							// Bit 3 high - enable MSSP interrupt 	
	INTCON = 0b11000000;	// Enable GIE, Enable PEIE
	
}

void InitComm()
{
	// setup SPI-1 (aka SSP) to communicate with Nokia LCD screen

	SSP1ADD = 0;			// Baud Rate = Fosc / ((SSP1ADD + 1)(4))
							// since Fosc = 4 MHz, Baud Rate = 1 MHz

	SSPSTAT = 0;			// data on rising edge, data @ middle
	WCOL = 0; 	 			// no collision
	SSPOV = 0; 				// no overflow
	SSP1EN = 1; 			// SSP Enable

	CKP = 1; 				// idle high 
	CKE = 1;				// sample even edges 

	SSP1M3 =  0;			// Set LF1827 as Master, clock rate Fosc / 4
	SSP1M2 = 0;
	SSP1M1 = 0; 
	SSP1M0 = 0; 



}

void Delay(int pause)
{
	currentTime = TMRS12_GetTime(); 
	newTime = currentTime;
	while (newTime - currentTime < pause){
		newTime = TMRS12_GetTime();
	}
		time2 = newTime - currentTime; 
		//printf("%d\r\n", time2); 
}

void NokiaInit()
{
	// initialization sequence for the PCD8544 driver on the Nokia LCD
	// beginning with RESET 

	PORTA &= BIT0LO;
	Delay(1);
	PORTA |= BIT0HI;

	PORTB &= lcd_command;	// tell LCD commands are coming 

	SSPBUF = 0x21;			// tell LCD extended commands to follow
	SSPBUF = 0xE0;			// set LCD Vop (contrast) ** parameter to mess with if screen doesn't display **** 
	SSPBUF = 0x04;			// set temp coefficient
	SSPBUF = 0x14; 			// LCD Bias mode 1:48 (if not working, try 0x13)

	SSPBUF = 0x20;			// back to regular commands
	SSPBUF = 0x0C; 			// enable normal display (dark on light), horiz addressing

}

void interrupt ISR()
{

	counter++; 
	if (TMR2IF)
	{
		if ((counter % 2) != 0)
		{
			PORTA |= BIT2HI;
			PORTB &= BIT6LO;
			
		}
		else 
		{
			PORTA &= BIT2LO;
			PORTB |= BIT6HI;
			counter = 0; 
		}

		TMR2IF = 0;		// clears the TIMR2IF (timer 2 interrupt flag)

	}
	return; 

}



/******** Main - which actually runs the code ***********/

void main ()
{

	// Initializing PIC16LF1827
	InitPorts();
	InitTimers();
	InitInterrupts();
	InitComm(); 
	NokiaInit();
	while(1)
	{

	}



}


