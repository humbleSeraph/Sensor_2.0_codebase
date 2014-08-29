/************************

This is a module to program the PIC16LF1827 get data from
the ECCO Pro sensor and output a moisture reading.

First going to get input caputure working with inputs
from a signal generator.

author: Osagie Igbeare

8/7/2014

*************************/

/******************* Pin Configuration *************

 Nokia 5110 LCD ----- < > ----- PIC16LF1827
    1-Vcc       ---------------     3.3V
    2-GND       ---------------     GND
    3-SCE       ---------------     RB5
    4-RST       ---------------     RA0
    5-D/C       ---------------     RB3
    6-DN(MOSI)  ---------------     RB2
    7-SCLK      ---------------     RB4
    8-LED       - 330 ohm res -     3.3V

 ***************************************************/

/**************** Header Files *********************/

#include "BitDefs.h"
#include <xc.h>
#include "pic.h"
#include "chip_select.h"



/***************** Configuration Macros ***************/

//__CONFIG(FCMEN_OFF & IESO_OFF & FOSC_LP & WDTE_OFF & MCLRE_ON & PWRTE_OFF & BOREN_OFF
//		& LVP_ON & WRT_OFF & CPD_OFF & CP_OFF);

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

// CONFIG1
#pragma config FOSC = LP        // Oscillator Selection (LP Oscillator, Low-power crystal connected between OSC1 and OSC2 pins)
#pragma config WDTE = OFF       // Watchdog Timer Enable (WDT disabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable (PWRT disabled)
#pragma config MCLRE = ON       // MCLR Pin Function Select (MCLR/VPP pin function is MCLR)
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Memory Code Protection (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown-out Reset Enable (Brown-out Reset disabled)
#pragma config CLKOUTEN = ON    // Clock Out Enable (CLKOUT function is enabled on the CLKOUT pin)
#pragma config IESO = OFF       // Internal/External Switchover (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is disabled)

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config PLLEN = OFF      // PLL Enable (4x PLL disabled)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
#pragma config LVP = ON         // Low-Voltage Programming Enable (Low-voltage programming enabled)


/***************** # Defines *****************/
#define lcd_data BIT3HI
#define lcd_command BIT3LO
#define hangTime 1000


/*************** module level variables ************/

static char counter; 
int x;
int i = 0; 
static char LCD_Init[6];

static unsigned int uPeriod;

int waterCal = 19667;
int twentyPer = 19235;
int airCal = 18587;
int moisture; 


/********* Function Prototypes ***************/

void InitPorts(void);
void InitTimers(void);
void InitInterrupts(void);
void InitComm(void);
void NokiaInit(void);
void Delay(int waitTime); 
void SightPin_B0(void);
void sendByte(char type, char byte);
void MoistureCalc(void);


/******* Acutal Functions ****************/

void InitPorts()
{

	ANSELA = 0x00;			// Port A pins are digital
	ANSELB = 0x00;			// Port B pins are digital

	TRISA = 0b00000000;		// 1 - input, 0 - output, RA2, RA0 are outputs
	TRISB = 0b00000011;		// 1 - input, 0 - output, RB0 is an input, it's the input capture pin

	PORTA = 0b11111110;             //RA0 - low
	PORTB = 0b11111111;

	APFCON0 = 0x00;
        //OSCCON = 0x00;
	
}

void InitTimers()
{
        T1CON = 0b00110011;             /********************************************
                                         bits <7:6>(TMR1CS) = 00; TMR1 clk source is instruction clk
                                         bits <5:4>(T1CKPS) = 11; divide by 8 prescale
                                         bit    3  (T1OSCEN)= 0;  TMR1 oscillator disabled
                                         bit    2  (T1SYNC) = 0; synchronize external clk with
                                                                 sys clock
                                         bit    1           = unimplemented
                                         bit    0   (TMR1ON)= 1; TMR1 on
                                         *******************************************/

        T1GCON = 0b10000100;            /********************************************
                                         bit 7(TMR1GE) = 1; TMR1 contolled by gate function
                                         bit 6(T1GPOL) = 0; TMR1 gate active low
                                         bit 5(T1GTM) = 0; gate toggle is disabled
                                         bit 4(T1GRPM) = 0; single pulse mode disabled
                                         bit 3(T1GGO) = 0; clear when bit 4 is clear
                                         bit 2(T1GVAL) = read only
                                         bit <1:0>(T1GSS) = 00; TMR1 gate pin
                                         *******************************************/

        CCP1CON = 0b0110100;            //capture mode: every falling edge


        T2CON = 0b01111110;		// Fosc / (4 instruct * 16 prescale * 16 postscale * 60 PR2) = 65 Hz
	PR2 = 1;
}

void InitInterrupts()
{

	PIE1 = 0b00000110; 		// Enable TMR2IE, interrupt when Timer 2 matches PR2
					// Enable CCP1 interrupt

	INTCON = 0b11000000;            // Enable GIE, Enable PEIE
	
}


void InitComm()
{
	// setup SPI-1 (aka SSP) to communicate with Nokia LCD screen

	SSP1ADD = 2;			// Baud Rate = Fosc / ((SSP1ADD + 1)(4))
							// since Fosc = 4 MHz, Baud Rate = 1 MHz
							// since SSP1ADD = 0 is not supported same timing is achieved
							// by setting bits <3:0> of SSPM all to 0's (see below)

	SSP1STATbits.SMP = 0;				// data on rising edge, data @ middle
	SSP1CON1bits.WCOL = 0; 	 			// no collision
	SSP1CON1bits.SSPOV = 0; 			// no overflow
	SSP1CON1bits.SSPEN = 1; 			// SSP Enable

	SSP1CON1bits.CKP = 1; 				// idle high 
	SSP1STATbits.CKE = 0;				// sample ? edges 

	SSP1CON1bits.SSPM3 =  0;			// Set LF1827 as Master, clock rate Fosc / 4
	SSP1CON1bits.SSPM2 = 0;
	SSP1CON1bits.SSPM1 = 0; 
	SSP1CON1bits.SSPM0 = 0;
       

}


void interrupt ISR()
{
	counter++;
	if (TMR2IF)
	{
		if ((counter % 2) != 0)
		{
			PORTA |= BIT2HI;

		}
		else
		{
			PORTA &= BIT2LO;
			counter = 0;
		}

		TMR2IF = 0;		// clears the TIMR2IF (timer 2 interrupt flag)

	}

        if (CCP1IF)
        {
            static unsigned int uLastEdge;
            char highByte;
            char lowByte;
            int CCPR1_Snapshot;

            highByte = CCPR1H;
            lowByte = CCPR1L;
            CCPR1_Snapshot = (highByte<<8) | lowByte;

            uPeriod = CCPR1_Snapshot - uLastEdge;
            uLastEdge = CCPR1_Snapshot;

            CCP1IF = 0; 


        }

}

void NokiaInit()
{

	//i = 0;
	// LCD_Init array populated with initialization sequence

	LCD_Init[0] = 0x21;			// tell LCD extended commands to 
	LCD_Init[1] = 0xE0;			// set LCD Vop (contrast) ** parameter to mess with if screen doesn't display ****
	LCD_Init[2] = 0x04;			// set temp coefficient
	LCD_Init[3] = 0x13;			// LCD Bias mode 1:48 (if not working, try 0x13)
	LCD_Init[4] = 0x20;			// back to regular commands
	LCD_Init[5] = 0x0C;			// enable normal display (dark on light), horiz addressing


	// initialization sequence for the PCD8544 driver on the Nokia LCD
	// starting off with RESET
	
	PORTA &= BIT0LO;
	Delay(hangTime); 
	PORTA |= BIT0HI;

        // the rest of the initialization sequence
	
	//PORTB &= lcd_command;           // tell LCD commands are coming
	
	PORTB &= BIT5LO;		// lower SCE line to begin transmission
	
	for (i=0; i<6; i++)             // sending 6 commands to the LCD screen
	{
		sendByte(lcd_command, LCD_Init[i]);
	}	

	PORTB |= BIT5HI;		// raising SCE line at the end of transmission0
}

void Delay(int waitTime)
{
	x = 0;
	while(x < waitTime)
	{
            x += 1;
	}
}

void sendByte (char type, char byte)
{
        PORTB &= type;           // tell the LCD commands are coming

        PIR1bits.SSP1IF = 0;

	SSP1BUF = byte; 

	while (!SSP1STATbits.BF); 


}

void MoistureCalc(void)
{
    int x;

    if (uPeriod < airCal)
    {
        x = airCal;
        
        moisture = ((uPeriod - airCal)*x )/((twentyPer - airCal)*100);

    }

    if (uPeriod <= twentyPer)
    {
        x = ((uPeriod - airCal)*100)/(twentyPer - airCal);

        moisture = ((uPeriod - airCal)*x )/((twentyPer - airCal)*100);
    }


    if (uPeriod > twentyPer)
    {
        x = ((uPeriod - twentyPer)*50)/((waterCal-twentyPer)+100);

        moisture = ((uPeriod - twentyPer)*(x-100))/ ((waterCal-twentyPer)*50);

    }

    if (uPeriod > waterCal)
    {
        x = waterCal;

        moisture = ((uPeriod - twentyPer)*(x-100))/ ((waterCal-twentyPer)*50);

    }


}

/***********************************************************/
/******************** Debugging Library ********************/

void SightPin_B0(void)
{
	// toggles pin B0 for debugging purposes
	if ((PORTB & BIT0HI) == BIT0HI)
	{
		PORTB &= BIT0LO;

	}
	else
	{
		PORTB |= BIT0HI; 
	}

}


/********************************************************/
/******** Main - which actually runs the code ***********/
/********************************************************/

void main ()
{

	// Initializing PIC16LF1827
	InitPorts();
	InitTimers();
	InitInterrupts();
	InitComm(); 
	//NokiaInit();
	while(1)
	{

            NokiaInit(); 
	}



}


