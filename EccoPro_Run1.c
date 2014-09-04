/************************

This is a module to program the PIC16LF1827 get data from
the ECCO Pro sensor and output a moisture reading.

First going to get input caputure working with inputs
from a signal generator.

author: Osagie Igbeare
8/7/2014

Modified by: joshuafrancis80@gmail.com
Date: 9/1/2014

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
#pragma config FOSC = XT        // Oscillator Selection (XT Oscillator, Crystal/resonator connected between OSC1 and OSC2 pins)
#pragma config WDTE = ON    // Watchdog Timer Enable (WDT controlled by the SWDTEN bit in the WDTCON register)
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
//static char LCD_Init[6];

static unsigned int uPeriod;

int waterCal = 19667;
int twentyPer = 19235;
int airCal = 18587;
int moisture;
 


/********* Function Prototypes ***************/

void InitPorts(void);
void InitTimers(void);
void InitInterrupts(void);
void WatchDogTimer(void);
//void InitComm(void);
//void NokiaInit(void);
//void Delay(int waitTime);
void SightPin_A2(void);
void SightPin_A3(void);
//void sendByte(char type, char byte);
void MoistureCalc(void);


/******* Acutal Functions ****************/

void InitPorts()
{

	ANSELA = 0x00;			// Port A pins are digital
	ANSELB = 0x00;			// Port B pins are digital

	TRISA = 0b00000000;		// 1 - input, 0 - output, all A pins are outputs 
	TRISB = 0b00000011;		// 1 - input, 0 - output, RB0 is an input, it's the input capture pin

	PORTA = 0b11100110;             //Pins RA7 to RA0, 1 for VIH(>1.5V) and 0 for VIL(<0.5V)
	PORTB = 0b00000000;

	APFCON0 = 0x00;                 // Alternative pin function control register
        //OSCCON = 0b10000010;
	
}

void InitTimers()
{
        T1CON = 0b01110011;             /********************************************
                                         bits <7:6>(TMR1CS) = 01; TMR1 clk source is Fosc
                                         bits <5:4>(T1CKPS) = 11; divide by 8 prescale
                                         bit    3  (T1OSCEN)= 0;  TMR1 oscillator disabled
                                         bit    2  (T1SYNC) = 0; synchronize external clk with
                                                                 sys clock (Fosc)
                                         bit    1           = unimplemented
                                         bit    0   (TMR1ON)= 1; TMR1 on
                                         *******************************************/

        T1GCON = 0b10000100;            /********************************************
                                         bit 7(TMR1GE) = 1; TMR1 contolled by gate function
                                         bit 6(T1GPOL) = 0; TMR1 counts when gate is low
                                         bit 5(T1GTM) = 0; gate toggle is disabled
                                         bit 4(T1GRPM) = 0; single pulse mode disabled
                                         bit 3(T1GGO) = 0; clear when bit 4 is clear
                                         bit 2(T1GVAL) = 1; current state of TMR1 gate
                                         bit <1:0>(T1GSS) = 00; TMR1 gate pin
                                         *******************************************/

        CCP1CON = 0b00110100;            //capture mode: every falling edge
        //CCP1CON = 0b00000100;        //least 4 bits sets capture mode

        T2CON = 0b01111110;		// Fosc / (4 instruct * 16 prescale * 16 postscale * 60 PR2) = 65 Hz
                                         /********************************************
                                         bit 7 unimplemented
                                         bit <6:3>(TxOUTPS) = 1111; 1:16 postscaler
                                         bit 2(TMR2ON) = 1; Timer2 is on
                                         bit <1:0>(T1GSS) = 10; prescaler is 16
                                         *******************************************/

	PR2 = 250;  //Period Register; each clock cycle if TMR2 == PR2 then match signal to output and TMR2 = 00h
}

void WatchDogTimer() {

                         //00010101 about 1 second
    WDTCON = 0b00010101; //00100101 prescale 1:2^23 which is about 4.3 minutes
    /*****************************
     bit 7-6 Unimplemented
     bit 5-1 WDTPS<4:0>: Watchdog Timer Period Select bits
     bit 0 SWDTEN: Software Enable/Disable for Watchdog Timer bit
     *****************************/

}

/* To enable interrupts the following bits of INTCON must be set: GIE and PEIE. And, the interrupt enable bits for
   the specific interrupt events. The following happens when an interrupt event happens and GIE is set: current
   prefetched instruction is flushed, GIE is cleared, PC pushed onto stack, critical registers saved to shadow
   registers, PC is loaded with interrupt vector 0004h.
 */
void InitInterrupts()
{
        //Peripheral Interrupt Enable Register
	PIE1 = 0b00000110; 		
					/********************************************
                                         bit 7(TMR1GIE) Timer1 Gate Interrupt Enable bit
                                         bit 6(ADIE) A/D Converter (ADC) Interrupt Enable bit
                                         bit 5(RCIE) USART Receive Interrupt Enable bit
                                         bit 4(TXIE) USART Transmit Interrupt Enable bit
                                         bit 3(SSP1IE) Synchronous Serial Port 1 (MSSP1)
                                         bit 2(CCP1IE) CCP1 Interrupt Enable bit
                                         bit 1(TMR2IE) TMR2 to PR2 Match Interrupt Enable bit
                                         bit 0(TMR1IE) Timer1 Overflow Interrupt Enable bit
                                         *******************************************/

        CCP1IF = 0;                     // Interrupt request flag bit of the PIR1 register, this is set on capture

	INTCON = 0b11000000;            
                                         /********************************************
                                         bit 7(GIE) = 1; Global Interrupt Enable bit
                                         bit 6(PEIE) = 1; Peripheral Interrupt Enable bit
                                         bit 5(TMR0IE) = 0; Timer0 Overflow Interrupt Enable bit
                                         bit 4(INTE) = 0; INT External Interrupt Enable bit 
                                         bit 3(IOCIE) = 0; Interrupt-on-Change Enable bit
                                         bit 2(TMR0IF) = 0; Timer0 Overflow Interrupt FLag bit
                                         bit 1(INTF) = 0; INT External Interrupt Flag bit
                                         bit 0(IOCIF) = 0; Interrupt-on-Change Interrupt Flag bit
                                         *******************************************/

        //Peripheral Interrupt Request Register 1
        //PIR1 = 0b00000000;
                                         /********************************************
                                         bit 7 TMR1GIF: Timer1 Gate Interrupt Flag bit
                                         bit 6 ADIF: A/D Converter Interrupt Flag bit
                                         bit 5 RCIF: USART Receive Interrupt Flag bit
                                         bit 4 TXIF: USART Transmit Interrupt Flag bit 
                                         bit 3 SSP1IF: Synchronous Serial Port 1 (MSSP1) Interrupt Flag bit
                                         bit 2 CCP1IF: CCP1 Interrupt Flag bit
                                         bit 1 TMR2IF: Timer2 to PR2 Interrupt Flag bit
                                         bit 0 TMR1IF: Timer1 Overflow Interrupt Flag bit
                                         *******************************************/
}

/*
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
*/

/* First need to poll the interrupt flag bits to determine source of interrupt.*/
void interrupt ISR()
{
	counter++;
	if (TMR2IF) // PIR1<1>
	{
		if ((counter % 2) != 0)
		{
			PORTA |= BIT2HI; // 0b00000100

		}
		else
		{
			PORTA &= BIT2LO; // 0b11111011
			counter = 0;
		}

		TMR2IF = 0;		// clears the TIMR2IF (timer 2 interrupt flag)

	}

        if (CCP1IF) // PIR1<2>
        {
            static unsigned int uLastEdge;
            char highByte;
            char lowByte;
            int CCPR1_Snapshot;

            SightPin_A3(); // debugging

            highByte = CCPR1H; // CCPR1H captures value from TMR1H register
            lowByte = CCPR1L; // CCPR1L captures value from TMR1L register
            CCPR1_Snapshot = (highByte<<8) | lowByte; // combine into 16 bit int

            uPeriod = CCPR1_Snapshot - uLastEdge;
            uLastEdge = CCPR1_Snapshot; // this variable should not be in scope next ISR

            //maybe write 0 to the TMR1H and TMR1L bytes to ensure against rollover?

            if (uPeriod == 0)
            {

                uPeriod = 23;
            }
            //twentyPer = uPeriod;

            CCP1IF = 0; // clear the flag


        }

} //pop previous address from the stack, restore registers, set GIE bit

/*
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
 */

 void Delay(int waitTime)
{
	x = 0;
	while(x < waitTime)
	{
            x += 1;
	}
}

 /*
 void sendByte (char type, char byte)
{
        PORTB &= type;           // tell the LCD commands are coming

        PIR1bits.SSP1IF = 0;

	SSP1BUF = byte; 

	while (!SSP1STATbits.BF); 
}
*/

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

    if (uPeriod == 0)
    {
        uPeriod = 0; 
    }


}

/***********************************************************/
/******************** Debugging Library ********************/

void SightPin_A4(void)
{
	// toggles pin A2 for debugging purposes
	if ((PORTA & BIT4HI) == BIT4HI)
	{
		PORTA &= BIT4LO;

	}
	else
	{
		PORTA |= BIT4HI;
	}

}

void SightPin_A3(void)
{
	// toggles pin A3 for debugging purposes
	if ((PORTA & BIT3HI) == BIT3HI)
	{
		PORTA &= BIT3LO;

	}
	else
	{
		PORTA |= BIT3HI;
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

	while(1)
	{

            PORTB &= BIT4LO; //makes LED on RB4 bright
            for (x=0;x<3;x++) {SLEEP();} // will sleep x number of times
            PORTB |= BIT4HI; //makes LED on RB4 dim
            SLEEP();




	}



}


