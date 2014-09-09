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
#define total_values 100 //this is the total number of values that will be saved in the array

/*************** module level variables ************/

static char counter; 
int x, y;
int i = 0;
int captureTracker = 0;

//int total_values = 24; //set the total number of values to capture
int moistureValues[total_values]; //array for moisture values
int moistureChangeRate[total_values]; //array for change in moisture values

static unsigned int uPeriod;
static unsigned int rawInterval; 

int waterCal = 19667; //where does this number come from?
int twentyPer = 19235; //where does this number come from?
int airCal = 18587; //where does this number come from?
int moisture = 0; //initialized to zero, maybe this should be unsigned
int previous_moisture = 0; //initialized to zero
int target_value = 20; //I set this arbitrarily to test the code
 

/********* Function Prototypes ***************/

void InitPorts(void);
void InitTimers(void);
void InitInterrupts(void);
void WatchDogTimer(void);
void SightPin_A2(void);
void SightPin_A3(void);
void MoistureCalc(void);
void SetLEDsForWatering(void);


/******* Acutal Functions ****************/

void InitPorts()
{

	ANSELA = 0x00;			// Port A pins are digital
	ANSELB = 0x00;			// Port B pins are digital

	TRISA = 0b00000000;		// 1 - input, 0 - output, all A pins are outputs 
	TRISB = 0b00001011;		// 1 - input, 0 - output, RB0 is an input, it's the input capture pin

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

/* First need to poll the interrupt flag bits to determine source of interrupt.
 * The two pulses we are looking for are separated by 100 to 200 miliseconds (ms),
 * so we need the ISR to finish its work in probably less than 100 ms. The program
 * will then return to the main function to wait out the rest of the delay and the
 * ISR should be called again before it goes into sleep mode. */
void interrupt ISR() // function needs to execute in <100ms
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

        if (CCP1IF) // flag is in register PIR1<2>
        {


            static unsigned int uLastEdge; 
            char highByte;
            char lowByte;
            int CCPR1_Snapshot;

            captureTracker ++; //keeps track of how many captures have been done

            //SightPin_A3(); // debugging

            highByte = CCPR1H; // CCPR1H captures value from TMR1H register
            lowByte = CCPR1L; // CCPR1L captures value from TMR1L register

            //this is the time when the pulse from the sensor arrived
            CCPR1_Snapshot = (highByte<<8) | lowByte; // combine into 16 bit int

            uPeriod = CCPR1_Snapshot - uLastEdge;
            uLastEdge = CCPR1_Snapshot; // this variable should not be in scope next ISR

            
            /*
            if ((captureTracker % 2) != 0)
		{
                        uPeriod = uPeriod;

            }
		else
		{
                        rawInterval = uPeriod;
			//captureTracker = 0;
		}
            */


            if (uPeriod == 0) { uPeriod = 23; }
            //twentyPer = uPeriod;

            //maybe write 0 to the TMR1H and TMR1L bytes to ensure against rollover?
            //the datasheet says to stop the timer before writing to below registers
            TMR1H = 0;
            TMR1L = 0;

            CCP1IF = 0; // clear the flag
        }

} //pops previous address from the stack, restores registers, and sets GIE bit


void MoistureCalc(void)
{
    
    /* The following moisture calculation section is commented out until we
     * can understand how this calculation yields the percentage of water in
     * the soil.

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
    */

    //set the value of the moisture, should be a number from 0 to 100
    moisture = 25; //this will be a formula based on the characterization curve

    //this gives the next index for the arrays
    int next_index = (captureTracker / 2) - 1; //can also use left shift ">>1" to divide

    //add the next moisture value and rate change to the arrays
    moistureValues[next_index] = moisture;
    moistureChangeRate[next_index] = moisture - previous_moisture;

    //save the current moisture reading to calculate the rate change
    previous_moisture = moisture;

}

void SetLEDsForWatering(void) // must account for rate of watering.
{
        if (moisture < target_value) {PORTB &= BIT5LO;} //RB5 port LED bright
        else {PORTB |= BIT5HI;} //RB5 port LED dim
  
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

            //turn sensor on
            //PORTB &= BIT4LO;
            PORTB |= BIT4HI;

            //PC stuck in loop until first capture (capTrack is an even number)
            while (captureTracker % 2 == 0){PORTA |= BIT3HI;}//RA3 dim // even and 0 % 2 = 0
     
            //PC stuck in another loop until second capture (capTrack is an odd number)
            while (captureTracker % 2 == 1){PORTA &= BIT3LO;}//RA4 dim //odd % 2 = 1

            //turn off sensor
            //PORTB |= BIT4HI; //turn the sensor off for the duration of the sleep cycle
            PORTB &= BIT4LO;

            //do calculation
            MoistureCalc();

            //turn on LEDs if need to water
            SetLEDsForWatering();

             //Put device to sleep and wait for the next time to take a measurment
            //the sleep time is set by the postscaler of the WDT
            for (y=0;y<4;y++) {SLEEP();} // will sleep x number of times

            //start the count over when the array's are full
            //capture tracker will be an even number at this point
            //if (captureTracker == (2*total_values)) {captureTracker = 0;}


            /* this code works to strobe an LED connected to RB4
            PORTB &= BIT4LO; //makes LED on RB4 bright -- currenly 2.2V (Sensor Power)
            for (y=0;y<2;y++) {SLEEP();} // will sleep x number of times
            PORTB |= BIT4HI; //makes LED on RB4 dim
            for (y=0;y<2;y++) {SLEEP();} // sleep x number of times
            */

	}



}


