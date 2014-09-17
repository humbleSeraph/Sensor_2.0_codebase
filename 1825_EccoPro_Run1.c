/************************

This is a module to program the PIC16LF1825 get data from
the ECCO Pro sensor and output a moisture reading.

First going to get input caputure working with inputs
from a signal generator. Josh has since gotten this to correctly receieve
data from Ecco Pro sensor

author: Osagie Igbeare - o.igbeare@gmail.com
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
#pragma config WDTE = ON        // Watchdog Timer Enable (WDT controlled by the SWDTEN bit in the WDTCON register)
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
#define hangTime 1000
#define total_values 100 //this is the total number of values that will be saved in the array
#define DREAM_TIME 4

/*************** module level variables ************/

static char counter; 
int x, y;
int i = 0;
int captureTracker = 0;
int buttonPush = 0;
int next_index;

//int total_values = 24; //set the total number of values to capture
int moistureValues[total_values]; //array for moisture values
int moistureChangeRate[total_values]; //array for change in moisture values

static unsigned int uPeriod; 

int moisture = 0; //initialized to zero, maybe this should be unsigned
int previous_moisture = 0; //initialized to zero
int target_value = 20; //I set this arbitrarily to test the code
 

/********* Function Prototypes ***************/

void InitPorts(void);
void InitTimers(void);
void InitInterrupts(void);
void InitComm(void);
void WatchDogTimer(void);
void IOC_Config(void);
void SightPin_C2(void);
void SightPin_C1(void);
void MoistureCalc(void);
void SetLEDsForWatering(void);


/******* Acutal Functions ****************/

void InitPorts()
{

	ANSELA = 0x00;			// Port A pins are digital
	ANSELC = 0x00;			// Port C pins are digital
  

	TRISA = 0b00111110;		// 1 - input, 0 - output; (?) - can be used for something else
                                         /********************************
                                          A0 - output - Tx pin
                                          (?)A1 - input - Rx pin
                                          A2 - input - data transfer button
                                          A3 - doesn't matter - MCLR pin
                                          A4 - doesn't matter - CLKOut pin
                                          A5 - doesn't matter - CLCKIn pin
                                          ********************************/

	TRISC = 0b00100001;		// 1 - input, 0 - output
                                        /**********************************
                                         C0 - input - 
                                         C1 - output - demo mode button
                                         C2 - output - IOC LED check
                                         (?) C3 - output- while loop indicator
                                         C4 - output - turns on Ecco
                                         C5 - input - input capture pin
                                         ********************************/



	PORTA = 0b00000100;             //Pins RA7 to RA0, 1 for VIH(>1.5V) and 0 for VIL(<0.5V)
	PORTC = 0b00000101;

	APFCON0 = 0b10000100;           // Enables RA0 to be Tx pin, RA1 to be Rx pin (for EUSART)
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
                                         bit 4(T1GSPM) = 0; single pulse mode disabled
                                         bit 3(T1GGO) = 0; clear when bit 4 is clear
                                         bit 2(T1GVAL) = 1; current state of TMR1 gate
                                         bit <1:0>(T1GSS) = 00; TMR1 gate pin
                                         *******************************************/

        CCP1CON = 0b00110100;            //capture mode: every falling edge
        //CCP1CON = 0b00000100;          //least 4 bits sets capture mode

        T2CON = 0b01111110;		// Fosc / (4 instruct * 16 prescale * 16 postscale * 60 PR2) = 65 Hz
                                         /********************************************
                                         bit 7 unimplemented
                                         bit <6:3>(TxOUTPS) = 1111; 1:16 postscaler
                                         bit 2(TMR2ON) = 1; Timer2 is on
                                         bit <1:0>(T1GSS) = 10; prescaler is 16
                                         *******************************************/

	PR2 = 250;
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

void InitComm()
{
    // configuration for EUSART

    BAUDCON = 0b00000000;   // bit 3 (BRG16) = 0; default setting, indcluded for clarity
    SPBRG = 25;             // SPBRG = (Fosc / (16*BaudRate))-1; Fosc = 4MHz
    SPBRGH = 0;
    TXSTA = 0b00100100;     //TxEn set, BRGH set, 8 bit trans, asynchronous
    RCSTA = 0b10010000;     //RxEn set, 8 bit receive, CREN set, no addr detection



}


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

	INTCON = 0b11001000;
                                         /********************************************
                                         bit 7(GIE) = 1; Global Interrupt Enable bit
                                         bit 6(PEIE) = 1; Peripheral Interrupt Enable bit
                                         bit 5(TMR0IE) = 0; Timer0 Overflow Interrupt Enable bit
                                         bit 4(INTE) = 0; INT External Interrupt Enable bit 
                                         bit 3(IOCIE) = 1; Interrupt-on-Change Enable bit
                                         bit 2(TMR0IF) = 0; Timer0 Overflow Interrupt FLag bit
                                         bit 1(INTF) = 0; INT External Interrupt Flag bit
                                         bit 0(IOCIF) = 0; Interrupt-on-Change Interrupt Flag bit
                                         *******************************************/
       // IOCIF = 0;                       // clear IOC Flag

}

void IOC_Config()
{
    IOCAP = 0x00;                       // IOC for rising edge disabled for port A
    IOCAN = 0b00000100;                 // IOC for falling edge enabled for A2
    IOCAF = 0x00;                       // clear IOC flags for port A 

}

void interrupt ISR() // function needs to execute in <100ms
{
	
       
	if (TMR2IF)

	{
                counter++;
                if ((counter % 2) != 0)
		{
			PORTC |= BIT1HI; // 0b00000100

		}
		else
		{
			PORTC &= BIT1LO; // 0b11111011
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


            highByte = CCPR1H; // CCPR1H captures value from TMR1H register
            lowByte = CCPR1L; // CCPR1L captures value from TMR1L register

            //this is the time when the pulse from the sensor arrived
            CCPR1_Snapshot = (highByte<<8) | lowByte; // combine into 16 bit int

            uPeriod = CCPR1_Snapshot - uLastEdge;
            uLastEdge = CCPR1_Snapshot; // this variable should not be in scope next ISR

            // above is a method to capture the values from the TMR1H and TMR1L register and combine
            // them into a 16 bit integer. This 16 bit integer gives the "time reading" when the falling
            // edge occurred.


            if (uPeriod == 0) { uPeriod = 23; }
            //twentyPer = uPeriod;

            //maybe write 0 to the TMR1H and TMR1L bytes to ensure against rollover?
            //the datasheet says to stop the timer before writing to below registers
            //TMR1H = 0;
            //TMR1L = 0;

            CCP1IF = 0; // clear the flag
        }

        if (IOCIF) // check to see if button (RA2) was pushed
        {
            
      
            if ((IOCAF & BIT2HI) == BIT2HI)
            {
            

                buttonPush = 1;

                INTCON &= BIT3LO;       // turn off IOCIE - interrupt on change


                IOCAF &= BIT2LO;        // clear IOC Flag for port A2
                IOCIF = 0;
                INTCON |= BIT3HI;       //Re-enable IOCIE interrupt 

            }

        }
        

} //pops previous address from the stack, restores registers, and sets GIE bit


void MoistureCalc(void)
{
    
    //set the value of the moisture, should be a number from 0 to 100
    //this will be a formula based on the characterization curve

    /********************************************************************************/
    /**********this is where to put new algorithm for for moisture calc**************/
    /********************************************************************************/
    moisture = (15*uPeriod) - 2399; 

    //this gives the next index for the arrays
    next_index = (captureTracker / 2) - 1; //can also use left shift ">>1" to divide

    //add the next moisture value and rate change to the arrays
    moistureValues[next_index] = moisture;
    moistureChangeRate[next_index] = moisture - previous_moisture;

    //save the current moisture reading to calculate the rate change
    previous_moisture = moisture;

}

void SetLEDsForWatering(void) // must account for rate of watering.
{
        if (moisture > target_value)
        {
            PORTA |= BIT2HI;        // turn on lights indicating "water"
        }
        else
        {
            PORTC |= BIT2HI;       // turn on lights indicating "stop watering"

        }
  
}


/***********************************************************/
/******************** Debugging Library ********************/

void SightPin_C1(void)
{
	// toggles pin A2 for debugging purposes
	if ((PORTC & BIT1HI) == BIT1HI)
	{
		PORTC &= BIT1LO;

	}
	else
	{
		PORTC |= BIT1HI;
	}

}

void SightPin_C2(void)
{
	// toggles pin A3 for debugging purposes
	if ((PORTC & BIT2HI) == BIT2HI)
	{
		PORTC &= BIT2LO;

	}
	else
	{
		PORTC |= BIT2HI;
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
        InitComm(); 
	InitInterrupts();
        IOC_Config(); 


        
	while(1)
	{
            //function to check flag, if true then send data via bluetooth to paired computer
            
            if (buttonPush == 1)
            {

                for (int j = 0; j<(next_index + 1); j++)
                {
                    while (!TXIF);
                    TXREG = moistureValues[j];
                }

                buttonPush = 0;
                captureTracker = 0;
            }
            

            //turn sensor on
            // pull low - turns off NPN BJT - which then turns on Ecco
          
            PORTC &= BIT4LO;

            //PC stuck in loop until first capture (capTrack is an even number)
            while (captureTracker % 2 == 0){PORTC &= BIT3LO;}//RA3 dim // even and 0 % 2 = 0
     
            //PC stuck in another loop until second capture (capTrack is an odd number)
            while (captureTracker % 2 == 1){PORTC |= BIT3HI;}//RA4 dim //odd % 2 = 1

            //turn off sensor
            //pull high - turns on NPN BJT - turns Ecco off
            PORTC |= BIT4HI;

            //do calculation
            MoistureCalc();

            //turn on LEDs if need to water
            SetLEDsForWatering();


            //Put device to sleep and wait for the next time to take a measurment
            //the sleep time is set by the postscaler of the WDT
            // DREAM_TIME is a #define and can be altered to alter how long mcu is asleep
            for (y=0;y<DREAM_TIME;y++) {SLEEP();} // will sleep x number of times
	}



}


