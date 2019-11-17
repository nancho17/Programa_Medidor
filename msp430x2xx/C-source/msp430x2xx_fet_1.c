//******************************************************************************
//  MSP430x2xx Demo - Software Toggle P1.0
//
//  Description; Toggle P1.0 by xor'ing P1.0 inside of a software loop.
//  ACLK = n/a, MCLK = SMCLK = default DCO
//
//                MSP430x2xx
//             -----------------
//         /|\|              XIN|--
//          | |                 | 16MHz
//          --|RST          XOUT|--
//            |                 |
//            |             P1.0|-->LED
//
//  A. Dannenberg
//  Texas Instruments, Inc
//  January 2006
//  Built with IAR Embedded Workbench Version: 3.40A
//******************************************************************************
#include "msp430.h"
#include "intrinsics.h"
#include "driverlib.h"
#include "stdint.h"
#include "stdbool.h"


void Set_DCO_using32kHz(void);
void Set_DCO_1MHzstored(void);
volatile unsigned int i;

static volatile uint32_t ms_ticks = 0;
static volatile bool tick_1ms_elapsed = false;
static volatile bool tick_1000ms_elapsed = false;
static volatile bool tick_500ms_elapsed = false;
static volatile bool tick_200ms_elapsed = false;
static volatile bool tick_100ms_elapsed = false;


void Roca_1(void)
{
    P4OUT ^= 0xFF;                           // Toggle P4.0
    //CCR0 += 400;                            // Add Offset to CCR0

}

/* (1/32768)s=0,00003051757812s -> (1/f)* 33 s =1,007080078125 ms */
void Time_Handler(void)
{
    ms_ticks++;
    tick_1ms_elapsed = true;  //1ms

    if (ms_ticks % 500 == 0) {
        tick_500ms_elapsed = true; //500ms
        }

    if (ms_ticks % 1000 == 0) {
        tick_1000ms_elapsed = true; //1000ms
        ms_ticks=0;
    }
}

//Interrupción TIMERA
#pragma vector=TIMERA0_VECTOR
__interrupt void Timer_A (void)
{
  Time_Handler();

}


int main(void)
{   
    WDTCTL = WDTPW+WDTHOLD;               // Stop watchdog timer
    for (i = 0; i < 0xfffe; i++);             // Delay for XTAL stabilization
  
    Set_DCO_1MHzstored();
//    Set_DCO_using32kHz();
    
    P4DIR |= 0xFF;                        // Set P2.0 to output direction
    P4OUT = 0x00;
    
    P5DIR |= 0x78;                            // P5.6,5,4,3 outputs
    P5SEL |= 0x70;                            // P5.6,5,4 options
    
    for (int a=0;a<2;a=a+1)
    {
        P4OUT ^= 0xFF;                      // Toggle P1.0 using exclusive-OR
        __delay_cycles(50000);
        
    }
        for (int a=0;a<2;a=a+1)
    {
        P4OUT ^= 0xFF;                      // Toggle P1.0 using exclusive-OR
        __delay_cycles(500000);
        
    }
    
    //P1DIR |= 0x01;                            // P1.0 output
    CCTL0 = CCIE;                             // CCR0 interrupt enabled
    CCR0 =16000; //65535;
    //TACTL = TASSEL_2 + MC_2;                  // SMCLK, contmode
    //TACTL = TASSEL_1 + MC_2;                  // ACLK, contmode
    //TACTL = TASSEL_2 + MC_1;                  // SMCLK, upmode
    TACTL = TASSEL_1 + MC_1;                  // ACLK, upmode
   

/*Entra en el bucle de las tasks*/
    //__bis_SR_register(LPM0_bits + GIE);       // Enter LPM0 w/ interrupt    
   
    __bis_SR_register(GIE);       //interrupt    
    while (1) 
        {
          
        if (tick_1ms_elapsed) {
            //P4OUT ^= 0xFF;
            tick_1ms_elapsed = false; // Reset the flag (signal 'handled')
        }
        if (tick_500ms_elapsed) {
            P4OUT ^= 0xFF; 
            tick_500ms_elapsed = false; // Reset the flag (signal 'handled')
        }

        if (tick_1000ms_elapsed) {
            //P4OUT ^= 0xFF; 
            tick_1000ms_elapsed = false; // Reset the flag (signal 'handled')
        }

        
        };
     //__delay_cycles(9000000);
    //TACTL=MC_0;
    // __bis_SR_register(LPM0_bits);
        
}

void Set_DCO_1MHzstored(void){
    if (CALBC1_1MHZ ==0xFF || CALDCO_1MHZ == 0xFF)                                     
    {  
    while(1);                                        // If cal constants erased, trap CPU!!
    }
    /*
    Del datasheet:
    ;Set DCO to 1 MHz:
    CLR.B &DCOCTL ; Select lowest DCOx and MODx settings
    MOV.B &CALBC1_1MHZ,&BCSCTL1 ; Set range
    MOV.B &CALDCO_1MHZ,&DCOCTL ; Set DCO step + modulation
    */
   
    DCOCTL=0x00;/* DCO Select Bit 0 *//* Modulation Bit 0 */
    
    //Siguiendo el datasheet del : DCO Frequency tabla de la pagina 42 del datasheet 
    //    RSELx = 6, DCOx = 3, MODx = 0 -> 0.54 MHz - 1.06 MHz 
    BCSCTL1=0x46; //      0x46 =  0 1 00 0110;
    DCOCTL=0x60;   //      0000 = 011 00000; 0110 0000  
    
    
    
    //Datos de calibracion
    BCSCTL1=CALBC1_1MHZ;
    DCOCTL=CALDCO_1MHZ;

    BCSCTL2 |= SELM_0 + DIVM_0;   //This statement chooses (SELM_0: 0000 0000) DCO as
                                  //the source for MCLK and SMCLK with internal
                                  //resistor. The clock division is by 8, as choosen DIVM_3 (x3).
    

    BCSCTL1 &= ~(0x03 << 4);  //ACK divider = 1
    BCSCTL1 |= XTS;  //XTS High frecuency
    BCSCTL3 = 0x00;
    BCSCTL3 |= LFXT1S_2 + XCAP_0; /* Mode 2 for LFXT1 : 3- to 16-MHz crystal or resonator */
    
}

void Set_DCO_using32kHz(void)                          // Set DCO to selected frequency
{                                           
    //unsigned int DELTA= 245;                    // target DCO = DELTA*(4096) = 1003520 Hz
    unsigned int DELTA= 256;                    // target DCO = DELTA*(4096) = 1048576 Hz
    unsigned int Compare, Oldcapture = 0;
    BCSCTL1=0x00;
    BCSCTL1 |= DIVA_3;                        // ACLK= LFXT1CLK/8 = 4096Hz
    TACCTL2 = CM_1 + CCIS_1 + CAP;            // CM2 on rising, CCl2B:  ACLK, CAP
    TACTL = TASSEL_2 + MC_2 + TACLR;          // SMCLK, cont-mode, clear
  
    while (1)
        {
        while (!(CCIFG & TACCTL2));             // Wait until capture occured
        TACCTL2 &= ~CCIFG;                      // Capture occured, clear flag
        Compare = TACCR2;                       // Get current captured SMCLK
        Compare = Compare - Oldcapture;         // SMCLK difference
        Oldcapture = TACCR2;                    // Save current captured SMCLK
    
        if (DELTA == Compare)
            break;  
        else if (DELTA < Compare)
            {
            DCOCTL--;                             // DCO is too fast, slow it down
            if (DCOCTL == 0xFF)                   // Did DCO roll under?
                if (BCSCTL1 & 0x0f)
                    BCSCTL1--;                        // Select lower RSEL
            }
        else
            {
            DCOCTL++;                             // DCO is too slow, speed it up
            if (DCOCTL == 0x00)                   // Did DCO roll over?
            if ((BCSCTL1 & 0x0f) != 0x0f)
            BCSCTL1++;
            }
        }
    TACCTL2 = 0;                              // Stop TACCR2
    TACTL = 0;                                // Stop Timer_A
    BCSCTL1 &= ~DIVA_3;                       // ACLK = LFXT1CLK = 32.768KHz
}

