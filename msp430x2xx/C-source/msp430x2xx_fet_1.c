//******************************************************************************
//  MSP430x2xx Demo - Software Toggle P1.0
//
//  Description; Toggle P1.0 by xor'ing P1.0 inside of a software loop.
//  ACLK = n/a, MCLK = SMCLK = default DCO
//
//                MSP430x2xx
//             -----------------
//         /|\|              XIN|-
//          | |                 |
//          --|RST          XOUT|-
//            |                 |
//            |             P1.0|-->LED
//
//  A. Dannenberg
//  Texas Instruments, Inc
//  January 2006
//  Built with IAR Embedded Workbench Version: 3.40A
//******************************************************************************

#include "msp430.h"

int main(void)
{
  
  if (CALBC1_1MHZ ==0xFF || CALDCO_1MHZ == 0xFF)                                     
  {  
  while(1);                                                                                // If cal constants erased, trap CPU!!
  }
  
/* Del datasheet:
;Set DCO to 1 MHz:
CLR.B &DCOCTL ; Select lowest DCOx and MODx settings
MOV.B &CALBC1_1MHZ,&BCSCTL1 ; Set range
MOV.B &CALDCO_1MHZ,&DCOCTL ; Set DCO step + modulation
*/
//En c, en este programa  
//DCOCTL=DCO0+MOD0;/* DCO Select Bit 0 *//* Modulation Bit 0 */

  DCOCTL=0x00;/* DCO Select Bit 0 *//* Modulation Bit 0 */
    
  //Datos de calibracion
  BCSCTL1=CALBC1_1MHZ;
  DCOCTL=CALDCO_1MHZ;

  //Otra opcion: siguiendo el datasheet del : DCO Frequency tabla de la pagina 42 del datasheet 
  //    RSELx = 6, DCOx = 3, MODx = 0 -> 0.54 MHz - 1.06 MHz 
  BCSCTL1=0x46; //      0x46 =  0 1 00 0110;
  DCOCTL=0x60;   //      0000 = 011 00000; 0110 0000  
  
  //Por si hace falta el DCO a 8MHz
  //BCSCTL1=CALBC1_8MHZ;
  //DCOCTL=CALDCO_8MHZ;
  
  BCSCTL2 |= SELM_0 + DIVM_1;//This statement chooses (SELM_0: 0000 0000) DCO as
                           //the source for MCLK and SMCLK with internal
  //resistor. The clock division is by 8, as choosen DIVM_3 (x3).  

  WDTCTL = WDTPW+WDTHOLD;             // Stop watchdog timer
 
  // P1DIR |= 0x01;                        // Set P1.0 to output direction
  P4DIR |= 0xFF;                        // Set P2.0 to output direction

  volatile unsigned int i; // volatile to prevent optimization
  for (int a=0;a<25;a=a+1)
  {
    P4OUT ^= 0x01;                      // Toggle P1.0 using exclusive-OR
    P4OUT ^= 0x02;                      // Toggle P1.0 using exclusive-OR
    P4OUT ^= 0x04;                      // Toggle P1.0 using exclusive-OR
    P4OUT ^= 0x08;                      // Toggle P1.0 using exclusive-OR
    P4OUT ^= 0x20;                      // Toggle P1.0 using exclusive-OR
    P4OUT ^= 0x40;                      // Toggle P1.0 using exclusive-OR
    P4OUT ^= 0x80;                      // Toggle P1.0 using exclusive-OR
    
    i = 32000;                          // SW Delay
    do i--;
    while (i > 0);
  }

    P4OUT ^= 0x10;                      // Toggle P1.0 using exclusive-OR
  
}
