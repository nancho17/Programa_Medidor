//******************************************************************************
//  MSP430F2618 - Software Medidor Alterna Proser
//
//  Description; ADE7953 measurement.
//  ACLK = 16MHza, MCLK = SMCLK = 1MHz DCO
//
//                MSP430x2xx
//             -----------------
//         /|\|              XIN|--
//          | |                 |     32768 Hz   
//          --|RST          XOUT|--
//            |                 |
//            |                 |
//            |     P3.4/UCA0TXD|------------>
//            |                 | 4800 - " 8N1 "     1bit c/ 208,33333us
//            |     P3.5/UCA0RXD|<------------
//
//  
//  Texas Instruments, Inc
//  January 2006
//  Built with IAR Embedded Workbench Version: 3.40A
//******************************************************************************
#include "msp430.h"
#include "intrinsics.h"
#include "stdint.h"
#include "stdbool.h"
#include "ADE7953.h"

/*---------Task_Time_Definitions---------*/
#define TAREA_ADE       100   // Valor en ms
#define TAREA_MODBUS    200   // Valor en ms
#define TAREA_DA        500   // Valor en ms

/*---------Functions_Declarations---------*/
void Set_DCO_using32kHz(void);
void Set_DCO_1MHzstored(void);
void UART0_P3_config (void);
//bool Escritura_ADE795 ( uint8_t); 
//void Lectura_ADE795 ( void);
//void ADE_Lectura_1ms_TIMING(uint8_t*);
//void ADE_Interruptor_RX(void);

/*---------NO_Optimized_Variables---------*/
static volatile uint32_t ms_ticks = 0;
static volatile uint32_t ms4_ticks = 0;

static volatile bool tick_0ms5_elapsed = false;
static volatile bool tick_1ms_elapsed = false;
static volatile bool tick_500ms_elapsed = false;
static volatile bool tick_200ms_elapsed = false;
static volatile bool tick_100ms_elapsed = false;

static volatile bool tick1_4ms_elapsed = false;
static volatile bool tick1_1000ms_elapsed = false;

/*---------Global_Variables---------*/

bool wellsended = false;

uint8_t DATA_ADE[3];

uint8_t auxiliar=0;



/* (1/32768)s=0,00003051757812s -> (1/f)* 33 s =1,007080078125 ms */
void Time_Handler_1(void)
{
    ms_ticks++;
    tick_0ms5_elapsed = true;


    if (ms_ticks > 499 ) {
        tick_500ms_elapsed = true; //500ms
        ms_ticks=0;
        }
   
               //0,48828125 ms 16
               //0,5187988  ms 17
               //0,9765625  ms 32 
               //1,0070800  ms 33
               //2,0141601  ms  66
               //4,0283203  ms 132
               //8,0566406  ms 264
               //16,113281  ms 528 
        CCR0+=33;
                
    
}


void Time_Handler_2(void)
{
    //tick_500ms_elapsed = true;
    tick1_1000ms_elapsed = true;
            //  1   s  32768
            //500  ms  16384
            //250  ms  8192               
        
    CCR1+=32.768;
}

/*---------Interrupt_Vectors---------*/
#pragma vector=TIMERA0_VECTOR
__interrupt void Timer_A_0(void){
  
  LPM1_EXIT;
  Time_Handler_1();

}

#pragma vector=TIMERA1_VECTOR
__interrupt void Timer_A_1 (void){
    
    LPM1_EXIT;
  
    if (TAIV==TA0IV_TACCR1){
        Time_Handler_2();
    }

    if (TAIV==TA0IV_TAIFG){
        //timeroverflow();
    }
}

#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void){
    LPM1_EXIT;
    if(auxiliar>2){auxiliar=0;}
    DATA_ADE[auxiliar]=ADE_Interruptor_RX(auxiliar);
    auxiliar++;
}





int main(void)
{   DATA_ADE[0]=0;
    DATA_ADE[1]=0;
    DATA_ADE[2]=0;
    WDTCTL = WDTPW+WDTHOLD;               // Stop watchdog timer
    for (int i = 0; i < 0xfffe; i++);             // Delay for XTAL stabilization
     
    Set_DCO_1MHzstored();
//    Set_DCO_using32kHz();
    
    P4DIR |= 0xFF;                        // Set P2.0 to output direction
    P4OUT = 0x00;
    
   // P5DIR |= 0x78;                            // P5.6,5,4,3 outputs
    //P5SEL |= 0x70;                            // P5.6,5,4 options
    
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
    //CCR0 =16000; //65535; 1 ms
    CCR0 =48; //65535; 0.250 ms
    
    CCR1 =16; //65535; 4 ms
    
    CCTL0=0;
    CCTL1=0;
    
    CCTL0 = CCIE;       // CCR0 interrupt enabled + compare mode
    CCTL1 = CCIE;       // CCR1 interrupt enabled + compare mode
    
    
    //TACTL = TASSEL_2 + MC_2;                 // SMCLK, contmode
    TACTL = TASSEL_1 + MC_2+TAIE;              // ACLK, contmode,TAIFG interrupt request
    //TACTL = TASSEL_2 + MC_1;                 // SMCLK, upmode
    //TACTL = TASSEL_1 + MC_1+ TAIE;           // ACLK, upmode, TAIFG interrupt request
   
    UART0_P3_config();
    
    uint8_t a=0x00; //Uart 3 data
    
    uint8_t ReadedADE[3];
    ReadedADE[0]=7;
    auxiliar=0;
    
     
    __bis_SR_register(GIE);       //interrupt
  /*Entra en el bucle de las tasks*/
    while (1) //milisecond controlled Loop
        {
          
        if ( tick_0ms5_elapsed ) {
          //P4OUT ^= 0xFF;
          //0x318 
          //V, (R) Default: 0x000000, Signed,Instantaneous voltage
           //ADE_Lectura_0ms5_TIMING(&a);
           ADE_Lectura_1ms_TIMING(&a);  
           
           //ReadedADE[0]=DATA_ADE[0];
           //ReadedADE[1]=DATA_ADE[1];
           
            
            tick_0ms5_elapsed  = false; // Reset the flag (signal 'handled')
        
        }
        
        if (tick1_4ms_elapsed) {
            //P4OUT ^= 0xFF;
            tick1_4ms_elapsed = false; // Reset the flag (signal 'handled')
        }
        
        if (tick_500ms_elapsed) {
            P4OUT^= 0xF0;
            tick_500ms_elapsed = false; // Reset the flag (signal 'handled')
        }

        if (tick1_1000ms_elapsed) {
            P4OUT^= 0x0F;
            
            tick1_1000ms_elapsed = false; // Reset the flag (signal 'handled')
        }

        //LPM1;
    };
    
    //__bis_SR_register(LPM0_bits); // Enter LPM0 w/ interrupt    
    //No Llega aqui
        
}



/*---------------------------Functions---------------------------*/
/*---------------------------------------------------------------*/

void UART0_P3_config (void){

    /* Comm ADE7953 4800baud oversampling */
  
    P3SEL = 0x30;                           // P3.4,5 = USCI_A0 TXD/RXD
    UCA0CTL1 |= UCSSEL0;//ACLK   //UCSSEL_2;                   // SMCLK
    UCA0CTL0=0;                         //LSB first, 8-bit data, Parity disabled        0x0060
    //UCA0CTL0=UCMSB;                      //MSB first, 8-bit data, Parity disabled
    //UCA0CTL0=UCPEN;                         //LSB first, 8-bit data, Parity enabled 
    
 
    //UCA0BR0 = 208;                       // 16 MHz 4800
    //UCA0BR1 = 0;                         // 16 MHz 4800

    UCA0BR0 = 6;                           //  32768 Hz 4800
    UCA0BR1 = 0;                           //  32768 Hz 4800
    
  //  UCA0MCTL=  UCBRF_5 +UCOS16;             // Modln UCBRFx=5 UCBRSx=0, over sampling
      UCA0MCTL=  UCBRS_7 ;                    // Modln UCBRSx=0
    
    UCA0CTL1 &= ~UCSWRST;                   // **Initialize USCI state machine**
    IE2 |= UCA0RXIE;                        // Enable USCI_A0 RX interrupt //UCA0TXIE para TX
   __bis_SR_register( GIE);

    //Max. TX bit error: -0.10999999999997123(-0.10999999999997123,0)
    //Max. RX bit error (sync.: -0.5 BRCLK): -0.10500000000010501(-0.10500000000010501,0)
    //Max. RX bit error (sync.: +/-0 BRCLK): -0.09000000000009001(-0.09000000000009001,0.009999999999992653)
    //Max. RX bit error (sync.: +0.5 BRCLK): -0.075000000000075(-0.075000000000075,0.024999999999994644)
    //Max. RX bit error (over: -0.5,0,+0.5): 0(-0.10500000000010501,0.024999999999994644)
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

    BCSCTL2 |= SELM_0 + DIVM_0 + SELS + DIVS_0 ;
                             //This statement chooses (SELM_0: 0000 1110) DCO as
                             //the source for MCLK 
                             //SMCLK <--  ACKL , SMCLK divider8
                             //The clock division is by 1 if DIVM_0 (x1) is choosen .
    
    
    //BCSCTL1 &= ~(0x03 << 4);  //ACK divider = 1
   // BCSCTL1 |= XTS;  //XTS High frecuency
    
    //BCSCTL3 = 0x00;
    //BCSCTL3 |= LFXT1S_2 + XCAP_0; /* Mode 2 for LFXT1 : 3- to 16-MHz crystal or resonator */
    
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

