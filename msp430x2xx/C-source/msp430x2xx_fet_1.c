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
#include "modbus_med.h"

/*---------Custom_Pin_Functions----------*/

//P1
#define MX_SHUTDOWN		(0x40)  //active low
#define MX_ENABLE 		(0x80)  //active low

//P2
#define ISL_R_ENABLE	(0x40)  //active low
#define ISL_D_ENABLE	(0x80)

//P3
#define ON_PCB_RELE 	(0x02)
#define ADE_RESET 		(0x04)
#define ADE_IRQ 		(0x08)  // no conectado
#define ADE_RX 			(0x10)
#define ADE_TX 			(0x20)
#define SERIAL_DIN 		(0x40)
#define SERIAL_ROUT 	(0x80)

//P6
#define LED_V1		 	(0x01)
#define LED_V2  		(0x02)
#define SALIDA_DAC 		(0x40)

/*---------Boot Options---------*/
#define SOBRE_TENSION           100   // Valor en ms
#define SOBRE_CORRIENTE         200   // Valor en ms
#define SUB_TENSION           500   // Valor en ms


/*---------Task_Time_Definitions---------*/
#define TAREA_ADE       100   // Valor en ms
#define TAREA_MODBUS    200   // Valor en ms
#define TAREA_DA        500   // Valor en ms


/*-- Type Definitions --*/
	typedef union {
	float efe;
	uint16_t A[2];
	} SendFloat;


/*---------Functions_Declarations---------*/
void Set_DCO_using32kHz(void);
void Set_DCO_1MHzstored(void);
void UART0_P3_config (void);
bool RS232_send ( uint8_t);
void Send_Text ( char*);
int numero_input( void);
void menu_serie(bool*,bool*, uint8_t*, uint8_t* );


//bool Escritura_ADE795 ( uint8_t); 
//void Lectura_ADE795 ( void);
//void ADE_Lectura_1ms_TIMING(uint8_t*);
//void ADE_Interruptor_RX(void);

/*---------NO_Optimized_Variables---------*/
static volatile uint32_t ms_ticks = 0;
static volatile uint32_t ms2_ticks = 0;

static volatile bool tick_1ms_elapsed = false;

static volatile bool tick_500ms_elapsed = false;
static volatile bool tick_200ms_elapsed = false;
static volatile bool tick_100ms_elapsed = false;

static volatile bool tick_10s_elapsed = true;


/*---------Global_Variables---------*/

static uint8_t DATA_ADE[3]; // Buffer RX0

double E_Acumulator=0;		// Variable para almacenar datos de energía medida (En Joules)

uint8_t TablaDatos[47];  	// Tabla con datos de los registros del ADE7953ade

uint8_t auxiliar=0;			// Manejo de RX0

uint8_t auxiliar2=0;		// Manejo de RX1

uint8_t aux_menu=0;			// Manejo menu RX1

char AuxBuffer[16];  	// Buffer consola serial

/*---------Definicion de estructura---------*/
struct boot_menu {
	uint32_t Resistores_VP_N;
	uint32_t Shunt_IAP_N;
	
	uint16_t Offset_VP_N;
	uint16_t Offset_IAP_N;
	
	uint8_t PGA_IA;
	uint8_t PGA_V;
	
	uint8_t Cloop; // 1 Vrms 2 Irms 3 Voltaje Pico
	
	uint8_t Rele_Selec; //1 Sobre Tension 2 Sobre corriente 3 Sub Tension 4 Sub Potencia
	uint32_t Rele_Value;
		  
} ;

/*Funcion que maneja la interrucpion del tmer 0*/

void Ten_Second_Waiter(void)
{
    ms_ticks++;
   if (ms_ticks > 9 ) {
        tick_10s_elapsed = false; 
        ms_ticks=0;
        }
   
        CCR0+=32768; // El timer interrumpe cada 1 s
}

void Time_Handler_1(void)
{
    ms_ticks++;
    tick_1ms_elapsed = true;

    if (ms_ticks > 499 ) {
        tick_500ms_elapsed = true; //500ms
        ms_ticks=0;
        }
   
        CCR0+=36; // El timer interrumpe cada 1,0986328 ms
}

/*Funcion que maneja la interrucpion del tmer 1*/
void Time_Handler_2(void)
{
  	ms2_ticks++;
	tick_100ms_elapsed = true;
  
	if (ms2_ticks > 1 ) {
        tick_200ms_elapsed = true; //200 ms
        ms2_ticks=0;
    }
        
    CCR1+=3276; // El timer interrumpe cada 100 ms
}

/*---------Vectores de interrupcion---------*/

/*Interruptor 0 Timer A*/
#pragma vector=TIMERA0_VECTOR
__interrupt void Timer_A_0(void){
  
  LPM1_EXIT;
  if(tick_10s_elapsed){  Ten_Second_Waiter(); }
  else{  Time_Handler_1(); }

}

/*Interruptor 1 Timer A*/
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

// Manejo interrupción RX0
#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void)
{
    LPM1_EXIT;
    if(auxiliar>2){auxiliar=0;}
    DATA_ADE[auxiliar]=UCA0RXBUF;
    auxiliar++;
}

// Manejo interrupción RX1
#pragma vector=USCIAB1RX_VECTOR
__interrupt void USCI1RX_ISR(void)

{
  LPM1_EXIT;
  if(1){
		while (!(UC1IFG&UCA1TXIFG));               // USCI_A1 TX buffer ready?
		aux_menu=UCA1RXBUF;
		if(auxiliar2>15){auxiliar=0;}
		AuxBuffer[auxiliar2]=aux_menu;
		UCA1TXBUF = aux_menu;                     // TX -> RXed character
		auxiliar2++;
  } else{
		callInterruptRX(  );  
  };

}



/*Main Function*/
int main(void)
{
  	WDTCTL = WDTPW+WDTHOLD;               // Stop watchdog timer
	struct	boot_menu medidor = {
  		911, //ohm                      Resistores_VP_N;
		400, //miliohm                  Shunt_IAP_N
	
		0,// Vp-n sin offset            Offset_VP_N
		0,//IAp-n sin offset           Offset_IAP_N
		
		2,// PGA_IA Ganancia 2 
		1,// PGA_V  Ganancia 1
		
		1,// proporcional a Vrms
		
		1,  // Rele acciona por sobretension
		150, //Valor :150V
	};

	for (int i = 0; i < 0xfffe; i++);     // Delay for XTAL stabilization
     
    Set_DCO_1MHzstored();
	
	
	/*Configuracion de Pines*/
	P4DIR |= 0xFF;                       // Set P2.0 to output direction
    P4OUT = 0x00;
	P3SEL &=0x00;  
	P3DIR |= 0x0F;
	P3OUT= ADE_RESET;
	UART0_P3_config();
	P2DIR |= 0xC0;
	P2OUT = 0x40;
	P1DIR |= 0xC0;
	P1OUT = 0x40;

	/*Delay*/
    for (int r=0;r<2;r=r+1)
    {
        P4OUT ^= 0xFF;                      // Toggle P1.0 using exclusive-OR
        __delay_cycles(50000);
        
    }
    
	/*Seteo e inicio de los timers*/
	CCR0 =48; //65535; 0.250 ms
    CCR1 =15; //65535; 4 ms
    
    CCTL0=0;
    CCTL1=0;
    
    CCTL0 = CCIE;       // CCR0 interrupt enabled + compare mode
    
    
    TACTL = TASSEL_1 + MC_2+TAIE;              // ACLK, contmode,TAIFG interrupt request
    
    
    uint8_t TablaTemporal[47];
	
	
	auxiliar=0;		//contador para la recepcion en UART0
    int be=55;     	//contador para la tabla de datos
	

    __bis_SR_register(GIE);       //interrupt enable

	float LazoCorriente =0x00; //Variable a enviar al lazo de corriente
	
	/*-----Modbus Test -------*/
	/*RTU, Esclavo 5, NULL,9600 baud , Paridad: PAR*/
	uint16_t * regs; 
	SetRegStart (7002); // Inicio del registro

	

	SendFloat pack;
	float namas;
	namas=45.55;
	
	
	regs=GetRegBuff();

	pack.efe=namas;
	*(regs)	 =pack.A[1];
	*(regs+1)=pack.A[0];

	pack.efe=88.66;
	*(regs+2)=pack.A[1];
	*(regs+3)=pack.A[0];

	pack.efe=78952;
	*(regs+4)=pack.A[1];
	*(regs+5)=pack.A[0];

	pack.efe=0.0466;
	*(regs+6)=pack.A[1];
	*(regs+7)=pack.A[0];
	
	/*MB_RTU , MB_ASCII */
	  
	//eMBInit(MB_ASCII, 0X05, 0, 9600, MB_PAR_EVEN);
	//eMBEnable();
	
	
	while(0){
	eMBPoll();
	
	
	}
	
	
 //Iniciar timer	
/////////////*			Serial en 10 segundos previos			*/////////////
        
        bool flag_0= false;			//Bandera del menu serie
	bool flag_1= true;			//Manejo de submenus serie

	uint8_t menu_switch=0;	//Posiciones del menu serie
     	
        
	while (tick_10s_elapsed) {// con un timer tick_10s_elapsed=0;
		for (int r=0;r<14;r=r+1){
			if(AuxBuffer[r]=='c' || AuxBuffer[r]=='C'){
				if(AuxBuffer[r+1]=='f' || AuxBuffer[r+1]=='F'){
					if(AuxBuffer[r+2]=='g' || AuxBuffer[r+2]=='G'){flag_0=true; tick_10s_elapsed=false;}
				}
			}
		}
	  
	}

	uint8_t Tolstoi=0;
	int Orwell[16];
	

	/*Menu*/
        while (flag_0) {// con un flag
            menu_serie(&flag_0,&flag_1,&menu_switch,&Tolstoi);
            }
            

/////////////*			Entra en el bucle de las tareas			*/////////////
	//activo 2do timer
	CCTL1 = CCIE;       // CCR1 interrupt enabled + compare mode
	uint8_t a =0x00; //Contador usado en el loop controlado por tiempo
 	    
	while (1) //milisecond controlled Loop
        {
		  
/* 		Tarea ADE 		*/		  
        if ( tick_1ms_elapsed ) {
       	
		  switch(auxiliar){
			  case 1: 
				TablaTemporal[be]=DATA_ADE[0]; 
			  	
				break;
			  case 2: 
				TablaTemporal[be]=DATA_ADE[0]; 
			  	TablaTemporal[be+1]=DATA_ADE[1];
				
			  	break;
			  case 3:  
				TablaTemporal[be]=DATA_ADE[0]; 
			  	TablaTemporal[be+1]=DATA_ADE[1]; 
			  	TablaTemporal[be+2]=DATA_ADE[2]; 
				break;
			}

	
			
          switch(be){
          //Escribo en registros
		  case 55:	if (Escritor_Dir_8  (PGA_IA_8, 0x01, &a )) {  auxiliar=0; be=0;}; break; 
			
          //Leo los registros
		  case  0:  if (Lector_Dir_8  (LCYCMODE_8    , &a )) {  auxiliar=0; be=1;};  break;
          case  1:  if (Lector_Dir_8  (LAST_OP_8   , &a )) { auxiliar =0;   be=2;};  break;
		  case  2:  if (Lector_Dir_8  (PGA_IA_8    , &a )) { auxiliar =0; be=3;};  break;
		  case  3:  if (Lector_Dir_24 (AP_NOLOAD_24 , &a )) { auxiliar =0; be=6;};  break;
		  case  6:  if (Lector_Dir_16 (CONFIG_16    , &a )) { auxiliar =0; be=8;};  break;
		  case  8:  if (Lector_Dir_16 (CFMODE_16    , &a )) { auxiliar =0; be=10;};  break;
		  
		  
		  case 10:  if (Lector_Dir_24 (AVA_24    	, &a )) { auxiliar =0; be=13;};  break;
		  case 13:  if (Lector_Dir_24 (AWATT_24  	, &a )) { auxiliar =0; be=16;};  break;
		  case 16:  if (Lector_Dir_24 (AVAR_24  	, &a )) { auxiliar =0; be=19;};  break;
		  case 19:  if (Lector_Dir_24 (IA_24     	, &a )) { auxiliar =0; be=22;};  break;
		  case 22:  if (Lector_Dir_24 (V_24      	, &a )) { auxiliar =0; be=25;};  break;
		  case 25:  if (Lector_Dir_24 (IRMSA_24     , &a )) { auxiliar =0; be=28;};  break;
		  case 28:  if (Lector_Dir_24 (VRMS_24      , &a )) { auxiliar =0; be=31;};  break;
		  case 31:  if (Lector_Dir_24 (AENERGYA_24  , &a )) { auxiliar =0; be=34;};  break;
		  case 34:  if (Lector_Dir_24 (RENERGYA_24  , &a )) { auxiliar =0; be=37;};  break;
		  case 37:  if (Lector_Dir_24 (APENERGYA_24 , &a )) { auxiliar =0; be=40;};  break;
		  case 40:  if (Lector_Dir_16 (PFA_16       , &a )) { auxiliar =0; be=42;};  break;
		  case 42:  if (Lector_Dir_16 (Period_16    , &a )) { auxiliar =0; be=44;};  break;
		  
		  case 44:  if (Lector_Dir_24 (VPEAK_24     , &a )) { auxiliar =0; be=47;};  break;		  

		  case 50:  
		  for (int r=0;r<47;r=r+1)
		  	{TablaDatos[r]=TablaTemporal[r];}
		  be=0;
		  break;
		  
          default: be=be+1;   break;  
          }
            
            tick_1ms_elapsed  = false; // Reset the flag (signal 'handled')
		  
        
        }
        
    
/*		Tarea Serial			*/		
        if (tick_200ms_elapsed) {
		  
           static int32_t Voltaje_Medido, I_Medido, P_Medido, Q_Medido, I_rms,E_Activa,E_Reactiva;
           static uint32_t Voltaje_rms , Voltaje_pico, kiloWh;
		   static float Voltaje_Ins_F,Voltaje_rms_F, Voltaje_p_F, Corriente_F, I_rms_F, Pow_F, Q_F;
		   static float E_Activa_F,E_Reactiva_F;
		   
		   static float PF_F;
		   static int16_t PowerFactor;
		   //<< 8 * 1 0000 0000 *256
		   
		 	Voltaje_Medido = (((uint32_t) TablaDatos[24] << 24) + ((uint32_t) TablaDatos[23] << 16) + ((uint32_t) TablaDatos[22] << 8));
      		Voltaje_rms    = (((uint32_t) TablaDatos[30] << 24) + ((uint32_t) TablaDatos[29] << 16) + ((uint32_t) TablaDatos[28] << 8));
			Voltaje_pico   = (((uint32_t) TablaDatos[46] << 24) + ((uint32_t) TablaDatos[45] << 16) + ((uint32_t) TablaDatos[44] << 8));
			I_Medido   	   = (((uint32_t) TablaDatos[21] << 24) + ((uint32_t) TablaDatos[20] << 16) + ((uint32_t) TablaDatos[19] << 8));
			I_rms   	   = (((uint32_t) TablaDatos[27] << 24) + ((uint32_t) TablaDatos[26] << 16) + ((uint32_t) TablaDatos[25] << 8));
			P_Medido   	   = (((uint32_t) TablaDatos[15] << 24) + ((uint32_t) TablaDatos[14] << 16) + ((uint32_t) TablaDatos[13] << 8));
			Q_Medido   	   = (((uint32_t) TablaDatos[18] << 24) + ((uint32_t) TablaDatos[17] << 16) + ((uint32_t) TablaDatos[16] << 8));
			E_Activa  	   = (((uint32_t) TablaDatos[33] << 24) + ((uint32_t) TablaDatos[32] << 16) + ((uint32_t) TablaDatos[31] << 8));
			E_Reactiva     = (((uint32_t) TablaDatos[36] << 24) + ((uint32_t) TablaDatos[35] << 16) + ((uint32_t) TablaDatos[34] << 8));
						
			PowerFactor    = (((uint16_t) TablaDatos[41] << 8)  + TablaDatos[40] );
			
			Voltaje_Ins_F=(Voltaje_Medido*495.5)/1664000000;
			//  495.5V ->	6,500,000 *256 32bit			  
			Voltaje_rms_F=(Voltaje_rms*350.3714100779)/2312193792;    // 353,5 mv rms ->(9032007 24 bit) en chip (9032007*256 =2312193792 32bit)
						
			Voltaje_p_F=(Voltaje_pico*495.5)/1664000000; //no encuentro datos en datasheet
			// 24bit unsigned... 
			
			Corriente_F=((float) I_Medido)*2.5/1664000000;  //0.1 ohm 2.5 ampere 0.250mv 
			
			I_rms_F=(I_rms*1.7677669529664)/2312193792;    
			
			Pow_F=(P_Medido*619.375)/1244774656;    // 619.375 W ? 4862401 LSBs (decimal)
			
			Q_F=(Q_Medido*619.375)/1244774656;    // 619.375 VAr ? 4862401 LSBs (decimal)
			
			E_Activa_F=E_Activa*(25084.6875/2147483392);// Watt segundo (Joule)
			
			E_Reactiva_F=E_Reactiva*(25084.6875/2147483392);// Watt segundo 
			
			E_Acumulator=E_Acumulator+E_Activa_F;
			kiloWh=E_Acumulator/60000;
			
			PF_F=((float)PowerFactor)/32767; //32767;
			
			//divisor resistivo 991
			
			Voltaje_Medido=Voltaje_Ins_F;
			Voltaje_rms=Voltaje_rms_F;
	  		Voltaje_pico=Voltaje_p_F;
			I_Medido=Corriente_F*1000;
			I_rms=I_rms_F*1000;
			P_Medido=Pow_F;
			Q_Medido=Q_F;
			
			/*Envio al lazo de corriente*/
			LazoCorriente=2.5*(Voltaje_rms_F/350.371);
			
			/* Mayor que +250 mV  */
            if(I_rms>(120)){
                P6OUT|= 0x01;
			    P3OUT|= 0x02;
			}
            else {
				P3OUT&= ~0x02;
                P6OUT&= ~0x01;}
		
            tick_200ms_elapsed = false; // Reset the flag (signal 'handled')
        }

		//eMBPoll();// Poner en bucle de 1 milisegundo (?)
		static uint16_t Periodo,enhz;
		

    		
/*		Tarea DA		*/
        if (tick_500ms_elapsed) {
		  
		  /* Salida Del lazo de corriente*/
			ADC12CTL0 = REF2_5V + REFON;
			DAC12_0CTL = DAC12IR+ DAC12AMP_5 + DAC12ENC;  // Int ref gain 1
  			DAC12_0DAT = (uint16_t)(LazoCorriente*4095/2.5) ;                  						// 1.0V (2.5V = 0x0FFFh)
					
            Periodo = ( ((uint16_t) TablaDatos[43] << 8) + (uint16_t) TablaDatos[42] );
			enhz=223750/(Periodo+1);
	    	
			P6OUT^= 0x01;
          	P6OUT^= 0x02;
		
          
          tick_500ms_elapsed = false; // Reset the flag (signal 'handled')
        }

        //LPM1;
    };
    
    //__bis_SR_register(LPM0_bits); // Enter LPM0 w/ interrupt    
    //No Llega aqui
        
}



/*---------------------------Functions---------------------------*/
/*---------------------------------------------------------------*/
bool RS232_send ( uint8_t alfa){    
    /*UCAxTXIFG is automatically reset if a character is written to UCAxTXBUF.*/
    if (UC1IFG&UCA1TXIFG){
 // while(!UC1IFG&UCA1TXIFG){};
  	 	 UCA1TXBUF =alfa;	
        return 1;
    }
    return 0;
}

void Send_Text ( char* beta){    
    
  	while((*beta)>0){
	    if(RS232_send((*beta)))
		{beta++;}
	};
	
}

//COnfiguracion de Pines del puerto P3 para UART
void UART0_P3_config (void){

    /* Comm ADE7953 4800baud oversampling */ 
   
	
	P3SEL |= 0x30;                           // P3.4,5 = USCI_A0 TXD/RXD
    UCA0CTL1 |= UCSSEL0;//ACLK   //UCSSEL_2;                   // SMCLK 0100 0000
    UCA0CTL0=0;                         //LSB first, 8-bit data, Parity disabled        0x0060

    UCA0BR0 = 6;                           //  32768 Hz 4800
    UCA0BR1 = 0;                           //  32768 Hz 4800
    
    UCA0MCTL=  UCBRS_7 ;                    // Modln UCBRSx=0
	  
    UCA0CTL1 &= ~UCSWRST;                   // **Initialize USCI state machine**
    IE2 |= UCA0RXIE;                        // Enable USCI_A0 RX interrupt //UCA0TXIE para TX
   	UC0IE |= UCA0RXIE;
	//Max. TX bit error: -0.10999999999997123(-0.10999999999997123,0)
    //Max. RX bit error (sync.: -0.5 BRCLK): -0.10500000000010501(-0.10500000000010501,0)
    //Max. RX bit error (sync.: +/-0 BRCLK): -0.09000000000009001(-0.09000000000009001,0.009999999999992653)
    //Max. RX bit error (sync.: +0.5 BRCLK): -0.075000000000075(-0.075000000000075,0.024999999999994644)
    //Max. RX bit error (over: -0.5,0,+0.5): 0(-0.10500000000010501,0.024999999999994644)

	P3SEL |= 0xC0;                          // P3.6,7 = USCI_A1 TXD/RXD
 	UCA1CTL1 |= UCSSEL0;                    // CLK = ACLK
	UCA1CTL0=0;                         	//LSB first, 8-bit data, Parity disabled
  	UCA1BR0 = 0x03;                         // 
  	UCA1BR1 = 0x00;                         //
  	UCA1MCTL = UCBRS_3;                     // Modulation UCBRSx = 6 UCBRFx=13 Oversampling  
  	UCA1CTL1 &= ~UCSWRST;                   // **Initialize USCI state machine**
  	UC1IE |= UCA1RXIE;                      // Enable USCI_A1 RX interrupt

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


int numero_input( void) {

  		int ese=0, scale=1;
		int gamma=0;
		for (int r=0;r<16;r=r+1){
		  if(AuxBuffer[r]=='*' || AuxBuffer[r]==8){
		  ese=r; r=17;
		  }

		}
		if(ese>0){

			for (int r=ese-1;r>-1;r=r-1){
				if(AuxBuffer[r]>47 && AuxBuffer[r]<58){
					gamma=gamma+(scale*(AuxBuffer[r]-48));
					scale=scale*10;
				}
				for(int r=ese;r<16;r++){AuxBuffer[r]=0;};
			}
		return gamma;
		}
		else{return 0;};
	  
}


void menu_serie(bool *a, bool *b, uint8_t *c, uint8_t *d){
  
    bool flag_0=*(a);
    bool flag_1=*b;
    uint8_t menu_switch=*c;
    uint8_t Tolstoi=*d;
   
    
    char salto1 []="\n\r"; 

    char escape =0x1B; //"\e";
    char aceptar=0xD; //"\r";
    char color_rojo[]		= "\033[0;31;40m" ;
    char color_verde[]		= "\033[0;32;40m" ;
    char color_amarillo[] 	= "\033[0;33;40m" ;
    char color_azul[]	 	= "\033[0;34;40m" ;
    char color_blanco[] 	= "\033[0;37;40m" ;
    char color_reset[]		="\033[m";
    
    char m1 []="Menu Principal\n\r";
    char m2 []="1.Editar Resistencias\n\r";
    char m3 []="2.Editar offset\n\r";
    char m4 []="3.Editar registros de ganancia\n\r";
    char m5 []="4.Variable de salida de lazo de corriente\n\r";
    char m6 []="5.Accion de relé\n\r";
    char m7 []="6.Activar Modbus/Desactivar com 232\n\r";
    char salida []="Q. Salir\n\r";
    

    char m2_1[]="1.Resistores de VP - VN\n\r";
    char m2_2[]="2.Resistor Shunt\n\r";
    
    char m3_1[]="1.Offset Vp - Vn\n\r";
    char m3_2[]="2.Offset Shunt\n\r";
    
    char m4_1[]="1.Editar PGA_IA\n\r";
    char m4_2[]="2.Editar PGA_V\n\r";
    
    char m4_101[]="Elija ganancia para PGA_IA\n\r";
    char m4_102[]="Elija ganancia para PGA_V\n\r";
    char m4_00[]="1. Ganancia 1  = Fondo de escala ±500   mV\n\r";
    char m4_01[]="2. Ganancia 2  = Fondo de escala ±250   mV\n\r";
    char m4_02[]="3. Ganancia 4  = Fondo de escala ±125   mV\n\r";
    char m4_03[]="4. Ganancia 8  = Fondo de escala ±62.5  mV\n\r";
    char m4_04[]="5. Ganancia 16 = Fondo de escala ±31.25 mV\n\r";
    char m4_05[]="6. Ganancia 22 = Fondo de escala ±22.7  mV\n\r";
    
    
    char m5_0 []="Elegir Variable Proporcional\n\r";
    char m5_1 []="1.Vrms\n\r";
    char m5_2 []="2.Irms\n\r";
    char m5_3 []="3.Voltaje Pico\n\r";
    
    char m6_0 []="Rele se acciona por:\n\r";
    char m6_1 []="1.Sobre tension\n\r";
    char m6_2 []="2.Sobre corriente\n\r";
    char m6_3 []="3.Sub tension\n\r";
    char m6_4 []="4.Sub potencia\n\r";
    
    char m8[]="Insertar valor\n\r";
    char m813[]="Insertar valor en Ohms\n\r";
    char m814[]="Insertar valor en miliAmperes\n\r";
    char m815[]="Insertar valor en Volts\n\r";
    char m816[]="Insertar valor en Watts\n\r";
    char m9[]="Valor insertado es : ";
    char m10[]="0.Volver\n\r";
    char m011[]="y.Aceptar\n\r";
    char m012[]="Valor actual es : ";
    char m013[]=" Ohms\n\r";
    char m014[]=" miliAmperes\n\r";
    char m015[]=" Volts\n\r";
    char m016[]=" Watts\n\r";
    char m017[]="Cancelado\n\r";
    char m018[]="*.Aceptar\n\r";
    char m019[]="\n\r -- Boot Menu Cerrado --\n\r";

    switch(menu_switch){
        case 0:
            if(flag_1){
                Send_Text(salto1);
                Send_Text(m1);Send_Text(m2);
                Send_Text(m3);Send_Text(m4);
                Send_Text(m5);Send_Text(m6);
                Send_Text(salida);
                flag_1=0;
            }
          
            if(aux_menu=='0'){menu_switch=0; flag_1=1; aux_menu=0;}
            if(aux_menu=='1'){menu_switch=1; flag_1=1; aux_menu=0;}
            if(aux_menu=='2'){menu_switch=2; flag_1=1; aux_menu=0;}
            if(aux_menu=='3'){menu_switch=3; flag_1=1; aux_menu=0;}
            if(aux_menu=='4'){menu_switch=4; flag_1=1; aux_menu=0;}
            if(aux_menu=='5'){menu_switch=5; flag_1=1; aux_menu=0;}
            if(aux_menu=='q'||aux_menu=='Q'){Send_Text(m019);flag_0=0;}
            break;
		  
        case 1:
            if(flag_1){
                Send_Text(salto1);
                Send_Text(color_azul);
                Send_Text(m2_1);
                Send_Text(m2_2);
                Send_Text(m10);
                flag_1=0;
            }
            if(aux_menu=='0'||aux_menu==0x1B){menu_switch=0; flag_1=1; aux_menu=0;}
            if(aux_menu=='1'){menu_switch=11; flag_1=1; aux_menu=0;} 
            if(aux_menu=='2'){menu_switch=12; flag_1=1; aux_menu=0;} 			
            break;
      
        case 11: //Inserte valor, presione * para confirmar q para salir
            if(flag_1){
                Send_Text(salto1);
                Send_Text(m813);
                Send_Text(m018);
                Send_Text(salida);
                auxiliar2=0;
                flag_1=0;}
           
            if(aux_menu=='*'||aux_menu==0xD){
                Send_Text(salto1);
                Send_Text(m9);
                Tolstoi=numero_input();//Editar resistores para VP-N
                
                Send_Text(AuxBuffer);
                Send_Text(m013);
                menu_switch=0; 
                flag_1=1; 
                aux_menu=0;}

            if(aux_menu=='q'||aux_menu=='Q'||aux_menu==0x1B){
                Send_Text(m017);
                menu_switch=0; 
                flag_1=1; 
                aux_menu=0;}
            break;
          
        case 12:
            if(flag_1){
                Send_Text(salto1);
                Send_Text(m813);
                Send_Text(m018);
                Send_Text(salida);
                auxiliar2=0;
                flag_1=0;}

            if(aux_menu=='*'){
                Send_Text(salto1);
                Send_Text(m9);
                Tolstoi=numero_input();//Editar Resistores IA
                
                Send_Text(AuxBuffer);
                Send_Text(m013);
                menu_switch=0; 
                flag_1=1; 
                aux_menu=0;}

            if(aux_menu=='q'||aux_menu=='Q'){
                Send_Text(m017);
                menu_switch=0; 
                flag_1=1; 
                aux_menu=0;}
            break;
        
        case 2:
            if(flag_1){
                Send_Text(salto1);
                Send_Text(color_rojo);
                Send_Text(m3_1);
                Send_Text(m3_2);
                Send_Text(m10);
                flag_1=0;}

            if(aux_menu=='0'){menu_switch=0; flag_1=1; aux_menu=0;}
            if(aux_menu=='1'){menu_switch=21; flag_1=1; aux_menu=0;} 
            if(aux_menu=='2'){menu_switch=22; flag_1=1; aux_menu=0;} 			
            break;

        case 21:
          if(flag_1){
            Send_Text(salto1);
            Send_Text(m8);
            Send_Text(m018);
            Send_Text(salida);
            auxiliar2=0;
            flag_1=0;}
       
            if(aux_menu=='*'){
                Send_Text(salto1);
                Send_Text(m9);
                Tolstoi=numero_input();	//Editar un ofset para IVP-N
                
                Send_Text(AuxBuffer);
                
                menu_switch=0; 
                flag_1=1; 
                aux_menu=0;}

            if(aux_menu=='q'||aux_menu=='Q'){
                Send_Text(m017);
                menu_switch=0; 
                flag_1=1; 
                aux_menu=0;}
            break;

        case 22:
            if(flag_1){
                Send_Text(salto1);
                Send_Text(m8);
                Send_Text(m018);
                Send_Text(salida);
                auxiliar2=0;
                flag_1=0;}
       
            if(aux_menu=='*'){
                Send_Text(salto1);
                Send_Text(m9);
                Tolstoi=numero_input();//Editar un ofset para IAP-N
                
                Send_Text(AuxBuffer);
                
                menu_switch=0; 
                flag_1=1; 
                aux_menu=0;}

            if(aux_menu=='q'||aux_menu=='Q'){
                Send_Text(m017);
                menu_switch=0; 
                flag_1=1; 
                aux_menu=0;}
            break;
      
        case 3:
            if(flag_1){
                Send_Text(salto1);
                Send_Text(color_verde);
                Send_Text(m4_1);
                Send_Text(m4_2);
                Send_Text(m10);
                flag_1=0;}
            
                if(aux_menu=='0'){menu_switch=0; flag_1=1; aux_menu=0;}
                if(aux_menu=='1'){menu_switch=31; flag_1=1; aux_menu=0;}
                if(aux_menu=='2'){menu_switch=32; flag_1=1; aux_menu=0;}
            break;

        case 31:
            if(flag_1){
                Send_Text(salto1);
                Send_Text(m4_101);
                Send_Text(m4_01);
                Send_Text(m4_02);
                Send_Text(m4_03);
                Send_Text(m4_04);
                Send_Text(m4_05);
                Send_Text(salida);
                flag_1=0;}
          
            //Cambiar PGA_IA
            if(aux_menu=='2'){Send_Text(salto1);Send_Text(m4_01);Tolstoi=1;menu_switch=3; flag_1=1; aux_menu=0;}
            if(aux_menu=='3'){Send_Text(salto1);Send_Text(m4_02);Tolstoi=2;menu_switch=3; flag_1=1; aux_menu=0;}
            if(aux_menu=='4'){Send_Text(salto1);Send_Text(m4_03);Tolstoi=3;menu_switch=3; flag_1=1; aux_menu=0;}
            if(aux_menu=='5'){Send_Text(salto1);Send_Text(m4_04);Tolstoi=4;menu_switch=3; flag_1=1; aux_menu=0;}
            if(aux_menu=='6'){Send_Text(salto1);Send_Text(m4_05);Tolstoi=5;menu_switch=3; flag_1=1; aux_menu=0;}
            
            if(aux_menu=='q'||aux_menu=='Q'){
                menu_switch=0; 
                flag_1=1; 
                aux_menu=0;}
            break;
        
        case 32:
            if(flag_1){
                Send_Text(salto1);
                Send_Text(m4_102);
                Send_Text(m4_00);
                Send_Text(m4_01);
                Send_Text(m4_02);
                Send_Text(m4_03);
                Send_Text(m4_04);
                Send_Text(salida);

                flag_1=0;}

            if(aux_menu=='1'){Send_Text(salto1);Send_Text(m4_00);Tolstoi=0;menu_switch=3; flag_1=1; aux_menu=0;}//Cambiar PGA_V
            if(aux_menu=='2'){Send_Text(salto1);Send_Text(m4_01);Tolstoi=1;menu_switch=3; flag_1=1; aux_menu=0;}//
            if(aux_menu=='3'){Send_Text(salto1);Send_Text(m4_02);Tolstoi=2;menu_switch=3; flag_1=1; aux_menu=0;}//
            if(aux_menu=='4'){Send_Text(salto1);Send_Text(m4_03);Tolstoi=3;menu_switch=3; flag_1=1; aux_menu=0;}//
            if(aux_menu=='5'){Send_Text(salto1);Send_Text(m4_04);Tolstoi=4;menu_switch=3; flag_1=1; aux_menu=0;}//

            if(aux_menu=='q'||aux_menu=='Q'){
                menu_switch=0; 
                flag_1=1; 
                aux_menu=0;}
            break;

          
        case 4:
            if(flag_1){
                Send_Text(salto1);
                Send_Text(color_amarillo);
                Send_Text(m5_0);
                Send_Text(m5_1);
                Send_Text(m5_2);
                Send_Text(m5_3);
                Send_Text(m10);
                flag_1=0;}
        
            if(aux_menu=='0'){menu_switch=0; flag_1=1; aux_menu=0;}
            if(aux_menu=='1'){menu_switch=41;  aux_menu=0;}
            if(aux_menu=='2'){menu_switch=42; flag_1=1; aux_menu=0;}
            if(aux_menu=='3'){menu_switch=43; flag_1=1; aux_menu=0;}
            break;
        
        case 41:
            Send_Text(m5_1); 
            Tolstoi=1;//Proporcional a Vrms
            aux_menu=0;
            menu_switch=4;
            flag_1=1;
            break;
        
        case 42:
            Send_Text(m5_2); 
            Tolstoi=2;//Proporcional a Irms
            aux_menu=0;
            menu_switch=4;
            flag_1=1;
            break;
        
        case 43:
                Send_Text(m5_3); 
                Tolstoi=3;//Proporcional a VPeak
                aux_menu=0;
                menu_switch=4;
                flag_1=1;
                break;
                  
        case 5:
          if(flag_1){
                Send_Text(salto1);
                Send_Text(color_reset);
                Send_Text(m6_0);
                Send_Text(m6_1);
                Send_Text(m6_2);
                Send_Text(m6_3);
                Send_Text(m6_4);
                Send_Text(m10) ;
                flag_1=0;}
                
                if(aux_menu=='0'){menu_switch=0;  flag_1=1; aux_menu=0;}
                if(aux_menu=='1'){
                        Send_Text(salto1);Send_Text(m6_1);Send_Text(m815);
                        menu_switch=51;  flag_1=1; aux_menu=0;auxiliar2=0;}
                if(aux_menu=='2'){
                        Send_Text(salto1);Send_Text(m6_2);Send_Text(m814);
                        menu_switch=52; flag_1=1; aux_menu=0;auxiliar2=0;}
                if(aux_menu=='3'){
                        Send_Text(salto1);Send_Text(m6_3);Send_Text(m815);
                        menu_switch=53; flag_1=1; aux_menu=0;auxiliar2=0;}
                if(aux_menu=='4'){
                        Send_Text(salto1);Send_Text(m6_4);Send_Text(m816);
                        menu_switch=54; flag_1=1; aux_menu=0;auxiliar2=0;}
        break;
          
        case 51: //sobre tension
                if(aux_menu=='*'){
                        Send_Text(salto1);
                        Send_Text(m9);
                        Tolstoi=numero_input();
                        Send_Text(AuxBuffer);
                        Send_Text(m015);
                        menu_switch=5; 
                        flag_1=1; 
                        aux_menu=0;}
                
                if(aux_menu=='q'||aux_menu=='Q'){
                        menu_switch=0; 
                        flag_1=1; 
                        aux_menu=5;}
        break;
        
        case 52://sobre corriente
                if(aux_menu=='*'){
                        Send_Text(salto1);
                        Send_Text(m9);
                        Tolstoi=numero_input();
                        Send_Text(AuxBuffer);
                        Send_Text(m014);
                        menu_switch=5; 
                        flag_1=1; 
                        aux_menu=0;}
          
                if(aux_menu=='q'||aux_menu=='Q'){
                        menu_switch=0; 
                        flag_1=1; 
                        aux_menu=5;}
                break;
        
        case 53://sub tension
                if(aux_menu=='*'){
                        Send_Text(salto1);
                        Send_Text(m9);
                        Tolstoi=numero_input();
                        Send_Text(AuxBuffer);
                        Send_Text(m015);
                        menu_switch=5; 
                        flag_1=1; 
                        aux_menu=0;}
                
                if(aux_menu=='q'||aux_menu=='Q'){
                        menu_switch=0; 
                        flag_1=1; 
                        aux_menu=5;}
        break;
        
        case 54://sub potencia
            if(aux_menu=='*'){
                Send_Text(salto1);
                Send_Text(m9);
                Tolstoi=numero_input();
                Send_Text(AuxBuffer);
                Send_Text(m016);
                menu_switch=5; 
                flag_1=1; 
                aux_menu=0;}

            if(aux_menu=='q'||aux_menu=='Q'){
                menu_switch=0; 
                flag_1=1; 
                aux_menu=5;}
                break;

    }
    *a=flag_0;
    *b=flag_1;
    *c=menu_switch;
    *d=Tolstoi;
} 