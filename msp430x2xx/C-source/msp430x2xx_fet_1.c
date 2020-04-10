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

/*--------- Numero de Version ----------*/
#define HW_VERSION 	"1.00"
#define Escribir_en_FLASH   0  // 0 Normal 1 Escribir setup por primera vez
#define Menus_solo_rs232    0  // 1 Solo ingreso por 232 0 Ingreso segun menu


/*---------Custom_Pin_Functions----------*/

//P1
#define MX_SHUTDOWN	(0x40)  //active low
#define MX_ENABLE 	(0x80)  //active low

//P2
#define ISL_R_ENABLE	(0x40)  //active low
#define ISL_D_ENABLE	(0x80)

#define SEND_485        (0xC0)
#define RECIEVE_485	(0x00)

//P3
#define ON_PCB_RELE 	(0x02)
#define ADE_RESET 	(0x04)
#define ADE_IRQ 	(0x08)  // no conectado
#define ADE_RX 		(0x10)
#define ADE_TX 		(0x20)
#define SERIAL_DIN 	(0x40)
#define SERIAL_ROUT 	(0x80)

//P6
#define LED_V1		(0x01)
#define LED_V2  	(0x02)
#define SALIDA_DAC 	(0x40)

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

/*---------Definicion de estructura---------*/
typedef struct  {
    // boot_menu
    //2
    uint32_t Resistores_VP_N;
    uint32_t Shunt_IAP_N;
    //3
    uint8_t PGA_IA;
    uint8_t PGA_V;
    //4
    uint32_t voltaje_AVGAIN; //4,194,304
    uint32_t corriente_AIGAIN; //4,194,304
    uint32_t potencia_WGAIN; //4194304
    //5
    uint16_t proporcional_4_20_;
    uint16_t offset_4_20_;
		  
} boot_menu;

typedef struct  {
	uint8_t         baud_rate; // 1 9600 2 4800 3 2400 
	bool            bits_datos; //
	uint8_t         paridad;
        bool            bits_parada;
        uint8_t         direccion_slave;
        bool            modbus_modo;
        bool            modbus_puerto;
        uint8_t         salida4_20;
        uint8_t         alarma4_20;
        uint32_t        valoralarma4_20;
        bool            flag4_20;
        bool            flag_reset_energy;
        bool            flag_GUARDAR;
        
		  
} estructura_cliente;  // (5 8bit)  (1 32bit) (7 8 bit)   56 + 32 + 40 = 88 + 40 = 128 bit - 16 bytes 

/*---------Functions_Declarations---------*/
//void Set_DCO_using32kHz(void);
void Set_DCO_1MHzstored(void);
void UART0_P3_config (void);
bool PORT_send ( uint8_t);
void Send_Text ( char*);
int numero_input( void);
void menu_serie(bool*,bool*, uint8_t*,boot_menu *,float *);
void menu_cliente(bool*,bool*, uint8_t*,estructura_cliente *,float *);

/*---------NO_Optimized_Variables---------*/
static volatile uint32_t ms_ticks = 0;
static volatile uint32_t ms2_ticks = 0;
static volatile uint32_t ten_ms_ticks = 0;

static volatile bool tick_1ms_elapsed = false;

static volatile bool tick_500ms_elapsed = false;
static volatile bool tick_200ms_elapsed = false;
static volatile bool tick_100ms_elapsed = false;

static volatile bool tick_10s_elapsed = false;


/*---------Global_Variables---------*/

static uint8_t DATA_ADE[3];     // Buffer RX0

double E_Acumulator=0;          // Variable para almacenar datos de energía medida (En Joules)

uint8_t TablaDatos[47];  	// Tabla con datos de los registros del ADE7953ade

uint8_t auxiliar=0;		// Manejo de RX0

uint8_t auxiliar2=0;		// Manejo de RX1

uint8_t aux_menu=0;		// Manejo menu RX1

char AuxBuffer[16];  	        // Buffer consola serial

/*Iniciadores de Menu Serie - Global */
bool flag_2= true;		//Bandera del menu serie - Modbus interrupt

bool flag_485= false;		//Bandera del menu serie - 485




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
   
      //CCR0+=36; // El timer interrumpe cada 1,0986328 ms
        CCR0+=37; // El timer interrumpe cada 1,1291503 ms
}

/*Funcion que maneja la interrucpion del tmer 1*/
void Time_Handler_2(void) {
    ms2_ticks++;
    
    
    tick_100ms_elapsed = true;
  
    if (ms2_ticks > 1 ) {
        tick_200ms_elapsed = true; //200 ms
        ms2_ticks=0;
    }
    
    if (ten_ms_ticks < 100 ) {
        ten_ms_ticks++; //10s
    }
    else {
        tick_10s_elapsed = true;
    }
    CCR1+=3276; // El timer interrumpe cada 100 ms
}

/*---------Vectores de interrupcion---------*/

/*Interruptor 0 Timer A*/
#pragma vector=TIMERA0_VECTOR
__interrupt void Timer_A_0(void){
  
  LPM1_EXIT;
//  if(tick_10s_elapsed){  Ten_Second_Waiter(); }
    Time_Handler_1();

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
  if(flag_2){
		while (!(UC1IFG&UCA1TXIFG));               // USCI_A1 TX buffer ready?
		aux_menu=UCA1RXBUF;
		if(auxiliar2>15){auxiliar=0;}
		AuxBuffer[auxiliar2]=aux_menu;
		//UCA1TXBUF = aux_menu;                     // TX -> RXed character
		auxiliar2++;
  } else{
		callInterruptRX(  );  
  };

}

/*// Manejo interrupción TX0
#pragma vector=USCIAB0TX_VECTOR
__interrupt void USCI0TX_ISR(void)
{
    LPM1_EXIT;
 
}
*/


/*Main Function*/
int main(void)
{
    WDTCTL = WDTPW+WDTHOLD;               // Stop watchdog timer



    /*------Seteo parametros predeterminados ------*/
    boot_menu medidor_setup = {
        911,//Resistores_VP_N=911,
        400,//Shunt_IAP_N=400,
        2,//PGA_IA =2,
        1,//PGA_V=1,
        4194304,//voltaje_AVGAIN=4194304,
        4194304,//corriente_AIGAIN=4194304,
        4194304,//potencia_WGAIN=4194304,
        1,//proporcional_4_20_ = 1,
        0//offset_4_20 =0
	};

#if Escribir_en_FLASH == 1
      
    estructura_cliente setup_inicial={
        1,//baud_rate=1, 1 9600 2 48
        0,//bits_datos=0, 1 7bit 0 8bit
        0,//paridad=0, 0 ninguna 1 impar 2 par 
        1,//bits_parada=0, 0 2bit 1 1bit
        5,//direccion_slave=5,
        0,//modbus_modo=0, // 0 RTU 1 ANSCII
        0,//modbus_puerto=0 (485)     1 (232)
        1,//salida4_20=1; 1 Tension 2 Corriente 3 Potencia activa 4 Potencia reactiva 5 Frecuencia 6 Factor de Potencia
        1,//alarma4_20=1  1 Tension 2 Corriente 3 Potencia activa 4 Potencia reactiva 5 Frecuencia 6 Factor de Potencia
        100,//valoralarma4_20=100
        0,//flag4_20=0  //0 Inactiva rele //1 activa rele        
        0,//flag_reset_energy
        0//flag_GUARDAR   
        };
    //
    FCTL2 = FWKEY + FSSEL0 + FN1;             // MCLK/3 for Flash Timing Generator (257 kHz a 476 kHz)
    estructura_cliente *Flash_ptr;                          // Flash pointer
  
    Flash_ptr = (estructura_cliente*)0x1040;               // Initialize Flash pointer
    FCTL3 = FWKEY;                            // Clear Lock bit

    FCTL1 = FWKEY + ERASE + EEI;              // Set Erase bit, allow interrupts
    Flash_ptr->paridad = 0;                           // Dummy write to erase Flash seg

    
    FCTL1 = FWKEY + WRT;
    *Flash_ptr = setup_inicial;     
   
  
    
    FCTL1 = FWKEY;                            // Clear WRT bit
    FCTL3 = FWKEY + LOCK;                     // Set LOCK bit
    //

#else
    estructura_cliente setup_inicial;
    estructura_cliente *Flash_ptr;                          // Flash pointer
    Flash_ptr = (estructura_cliente*)0x1040;               // Initialize Flash pointer
    setup_inicial = *Flash_ptr ;

#endif    
    

    //Edicion
    estructura_cliente setup_editado;
    setup_editado=setup_inicial;
    setup_inicial.flag_GUARDAR=1;
    
    /*---Delays---*/
    for (int i = 0; i < 0xfffe; i++);     // Delay for XTAL stabilization
    Set_DCO_1MHzstored();
	
    /*------Configuracion de Pines---*/
    P4DIR |= 0xFF;                       // Set P2.0 to output direction
    P4OUT = 0x00;
    P3SEL &=0x00;  
    P3DIR |= 0x0F;
    P3OUT= ADE_RESET;
    UART0_P3_config();

    P6DIR = 0x03;
    P6OUT = 0x00;
  	
    /*------ Salida RS232 o 485-----*/
    P1DIR |= 0xC0;
    P2DIR |= 0xC0;
    
    //232
    P1OUT &= ~ MX_ENABLE;
    P1OUT |=MX_SHUTDOWN;
    //485
    //P1OUT &= ~MX_SHUTDOWN;
    //P1OUT |=MX_ENABLE;

    //232
    P2OUT &= ~ISL_D_ENABLE;
    P2OUT |=ISL_R_ENABLE;
    //485
    //recibir
    //P2OUT &= ~ISL_R_ENABLE;
    //P2OUT &= ~ISL_D_ENABLE;
#if Menus_solo_rs232 == 0	
	if(!(setup_inicial.modbus_puerto)){
			flag_485=true;
			//apagar 232
			P1OUT &= ~MX_SHUTDOWN;
			P1OUT |=MX_ENABLE;
			//recibir
			P2OUT &= ~ISL_R_ENABLE;
			P2OUT &= ~ISL_D_ENABLE; 
			set_485_flag(flag_485); } 
#endif				
        

    /*Delay*/
	UCA0TXBUF=0;
    for (int r=0;r<2;r=r+1)
    {
        P4OUT ^= 0xFF;                      // Toggle P1.0 using exclusive-OR
        __delay_cycles(100000);
    }
    
    /*-------------Seteo e inicio de los timers-----------*/
    CCR0 =48; //65535; 0.250 ms
    CCR1 =15; //65535; 4 ms
    CCTL0=0;
    CCTL1=0;
    TACTL = TASSEL_1 + MC_2+TAIE;              // ACLK, contmode,TAIFG interrupt request
    /*Activo 1er timer*/
    CCTL0 = CCIE;       // CCR0 interrupt enabled + compare mode
    /*Activo 2do timer*/
    CCTL1 = CCIE;       // CCR1 interrupt enabled + compare mode
    	
    /*-----------------Modbus registros ------------------------*/
    uint16_t * regs;    // variable para el manejo de registro modbus
    regs=GetRegBuff();
    SetRegStart (7001); // Inicio del registro

    /*-----------Manejo del loop de milisegundo-------------*/
    uint8_t a =0x00;            //Contador usado en el loop controlado por tiempo
    uint8_t TablaTemporal[47];  // Tabla para la captura de datos del ADE7953      
	
    auxiliar=0;		//contador para la recepcion en UART0
    static int be=55;     	//contador para la tabla de datos

    
    float LazoCorriente =0x00; //Variable a enviar al lazo de corriente
    float Tabla_floats[13];
      
    /*Iniciadores de Menu Serie*/
    bool flag_0= false;			//Bandera del menu serie
    bool flag_0_calib= false;		//Bandera del menu calbracion
	bool flag_modbus= false;
    
    bool flag_1= true;			//Manejo de submenus serie
    int8_t menu_switch=0;	//Posiciones del menu serie
    
    
    /*interrupt enable*/
    __bis_SR_register(GIE);       
  	


	/*-----------------------Entro al Loop Principal controlado por tiempo -----------------------------*/
/*--------------------------------------------------------------------------------------------------*/
    while (1) //milisecond controlled Loop
    {
    /*--------------------Menu Serie--------- -----------------*/	
    if (!tick_10s_elapsed) {
                    P6OUT|= 0x02;
        for (int r=0;r<14;r=r+1){
            if(AuxBuffer[r]=='c' || AuxBuffer[r]=='C'){
                if(AuxBuffer[r+1]=='f' || AuxBuffer[r+1]=='F'){
                    if(AuxBuffer[r+2]=='g' || AuxBuffer[r+2]=='G'){aux_menu=0; flag_0=true; tick_10s_elapsed=true;}
                }
            }
        }
        
    }

    if (!tick_10s_elapsed) {

        for (int r=0;r<14;r=r+1){
            if(AuxBuffer[r]=='s' || AuxBuffer[r]=='S'){
                if(AuxBuffer[r+1]=='r' || AuxBuffer[r+1]=='R'){
                    if(AuxBuffer[r+2]=='v' || AuxBuffer[r+2]=='V'){aux_menu=0; flag_0_calib=true; tick_10s_elapsed=true;}
                }
            }
        }

    }

    
    /*---------------Acciones Posguardado-------*/
    if(setup_editado.flag_GUARDAR  ){
        setup_inicial=setup_editado;
        setup_editado.flag_GUARDAR=0;

        FCTL2 = FWKEY + FSSEL0 + FN1;             // MCLK/3 for Flash Timing Generator (257 kHz a 476 kHz)
        estructura_cliente *Flash_ptr;                          // Flash pointer
      
        Flash_ptr = (estructura_cliente*)0x1040;               // Initialize Flash pointer
        FCTL3 = FWKEY;                            // Clear Lock bit
        FCTL1 = FWKEY + ERASE + EEI;              // Set Erase bit, allow interrupts
        Flash_ptr->paridad=0;           // Dummy write to erase Flash seg
       
        FCTL1 = FWKEY + WRT ;//+ BLKWRT;

        (*Flash_ptr) = setup_editado;

        FCTL1 = FWKEY;                            // Clear WRT bit
        FCTL3 = FWKEY + LOCK;                     // Set LOCK bit
        }
      
    if(setup_inicial.flag_GUARDAR && tick_10s_elapsed && (!flag_0) ){
        setup_inicial.flag_GUARDAR=0;
        /*UART*/
        UCA1CTL0=0;
        if(setup_inicial.bits_datos){
            UCA1CTL0|=UC7BIT; }
        if(!(setup_inicial.bits_parada)){
            UCA1CTL0|=UCSPB; } 

        switch(setup_inicial.baud_rate){
            case 1: //9600
            UCA1BR0 = 0x03;     
            UCA1BR1 = 0x00;         
            UCA1MCTL = UCBRS_3;
            break;              
            case 2: //4800
            UCA1BR0 = 0x06;     
            UCA1BR1 = 0x00;         
            UCA1MCTL = UCBRS_7;
            break;
            case 3://2400
            UCA1BR0 = 13;     
            UCA1BR1 = 0x00;         
            UCA1MCTL = UCBRS_6;
            break;
            case 4://1200
            UCA1BR0 = 27;     
            UCA1BR1 = 00;
            UCA1MCTL = UCBRS_2;
            break;
            }
        
        switch(setup_inicial.paridad){
        case 0: break;
        case 1: UCA1CTL0|=UCPEN; break;
        case 2: UCA1CTL0|=UCPEN+UCPAR; break;
        }

        /*Modbus*/
        if(setup_inicial.modbus_puerto){
            flag_485=false;
            //activar 232
            P1OUT &= ~ MX_ENABLE;
            P1OUT |=MX_SHUTDOWN;
            //apagar 485
            P2OUT &= ~ISL_D_ENABLE;
            P2OUT |=ISL_R_ENABLE; } 
        else {
            flag_485=true;
            //apagar 232
            P1OUT &= ~MX_SHUTDOWN;
            P1OUT |=MX_ENABLE;
            //recibir
            P2OUT &= ~ISL_R_ENABLE;
            P2OUT &= ~ISL_D_ENABLE; }
            set_485_flag(flag_485);
			flag_modbus=true;       
    }

    
    /* Escribir los valores por default */
//    if (flag_0_calib) {// con un flag
//        menu_serie(&flag_0_calib,&flag_1,&menu_switch,&medidor_setup,Tabla_floats);
//                }
    
            //else{ 		  	P6OUT&= 0xFF;}
    
//        if (flag_0) {// con un flag
//            menu_cliente(&flag_0,&flag_1,&menu_switch,&medidor_setup,Tabla_floats);
//            }
//        else if(tick_10s_elapsed){
//            /*Inicia Modbus*/
//            eMBInit(MB_RTU, 0X05, 0, 9600, MB_PAR_EVEN); //RTU  Direccion_de_esclavo:5 9600bps Paridad:Par     (para ASCII: MB_ASCII)
//            eMBEnable();
//        }
    
    /*--------------------Tarea Lectura del ADE -----------------*/		  
    if ( tick_1ms_elapsed ) {
		if(be<50){
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
				break;	}
		}
    
                    
            switch(be){
      //Escribo en registros
                case 55:	if (Escritor_Dir_8  (PGA_IA_8, 0x01, &a )) {  auxiliar=0; be=0;}; break; 
                
                 
      //Leo registros de configuracion
                case  0:  if (Lector_Dir_8  (LCYCMODE_8    , &a )) {  auxiliar=0; be=1;};  break;
                case  1:  if (Lector_Dir_8  (LAST_OP_8   , &a )) { auxiliar =0;   be=2;};  break;
                case  2:  if (Lector_Dir_8  (PGA_IA_8    , &a )) { auxiliar =0; be=3;};  break;
                case  3:  if (Lector_Dir_24 (AP_NOLOAD_24 , &a )) { auxiliar =0; be=6;};  break;
                case  6:  if (Lector_Dir_16 (CONFIG_16    , &a )) { auxiliar =0; be=8;};  break;
                case  8:  if (Lector_Dir_16 (CFMODE_16    , &a )) { auxiliar =0; be=10;};  break;
      //Leo registros de medicion
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
                
                default: be=be+1;   break;  }
        
        tick_1ms_elapsed  = false; // Reset the flag (signal 'handled')
        
        if(be==50){ /// Ejecuta el menu
            if (flag_0) {// con un flag
                menu_cliente(&flag_0,&flag_1,&menu_switch,&setup_editado,Tabla_floats);
                }
            else if(tick_10s_elapsed&&(!flag_0_calib)&&flag_modbus){
			  flag_modbus=false;
              P6OUT &= ~ 0x01;
              /*Inicia Modbus*/ 
              eMBInit(setup_inicial.modbus_modo, setup_inicial.direccion_slave, 0, 9600, setup_inicial.paridad); //RTU  Direccion_de_esclavo:5 9600bps Paridad:Par     (para ASCII: MB_ASCII)
              eMBEnable();
            }
        }       
    }
    

    /*---------------------------------Calculo de Datos----------------------*/		
    if (tick_200ms_elapsed) {
              
        static int32_t Voltaje_Medido, I_Medido, P_Medido, Q_Medido, I_rms,E_Activa,E_Reactiva;
        static uint32_t Voltaje_rms , Voltaje_pico, kiloWh;
        static float Voltaje_Ins_F,Voltaje_rms_F, Voltaje_p_F, Corriente_F, I_rms_F, Pow_F, Q_F;
        static float E_Activa_F,E_Reactiva_F,Frecuenciahz;
             
        static float PF_F;
        static int16_t PowerFactor, Periodo;

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
        Periodo = ( ((uint16_t) TablaDatos[43] << 8) + (uint16_t) TablaDatos[42] );

        /*------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
        Añadir calculos relacionados con
        medidor_setup.Resistores_VP_N;
        medidor_setup.Shunt_IAP_N;
        ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/

        Frecuenciahz=223750/(Periodo+1);
        Voltaje_Ins_F=(Voltaje_Medido*495.5)/1664000000;
        //  495.5V ->	6,500,000 *256 32bit			  
        Voltaje_rms_F=(Voltaje_rms*350.3714100779)/2312193792;    // 353,5 mv rms ->(9032007 24 bit) en chip (9032007*256 =2312193792 32bit)
        Voltaje_p_F=(Voltaje_pico*495.5)/1664000000; //no encuentro datos en datasheet
        // 24bit unsigned... 
        Corriente_F=(((float) I_Medido)*2.5)/1664000000;  //0.1 ohm 2.5 ampere 0.250mv 
        I_rms_F=((float)(I_rms)*1.7677669529664)/2312193792;    
        Pow_F=(P_Medido*619.375)/1244774656;    // 619.375 W ? 4862401 LSBs (decimal) 
        Q_F=(Q_Medido*619.375)/1244774656;    // 619.375 VAr ? 4862401 LSBs (decimal)
        E_Activa_F=E_Activa*(25084.6875/2147483392);// Watt segundo (Joule)
        E_Reactiva_F=E_Reactiva*(25084.6875/2147483392);// Watt segundo 
        E_Acumulator=E_Acumulator+E_Activa_F;
        kiloWh=E_Acumulator/(3600*1000);

        PF_F=((float)PowerFactor)/32767; //32767;

        float multitodo, multi_total;
        
        /*----------- Ejecucion de Relé  (Alarma)--------*/
        if(setup_inicial.flag4_20){
            
            //inicio alarma
            switch( setup_inicial.alarma4_20){
            case 1:  multitodo= Voltaje_rms_F;      break;
            case 2:  multitodo= I_rms_F*1000;       break;
            case 3:  multitodo= Pow_F;              break;
            case 4:  multitodo= Q_F;                break;
            case 5:  multitodo= Frecuenciahz;       break;
            case 6:  multitodo= PF_F;                          break;                         
            }                  
            if(multitodo>(setup_inicial.valoralarma4_20)){
                //P6OUT|= 0x02;
                P3OUT|= 0x02; }
            else {
              P3OUT&= ~0x02;}   //P6OUT&= ~0x02;}

        } 
        /*----------- Lazo de corriente --------*/
        switch( setup_inicial.salida4_20){
            case 1: multitodo=Voltaje_rms_F;  multi_total=220;  break;
            case 2: multitodo= I_rms_F*1000; multi_total=2500;  break;
            case 3: multitodo= Pow_F;        multi_total=619.375;   break;
            case 4: multitodo= Q_F;          multi_total=619.375;   break;
            case 5: multitodo= Frecuenciahz; multi_total=80;   break;
            case 6: multitodo= PF_F;         multi_total=1;   break;                         
            }
        LazoCorriente=((1.6)*(multitodo/multi_total))+0.4; // 2V a 0.4V 
        
        if(setup_inicial.flag_reset_energy){
            E_Acumulator=0;
            setup_inicial.flag_reset_energy=0; }

        /* Armado de tabla de floats */
        Tabla_floats[0]=Voltaje_Ins_F; 
        Tabla_floats[1]=Voltaje_rms_F; 
        Tabla_floats[2]=Voltaje_p_F;   
        Tabla_floats[3]=Corriente_F;   
        Tabla_floats[4]=I_rms_F;              
        Tabla_floats[5]=Pow_F;
        Tabla_floats[6]=Q_F;
        Tabla_floats[7]=E_Activa_F;
        Tabla_floats[8]=E_Reactiva_F;
        Tabla_floats[9]=E_Acumulator;
        Tabla_floats[10]=kiloWh;
        Tabla_floats[11]=PF_F;
        Tabla_floats[12]=Frecuenciahz;
        /*---------------------------*/
        /* - Seteo registros del Modbus - */
        //1

        SendFloat pack;
        pack.efe=Voltaje_Ins_F;
        *(regs+0)=pack.A[1];
        *(regs+1)=pack.A[0];
        //2
        pack.efe=Voltaje_rms_F;
        *(regs+2)=pack.A[1];
        *(regs+3)=pack.A[0];
        //3
        pack.efe=Corriente_F;
        *(regs+4)=pack.A[1];
        *(regs+5)=pack.A[0];
        //4
        pack.efe=I_rms_F;
        *(regs+6)=pack.A[1];
        *(regs+7)=pack.A[0];
        //5
        pack.efe=Pow_F;
        *(regs+8)=pack.A[1];
        *(regs+9)=pack.A[0];
        //6
        pack.efe=Q_F;
        *(regs+10)=pack.A[1];
        *(regs+11)=pack.A[0];
        //7
        pack.efe=PF_F;
        *(regs+12)=pack.A[1];
        *(regs+13)=pack.A[0];
        //8
        pack.efe=Frecuenciahz;
        *(regs+14)=pack.A[1];
        *(regs+15)=pack.A[0];
        //9
        pack.efe=E_Reactiva_F;
        *(regs+16)=pack.A[1];
        *(regs+17)=pack.A[0];
        //10
        pack.efe=kiloWh;
        *(regs+18)=pack.A[1];
        *(regs+19)=pack.A[0];

        //
        if (flag_0_calib) {// con un flag
            menu_serie(&flag_0_calib,&flag_1,&menu_switch,&medidor_setup,Tabla_floats); }
        //
        tick_200ms_elapsed = false; // Reset the flag (signal 'handled')
        }

    /*		Tarea DA		*/
    if (tick_500ms_elapsed) {
    /* Salida Del lazo de corriente*/
    
    DAC12_0CTL = DAC12IR+ DAC12AMP_5 + DAC12ENC+DAC12SREF1;        // Int ref gain 1 ( +DAC12SREF1 referencia externa)
    DAC12_0DAT = (uint16_t)(LazoCorriente*4095/2.498) ;   // 1.0V (2.5V = 0x0FFFh)
    
    
    
    /*Leds */
    P6OUT^= 0x02;
    if( setup_inicial.modbus_puerto){
        P6OUT^= 0x01;}
    else{P6OUT &= ~ 0x01;}
    
    tick_500ms_elapsed = false; // Reset the flag (signal 'handled')
    }
    
    /* Tarea Modbus! (Se activa si sale del menu)*/
    if(tick_10s_elapsed&&(!flag_0)&&(!flag_0_calib)){
        flag_2=false;
        eMBPoll();
    }	
    //LPM1;
};

    //__bis_SR_register(LPM0_bits); // Enter LPM0 w/ interrupt    
    //No Llega aqui
        
    }







/*-----------------------------------------------Functions-----------------------------------------------*/
/*-------------------------------------------------------------------------------------------------------*/
bool PORT_send ( uint8_t alfa){ 
    /*UCAxTXIFG is automatically reset if a character is written to UCAxTXBUF.*/
    if (UC1IFG&UCA1TXIFG){
        // while(!UC1IFG&UCA1TXIFG){};
        UCA1TXBUF =alfa;
        return 1;
    }
    return 0;
}

void Send_Text ( char* beta){    
  //  __disable_interrupt();
    if(flag_485){
        P2OUT =SEND_485;
        __delay_cycles(512); }
    
    while((*beta)>0){
        if(PORT_send((*beta)))
            {beta++;}
    };
    if(flag_485){
        __delay_cycles(2048);
        P2OUT =RECIEVE_485;}
    //__enable_interrupt();

    //return;
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




int numero_input( void) {

  		int ese=0, scale=1;
		int gamma=0;
		for (int r=0;r<16;r=r+1){
		  if(AuxBuffer[r]=='*' || AuxBuffer[r]==0xD){
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


/*------------------ Menu Client ---  CFG --------------------------------*/

bool extra_flag_G=false;
int contador_medidores=0;
void menu_cliente(bool *P_flag_0, bool *P_flag_1, uint8_t *P_M_switch, estructura_cliente *inicial,float *Mediciones){
    
    char Fbuffer[24];
    float escribir_float;
    int escribir_int;
    
    /* Cadenas de caracteres*/
    char FORM_FEED_T[]="\f";
   // char color_rojo[]		= "\033[0;31;40m";
    char color_verde[]		= "\033[0;32;40m";
    char color_amarillo[] 	= "\033[0;33;40m";
    //char color_azul[]	 	= "\033[0;34;40m";
    char color_cyan[]	 	= "\033[0;36;40m";
    char color_blanco[] 	= "\033[0;37;40m";
    char color_reset[]		= "\033[m";
    
    char salto1 []="\n\r"; // 0x0A 0x0D
   // char espacios []="  ";
    char error []="\n\r    ----------- Error -----------    \n\r \n\r \n\r  ";
    
    char m0_0 []="PROSER MENÚ - 380V,S/N:SV-0500003 - VER: ";
    char m0_00[]=HW_VERSION;
    char m0_1 []="\n\r* MENU PRINCIPAL *\n\r";
    
    char m1 []="1.Visualización Variables.\n\r";
    char m2 []="2.Puerto serie.\n\r";
    char m3 []="3.Salida 4- 20mA.\n\r";
    char m4 []="4.Alarma.\n\r";

    char m7[]="\n\r -- Boot Menu Cerrado --\n\r";
   
    /* VISUALIZACION DE VARIABLES */
    char m1_0[]="* VISUALIZACION DE VARIABLES *\n\r";
    char m1_1[]="Tensión (V): ";
    char m1_2[]="Corriente (A): ";
    char m1_3[]="Potencia Activa (W): ";
    char m1_4[]="Potencia Reactiva (W): ";
    char m1_5[]="Frecuencia (Hz): ";
    char m1_6[]="Factor de potencia: ";
    char m1_7[]="Totalizado(kWh): ";
 
    char m_0_D[]="D: Aplicar valores por defecto globalmente.\n\r";
    char m_0_G[]="G: Guardar configuracion.\n\r \n\r";
    char m_0_G_1[]="No se han guardado cambios, desea guardar la configuración?\n\r";
    char m_0_G_2[]="Guardado\n\r";
    char m_0_G_Y[]="S. Si\n\r";
    char m_0_G_N[]="N. No\n\r";
    
    char salida[]="Esc. Abandonar terminal.\n\r";


    char m_R[]="R: Reset totalizado. \n\r";
    char m_D[]="D: Aplica configuracion por defecto.\n\r \n\r";
    char m_D_0[]="Por defecto aplicado!\n\r";

    char m_ESC[]="ESC: Menu principal.\n \r \n \r";
    
    /* CONFIGURACION PUERTO SERIE */
    char m2_0[]="* CONFIGURACION PUERTO SERIE *\n\r";
    char m2_1[]="1:Baud Rate: ";
    char m2_1_M[]="Elegir Baud Rate:\n\r";
    char m2_1_1[]="9600\n\r";
    char m2_1_10[]="1.9600\n\r";
    char m2_1_2[]="4800\n\r";
    char m2_1_20[]="2.4800\n\r";
    char m2_1_3[]="2400\n\r";
    char m2_1_30[]="3.2400\n\r";
    char m2_1_4[]="1200\n\r";
    char m2_1_40[]="4.1200\n\r";
   
    char m2_2[]="2:Bits de datos: ";
    char m2_2_M[]="Elige cantidad de bits:\n\r";
    char m2_2_1[]="7 Bits\n\r";
    char m2_2_10[]="1. 7 Bits\n\r";
    char m2_2_2[]="8 Bits\n\r";
    char m2_2_20[]="2. 8 Bits\n\r";
    
    char m2_3[]="3:Paridad: ";
    char m2_3_M[]="Elige la Paridad:\n\r";
    char m2_3_1[]="NINGUNA\n\r";
    char m2_3_2[]="PAR\n\r";
    char m2_3_3[]="IMPAR\n\r";
    char m2_3_10[]="1.NINGUNA\n\r";
    char m2_3_20[]="2.PAR\n\r";
    char m2_3_30[]="3.IMPAR\n\r";
    
    char m2_4[]="4:Bits de parada: ";
    char m2_4_M[]="Elige cantidad de bits:\n\r";
    char m2_4_1[]="1\n\r";
    char m2_4_2[]="2\n\r";
    char m2_4_10[]="1. 1\n\r";
    char m2_4_20[]="2. 2\n\r";
   
    char m2_5[]="5:Dirección MODBUS: ";
    char m2_5_M[]="Ingrese direccion slave de MODBUS\n\r";

    char m2_6[]="6:Protocolo modo: ";
    char m2_6_M[]="Ingrese el modo del protocolo\n\r";
    char m2_6_1[]="ASCII\n\r";
    char m2_6_2[]="RTU\n\r";
    char m2_6_10[]="1.ASCII\n\r";
    char m2_6_20[]="2.RTU\n\r";

    char m2_7[]="7:Protocolo tipo: ENRON\n\r";

    char m2_8[]="8:Puerto elegido: ";
    char m2_8_M[]="Ingrese puerto a usar\n\r";
    char m2_8_1[]="RS232\n\r";
    char m2_8_2[]="RS485\n\r";
    char m2_8_10[]="1.RS232\n\r";
    char m2_8_20[]="2.RS485\n\r";
    
    //m_D
    //m_ESC

    /* SALIDA ANALOGICA 4-20mA */
    char m3_0_0[]="* SALIDA ANALOGICA 4-20mA *\n\r\n\r";
    char m3_0_1[]="Salida 4-20mA proporcional a: "; //Tensión
    char m3_0_2[]="Cambiar Variable: Seleccione una opcion (1-6)\n\r \n\r";
 
    char m3_1[]="1-Tensión.\n\r";
    char m3_10[]="Tensión\n\r";
    char m3_2[]="2-Corriente.\n\r";
    char m3_20[]="Corriente\n\r";
    char m3_3[]="3-Potencia Activa.\n\r";
    char m3_30[]="Potencia Activa\n\r";
    char m3_4[]="4-Potencia Reactiva.\n\r";
    char m3_40[]="Potencia Reactiva\n\r";
    char m3_5[]="5-Frecuencia.\n\r";
    char m3_50[]="Frecuencia\n\r";
    char m3_6[]="6-Factor de potencia.\n\r";
    char m3_60[]="Factor de potencia\n\r";
    //m_D
    //m_ESC
    
    /*char m4_0_0[]="* ALARMA *\n\r\n\r";*/
    char m4_0_0[]="* ALARMA *\n\r\n\r";
    char m4_S[]="S:Salida a Rele: ";  
    char m4_S_0[]="INACTIVA\n\r\n\r";
    char m4_S_1[]="ACTIVA\n\r \n\r";
    
    char m4_L[]="L:Limite para activación: ";//100

    char m4_L_0[]="Inserte valor de la variable para limite de activacion\n\r"; 
    char m4_L_1[]="Variable ingresada\n\r";//100
    char m4_L_2[]="Cancelado\n\r";//100
    
    //m3_0_2    char m4_0_1[]="Cambiar Variable: Seleccione una opcion (1-6)\n\r";
    
    //m3_1 char m4_1[]="1-Tensión.\n\r";
    //m3_2 char m4_2[]="2-Corriente.\n\r";
    //m3_3 char m4_3[]="3-Potencia Activa.\n\r";
    //m3_4 char m4_4[]="4-Potencia Reactiva.\n\r";
    //m3_5 char m4_5[]="5-Frecuencia.\n\r";
    //m3_6 char m4_6[]="6-Factor de potencia.\n\r";
    char m4_0_2[]="Variable asociada: ";//tension
    //m_D
    //m_ESC

    switch(*P_M_switch){
        case 0:
            if(*P_flag_1){
              
                Send_Text(FORM_FEED_T);
         
                Send_Text(color_verde);
                Send_Text(m0_0); Send_Text(m0_00);
                Send_Text(salto1); Send_Text(m0_1);
                Send_Text(color_reset);
                
                Send_Text(color_cyan);
                Send_Text(m1);
                Send_Text(m2);
                Send_Text(m3);
                Send_Text(m4);
        
                Send_Text(m_0_D);
                Send_Text(m_0_G);
                Send_Text(salida);
       
                Send_Text(color_reset);
                *P_flag_1=0;
            }
          
            if(aux_menu=='0'){*P_M_switch=0; *P_flag_1=1; aux_menu=0; }
            if(aux_menu=='1'){*P_M_switch=1; *P_flag_1=1; aux_menu=0; }
            if(aux_menu=='2'){*P_M_switch=2; *P_flag_1=1; aux_menu=0; }
            if(aux_menu=='3'){*P_M_switch=3; *P_flag_1=1; aux_menu=0; }
            if(aux_menu=='4'){*P_M_switch=4; *P_flag_1=1; aux_menu=0; }
            if(aux_menu=='D'||aux_menu=='d'){
                
                Send_Text(color_amarillo);
                Send_Text(m_D_0);
                Send_Text(color_reset);
                *P_M_switch=0;
                *P_flag_1=1;
                aux_menu=0;
                /*Valores por defecto*/
                inicial->baud_rate=1;
                inicial->bits_datos=0;
                inicial->paridad=0;
                inicial->bits_parada=1;
                inicial->direccion_slave=5;
                inicial->modbus_modo=1;
                inicial->modbus_puerto=0;
                inicial->salida4_20=1;
                inicial->alarma4_20=1;
                inicial->valoralarma4_20=100;
                inicial->flag4_20=0;
                inicial->flag_reset_energy=0;
                inicial->flag_GUARDAR=0;
            }
            if(aux_menu=='G'||aux_menu=='g'){
                Send_Text(color_amarillo);
                Send_Text(m_0_G_2);
                Send_Text(color_reset);
                *P_M_switch=0;
                *P_flag_1=1; aux_menu=0;
                inicial->flag_GUARDAR=1;
                extra_flag_G=true;
            }
            
            if(aux_menu=='q'||aux_menu=='Q'||aux_menu==0x1B){
                if(extra_flag_G){
                Send_Text(m7);
                *P_flag_0=0;
                }else{
                *P_M_switch='q';
                *P_flag_1=1;
                aux_menu=0; 
                }
            }
        break;

        
        case 'q':
            if(*P_flag_1){
                
                Send_Text(FORM_FEED_T);

                Send_Text(m_0_G_1);

                Send_Text(color_cyan);
                Send_Text(m_0_G_Y);
                Send_Text(m_0_G_N);
                Send_Text(salto1);
                
                Send_Text(m_ESC);

                Send_Text(color_reset);
                
                *P_flag_1=0;
            }
            Send_Text(color_amarillo);

            if(aux_menu=='N'||aux_menu=='n'){
                Send_Text(m7);
                *P_flag_0=0; }
            if(aux_menu=='S'||aux_menu=='s'){
                inicial->flag_GUARDAR=1;
                Send_Text(m_0_G_2);
                Send_Text(m7);
                *P_flag_0=0;}
            
            if(aux_menu=='q'||aux_menu=='Q'||aux_menu==0x1B){
                *P_M_switch=0;
                *P_flag_1=1;
                aux_menu=0;
            }
            Send_Text(color_reset);

            
            
        break;

        case 1:
            if(*P_flag_1){



                Send_Text(color_verde);
                Send_Text(FORM_FEED_T);
                Send_Text(color_verde);
                Send_Text(m1_0);
                Send_Text(color_reset);
                
                Send_Text(m1_1);
                escribir_float=*(Mediciones+1) ;//+1 Voltaje RMS
                    snprintf(Fbuffer, sizeof(Fbuffer), "%f",  escribir_float);
                    Send_Text(Fbuffer); Send_Text(salto1);
                
                Send_Text(m1_2);
                escribir_float=*(Mediciones+4) ;//+4 Corriente RMS 
                    snprintf(Fbuffer, sizeof(Fbuffer), "%f",  escribir_float);
                    Send_Text(Fbuffer); Send_Text(salto1);
                
                
                Send_Text(m1_3);
                escribir_float=*(Mediciones+5) ;//+5 Potencia Activa
                    snprintf(Fbuffer, sizeof(Fbuffer), "%f",  escribir_float);
                    Send_Text(Fbuffer); Send_Text(salto1);
                
                Send_Text(m1_4);
                escribir_float=*(Mediciones+6) ;//+6 Potencia Reactiva
                    snprintf(Fbuffer, sizeof(Fbuffer), "%f",  escribir_float);
                    Send_Text(Fbuffer); Send_Text(salto1);
                
                Send_Text(m1_5);
                escribir_float=*(Mediciones+12) ;//+12 Frecuencia
                    snprintf(Fbuffer, sizeof(Fbuffer), "%f",  escribir_float);
                    Send_Text(Fbuffer); Send_Text(salto1);
                
                Send_Text(m1_6);
                escribir_float=*(Mediciones+11) ;//+11 Factor de potencia
                    snprintf(Fbuffer, sizeof(Fbuffer), "%f",  escribir_float);
                    Send_Text(Fbuffer); Send_Text(salto1);
                
                Send_Text(m1_7);
                escribir_float=*(Mediciones+10) ;//+10 Energia KWh (Totalizado)
                    snprintf(Fbuffer, sizeof(Fbuffer), "%f",  escribir_float);
                    Send_Text(Fbuffer); Send_Text(salto1);
                
                if(inicial->flag_reset_energy){Send_Text(color_amarillo);}
                
                Send_Text(salto1);
                Send_Text(m_R);
               
                Send_Text(color_cyan);
                Send_Text(salto1);
                Send_Text(m_D);
                Send_Text(m_ESC);
                Send_Text(color_reset);
                
                *P_flag_1=0;}

            contador_medidores++;
            if(contador_medidores>2){  contador_medidores=0;*P_M_switch=1; *P_flag_1=1; }
            if(aux_menu==0x1B){*P_M_switch=0; *P_flag_1=1; aux_menu=0;}
            if(aux_menu=='D'||aux_menu=='d'){
                Send_Text(color_amarillo); Send_Text(m_D_0); Send_Text(color_reset);
                *P_M_switch=1; *P_flag_1=1; aux_menu=0;
                inicial->flag_reset_energy=0;}
            
            if(aux_menu=='R'||aux_menu=='r'){*P_M_switch=1; *P_flag_1=1; aux_menu=0;auxiliar2=0;
            inicial->flag_reset_energy=1; contador_medidores=999;} //Resetear totalizado
            break;
            
            case 2:
            if(*P_flag_1){
              
                Send_Text(FORM_FEED_T);
                Send_Text(color_verde);

                Send_Text(m2_0);
                Send_Text(color_blanco);
                
                Send_Text(m2_1);//Baud rate
                Send_Text(color_cyan);
                switch(inicial->baud_rate){
                    case 1:
                    Send_Text(m2_1_1); break;
                    case 2:
                    Send_Text(m2_1_2); break;
                    case 3:
                    Send_Text(m2_1_3); break;
                    case 4:
                    Send_Text(m2_1_4); break;
                    default: Send_Text(error);break;
                    }
                Send_Text(color_reset);

                Send_Text(m2_2);//Bits de datos
                Send_Text(color_cyan);
                if(inicial->bits_datos){
                    Send_Text(m2_2_1);}
                else{Send_Text(m2_2_2);}
                Send_Text(color_reset);

                Send_Text(m2_3);//Paridad
                Send_Text(color_cyan);
                switch( inicial->paridad ){
                    case 0:
                    Send_Text(m2_3_1); break;
                    case 2:
                    Send_Text(m2_3_2); break;
                    case 1:
                    Send_Text(m2_3_3); break;
                    default: Send_Text(error);break;
                }
                Send_Text(color_reset);

                Send_Text(m2_4);//Bits de parada
                Send_Text(color_cyan);
                if(inicial->bits_parada ){
                    Send_Text(m2_4_1);
                    }
                else{Send_Text(m2_4_2);}
                Send_Text(color_reset);

                Send_Text(m2_5);//Direccion slave Modbus
                Send_Text(color_cyan);
                Send_Text(color_cyan);
                escribir_int=inicial->direccion_slave;
                snprintf(Fbuffer, sizeof(Fbuffer), "%d", 0, escribir_int);
                Send_Text(Fbuffer);
                Send_Text(color_reset);
                Send_Text(salto1);
                
                Send_Text(m2_6);// Modo de protocolo
                Send_Text(color_cyan);
                if(inicial->modbus_modo){
                    Send_Text(m2_6_1);
                    }
                else{Send_Text(m2_6_2);};
                Send_Text(color_reset);

                Send_Text(m2_7); // Protocolo tipo ENRON

                Send_Text(m2_8);// Puerto elegido
                Send_Text(color_cyan);
                if(inicial->modbus_puerto){
                    Send_Text(m2_8_1);
                    }
                else{Send_Text(m2_8_2);}
                Send_Text(salto1);
                Send_Text(m_D);
                Send_Text(m_ESC);
                Send_Text(color_reset);
                
                
                *P_flag_1=0;}
            
            if(aux_menu==0x1B){*P_M_switch=0; *P_flag_1=1; aux_menu=0;}
            if(aux_menu=='1'){*P_M_switch=21; *P_flag_1=1; aux_menu=0;} 
            if(aux_menu=='2'){*P_M_switch=22; *P_flag_1=1; aux_menu=0;} 
            if(aux_menu=='3'){*P_M_switch=23; *P_flag_1=1; aux_menu=0;} 
            if(aux_menu=='4'){*P_M_switch=24; *P_flag_1=1; aux_menu=0;} 
            if(aux_menu=='5'){*P_M_switch=25; *P_flag_1=1; aux_menu=0; auxiliar2=0;} 
            if(aux_menu=='6'){*P_M_switch=26; *P_flag_1=1; aux_menu=0;} 
            if(aux_menu=='8'){*P_M_switch=28; *P_flag_1=1; aux_menu=0;} 
            if(aux_menu=='D'||aux_menu=='d'){
                Send_Text(color_amarillo); Send_Text(m_D_0); Send_Text(color_reset);
                inicial->baud_rate=1; 
                inicial->bits_datos=0;
                inicial->paridad=1;
                inicial->bits_parada=0;
                inicial->direccion_slave=5;
                inicial->modbus_modo=0;
                inicial->modbus_puerto=0;
                *P_M_switch=2; *P_flag_1=1; aux_menu=0;} 
            break;

            case 21:
            
            if(*P_flag_1){
                Send_Text(salto1);
                Send_Text(m2_1_M);//elegir baud rate
                Send_Text(color_cyan);
                Send_Text(m2_1_10); 
                Send_Text(m2_1_20); 
                Send_Text(m2_1_30); 
                Send_Text(m2_1_40); 
                Send_Text(color_reset);
                *P_flag_1=0;}
          
            if(aux_menu=='1'){Send_Text(salto1);Send_Text(color_amarillo);Send_Text(m2_1_1);Send_Text(color_reset);
            inicial->baud_rate=1;*P_M_switch=2; *P_flag_1=1; aux_menu=0;}
            if(aux_menu=='2'){Send_Text(salto1);Send_Text(color_amarillo);Send_Text(m2_1_2);Send_Text(color_reset);
            inicial->baud_rate=2;*P_M_switch=2; *P_flag_1=1; aux_menu=0;}
            if(aux_menu=='3'){Send_Text(salto1);Send_Text(color_amarillo);Send_Text(m2_1_3);Send_Text(color_reset);
            inicial->baud_rate=3;*P_M_switch=2; *P_flag_1=1; aux_menu=0;}
            if(aux_menu=='4'){Send_Text(salto1);Send_Text(color_amarillo);Send_Text(m2_1_4);Send_Text(color_reset);
            inicial->baud_rate=4;*P_M_switch=2; *P_flag_1=1; aux_menu=0;}
                     
            if(aux_menu=='q'||aux_menu=='Q'||aux_menu==0x1B){
                *P_M_switch=2; 
                *P_flag_1=1; 
                aux_menu=0;}
              
              
            break;

            case 22:
            if(*P_flag_1){
                Send_Text(salto1);
                Send_Text(m2_2_M);//elegir bits de datos
                Send_Text(color_cyan);
                Send_Text(m2_2_10); 
                Send_Text(m2_2_20); 
                Send_Text(color_reset);
                *P_flag_1=0;}
          
            if(aux_menu=='1'){Send_Text(salto1);Send_Text(color_amarillo);Send_Text(m2_2_1);Send_Text(color_reset);
            inicial->bits_datos=1;*P_M_switch=2; *P_flag_1=1; aux_menu=0;}
            if(aux_menu=='2'){Send_Text(salto1);Send_Text(color_amarillo);Send_Text(m2_2_2);Send_Text(color_reset);
            inicial->bits_datos=0;*P_M_switch=2; *P_flag_1=1; aux_menu=0;}
                     
            if(aux_menu=='q'||aux_menu=='Q'||aux_menu==0x1B){
                *P_M_switch=2; 
                *P_flag_1=1; 
                aux_menu=0;}
            break;

            case 23:
            if(*P_flag_1){
                Send_Text(salto1);
                Send_Text(m2_3_M);//elegir paridad
                Send_Text(color_cyan);
                Send_Text(m2_3_10); 
                Send_Text(m2_3_20); 
                Send_Text(m2_3_30); 
                Send_Text(color_reset);
                *P_flag_1=0;}
          
            if(aux_menu=='1'){Send_Text(salto1);Send_Text(color_amarillo);Send_Text(m2_3_1);Send_Text(color_reset);
            inicial->paridad=0;*P_M_switch=2; *P_flag_1=1; aux_menu=0;}
            if(aux_menu=='2'){Send_Text(salto1);Send_Text(color_amarillo);Send_Text(m2_3_2);Send_Text(color_reset);
            inicial->paridad=2;*P_M_switch=2; *P_flag_1=1; aux_menu=0;}
            if(aux_menu=='3'){Send_Text(salto1);Send_Text(color_amarillo);Send_Text(m2_3_3);Send_Text(color_reset);
            inicial->paridad=1;*P_M_switch=2; *P_flag_1=1; aux_menu=0;}
                     
            if(aux_menu=='q'||aux_menu=='Q'||aux_menu==0x1B){
                *P_M_switch=2; 
                *P_flag_1=1; 
                aux_menu=0;}
            break;

            case 24:
            if(*P_flag_1){
                Send_Text(salto1);
                Send_Text(m2_4_M);//
                Send_Text(color_cyan);
                Send_Text(m2_4_10);
                Send_Text(m2_4_20);
                Send_Text(color_reset);
                *P_flag_1=0;}
          
            if(aux_menu=='1'){Send_Text(salto1);Send_Text(color_amarillo);Send_Text(m2_4_1);Send_Text(color_reset);
            inicial->bits_parada=1;*P_M_switch=2; *P_flag_1=1; aux_menu=0;}
            if(aux_menu=='2'){Send_Text(salto1);Send_Text(color_amarillo);Send_Text(m2_4_2);Send_Text(color_reset);
            inicial->bits_parada=0;*P_M_switch=2; *P_flag_1=1; aux_menu=0;}
           
                     
            if(aux_menu=='q'||aux_menu=='Q'||aux_menu==0x1B){
                *P_M_switch=2; 
                *P_flag_1=1; 
                aux_menu=0;}
            break;
            
            case 25:
            if(*P_flag_1){
                Send_Text(salto1);
                Send_Text(m2_5_M);//elegir coso
                // Send_Text(color_cyan);
                // Send_Text(color_reset);
                *P_flag_1=0;}
          
            if(aux_menu=='*'||aux_menu==0xD){
                Send_Text(salto1);
                Send_Text(color_amarillo);
                Send_Text(m2_5);
                inicial->direccion_slave=numero_input();//Editar resistores para VP-N
                
                Send_Text(AuxBuffer);
                Send_Text(salto1);
                Send_Text(color_reset);
                *P_M_switch=2; 
                *P_flag_1=1; 
                aux_menu=0;}
           
            if(aux_menu=='q'||aux_menu=='Q'||aux_menu==0x1B){
                *P_M_switch=2; 
                *P_flag_1=1; 
                aux_menu=0;}
            break;

            
            case 26:
            if(*P_flag_1){
                Send_Text(salto1);
                Send_Text(m2_6_M);// Modo de protocolo
                Send_Text(color_cyan);
                Send_Text(m2_6_10);
                Send_Text(m2_6_20);
                Send_Text(color_reset);
                *P_flag_1=0;}
          
            if(aux_menu=='1'){Send_Text(salto1);Send_Text(color_amarillo);Send_Text(m2_6_1);Send_Text(color_reset);
            inicial->modbus_modo=1;*P_M_switch=2; *P_flag_1=1; aux_menu=0;}
            if(aux_menu=='2'){Send_Text(salto1);Send_Text(color_amarillo);Send_Text(m2_6_2);Send_Text(color_reset);
            inicial->modbus_modo=0;*P_M_switch=2; *P_flag_1=1; aux_menu=0;}
            break;
            
            case 28:
            if(*P_flag_1){
                Send_Text(salto1);
                Send_Text(m2_8_M);//elegir coso
                Send_Text(color_cyan);
                Send_Text(m2_8_10);     
                Send_Text(m2_8_20);
                Send_Text(color_reset);
                *P_flag_1=0;}
          
            if(aux_menu=='1'){Send_Text(salto1);Send_Text(color_amarillo);Send_Text(m2_8_1);Send_Text(color_reset);
            inicial->modbus_puerto=1;*P_M_switch=2; *P_flag_1=1; aux_menu=0;}
            if(aux_menu=='2'){Send_Text(salto1);Send_Text(color_amarillo);Send_Text(m2_8_2);Send_Text(color_reset);
            inicial->modbus_puerto=0;*P_M_switch=2; *P_flag_1=1; aux_menu=0;}
                     
           
                     
            if(aux_menu=='q'||aux_menu=='Q'||aux_menu==0x1B){
                *P_M_switch=2; 
                *P_flag_1=1; 
                aux_menu=0;}
            break;
            

            
            case 3:
            if(*P_flag_1){
                *P_flag_1=0;
                Send_Text(color_verde);
                Send_Text(color_verde);
                Send_Text(FORM_FEED_T);
                Send_Text(m3_0_0);
                Send_Text(color_reset);

                Send_Text(m3_0_1);
                Send_Text(color_cyan);
            switch(inicial->salida4_20){
                case 1:
                Send_Text(m3_10); break;
                case 2:
                Send_Text(m3_20); break;
                case 3:
                Send_Text(m3_30); break;
                case 4:
                Send_Text(m3_40); break;
                case 5:
                Send_Text(m3_50); break;
                case 6:
                Send_Text(m3_60); break;
                default: Send_Text(error);break;
                }

            Send_Text(color_reset);
            Send_Text(m3_0_2);
            Send_Text(color_cyan);
            Send_Text(m3_1);
            Send_Text(m3_2);
            Send_Text(m3_3);
            Send_Text(m3_4);
            Send_Text(m3_5);
            Send_Text(m3_6);
            
            Send_Text(salto1);
            Send_Text(m_D);
            Send_Text(m_ESC);
            Send_Text(color_reset);
            
            }
            
            if(aux_menu=='Q'|| aux_menu==0x1B){*P_M_switch=0; *P_flag_1=1; aux_menu=0;}
            if(aux_menu=='D'|| aux_menu=='d'){*P_M_switch=3; *P_flag_1=1; aux_menu=0;inicial->salida4_20=1;
                Send_Text(color_amarillo); Send_Text(m_D_0); Send_Text(color_reset);}
            if(aux_menu=='1'){*P_M_switch=3; *P_flag_1=1; aux_menu=0; inicial->salida4_20=1;} 
            if(aux_menu=='2'){*P_M_switch=3; *P_flag_1=1; aux_menu=0; inicial->salida4_20=2;} 
            if(aux_menu=='3'){*P_M_switch=3; *P_flag_1=1; aux_menu=0; inicial->salida4_20=3;} 
            if(aux_menu=='4'){*P_M_switch=3; *P_flag_1=1; aux_menu=0; inicial->salida4_20=4;} 
            if(aux_menu=='5'){*P_M_switch=3; *P_flag_1=1; aux_menu=0; inicial->salida4_20=5;} 
            if(aux_menu=='6'){*P_M_switch=3; *P_flag_1=1; aux_menu=0; inicial->salida4_20=6;} 
                        
            break;

            case 4:
            if(*P_flag_1){
            *P_flag_1=0;
            Send_Text(FORM_FEED_T);
            Send_Text(color_verde);
            Send_Text(m4_0_0);
            Send_Text(color_reset);
            Send_Text(m4_S);
            Send_Text(color_cyan);
            if(inicial->flag4_20){  Send_Text(m4_S_1);} //activa
            else{   Send_Text(m4_S_0);}
            Send_Text(color_reset);
            
            Send_Text(m4_L);
            Send_Text(color_cyan);
            escribir_int=inicial->valoralarma4_20;            //escribir limite de activacion
            snprintf(Fbuffer, sizeof(Fbuffer), "%d", 0, escribir_int);
            Send_Text(Fbuffer); Send_Text(color_reset);
                
            Send_Text(salto1);
            

            Send_Text(m3_0_2);
            Send_Text(color_cyan);
            Send_Text(m3_1);
            Send_Text(m3_2);
            Send_Text(m3_3);
            Send_Text(m3_4);
            Send_Text(m3_5);
            Send_Text(m3_6);
            Send_Text(salto1);
            Send_Text(color_reset);

            Send_Text(m4_0_2);//switch que variable
            Send_Text(color_cyan);
            switch(inicial->alarma4_20){
                case 1:
                Send_Text(m3_10); break;
                case 2:
                Send_Text(m3_20); break;
                case 3:
                Send_Text(m3_30); break;
                case 4:
                Send_Text(m3_40); break;
                case 5:
                Send_Text(m3_50); break;
                case 6:
                Send_Text(m3_60); break;
                default: Send_Text(error);break;
                }

            Send_Text(salto1);
            Send_Text(m_D);
            Send_Text(m_ESC);
            Send_Text(color_reset);
            }
            
            if(aux_menu==0x1B){*P_M_switch=0; *P_flag_1=1; aux_menu=0;}
            if(aux_menu=='1'){*P_M_switch=4; *P_flag_1=1; aux_menu=0; inicial->alarma4_20=1;} 
            if(aux_menu=='2'){*P_M_switch=4; *P_flag_1=1; aux_menu=0; inicial->alarma4_20=2;} 
            if(aux_menu=='3'){*P_M_switch=4; *P_flag_1=1; aux_menu=0; inicial->alarma4_20=3;} 
            if(aux_menu=='4'){*P_M_switch=4; *P_flag_1=1; aux_menu=0; inicial->alarma4_20=4;} 
            if(aux_menu=='5'){*P_M_switch=4; *P_flag_1=1; aux_menu=0; inicial->alarma4_20=5;} 
            if(aux_menu=='6'){*P_M_switch=4; *P_flag_1=1; aux_menu=0; inicial->alarma4_20=6;}      
            if(aux_menu=='L'||aux_menu=='l'){*P_M_switch=41; *P_flag_1=1; aux_menu=0; }      
            if(aux_menu=='S'||aux_menu=='s'){*P_M_switch=4; *P_flag_1=1; aux_menu=0; inicial->flag4_20=! inicial->flag4_20;}      
            if(aux_menu=='D'||aux_menu=='d'){*P_M_switch=4; *P_flag_1=1; aux_menu=0;
                inicial->alarma4_20=1;inicial->valoralarma4_20=100;inicial->flag4_20=0;
                Send_Text(color_amarillo); Send_Text(m_D_0); Send_Text(color_reset);}      

            break;
            
            case 41:
            inicial->valoralarma4_20=500;
            ////
            ///
              //Inserte valor, presione * para confirmar q para salir
            if(*P_flag_1){
                Send_Text(FORM_FEED_T);
                Send_Text(salto1);
                Send_Text(m4_L_0); // Inserte valor de la variable para limite de activacionr/n
                Send_Text(salto1);
                Send_Text(m_ESC);
                auxiliar2=0;
                *P_flag_1=0;}
           
            if(aux_menu=='*'||aux_menu==0xD){
                Send_Text(salto1);
                Send_Text(color_amarillo);
                Send_Text(m4_L_1);//Insertado/r/n
                Send_Text(salto1);
                Send_Text(color_reset);
                
                inicial->valoralarma4_20=numero_input();//Editar resistores para VP-N
                *P_M_switch=4; 
                *P_flag_1=1; 
                aux_menu=0;}

            if(aux_menu=='q'||aux_menu=='Q'||aux_menu==0x1B){
                Send_Text(color_amarillo);
                Send_Text(m4_L_2);//cancelador/n
                Send_Text(color_reset);
                *P_M_switch=4; 
                *P_flag_1=1; 
                aux_menu=1;}
            break;
            ///
            ////
              
            break;
    }
}

/*------------------ Menu Serie Calibracion ---  SRV --------------------------------*/

void menu_serie(bool *P_flag_0, bool *P_flag_1, uint8_t *P_M_switch, boot_menu *Setup,float *Mediciones){

  /*
    boot_menu
    //2
    uint32_t Resistores_VP_N;
    uint32_t Shunt_IAP_N;
    //3
    uint8_t PGA_IA;
    uint8_t PGA_V;
    //4
    uint32_t voltaje_AVGAIN; //4,194,304
    uint32_t corriente_AIGAIN; //4,194,304
    uint32_t potencia_WGAIN; //4194304
    //5
    uint16_t 4_20_proporcional;
    uint16_t 4_20_offset
  */
  
  
    /* --- Variables --- */
    char enter[16];
    //char floaterr[16];
    char     Fbuffer[16];
    float muestra_float;
    uint32_t muestra_int;
    /* --- Escritos --- */
    char FORM_FEED_T[]="\f";
    char salto1 []="\n\r";

    char color_verde[]		= "\033[0;32;40m" ;
    char color_amarillo[] 	= "\033[0;33;40m" ;
    char color_cyan[]	 	= "\033[0;36;40m";
    char color_reset[]		="\033[m";
    
    char m0 []="Menu Calibracion\n\r";
    char m1 []="1.Visualizador de variables\n\r";
    char m2 []="2.Editar resistencias en uso\n\r";
    char m3 []="3.Editar registros de ganancia\n\r";
    char m4 []="4.Editor de registros de calibración\n\r";
    char m5 []="5.Editor de salida 4-20 mA\n\r";

    char m_ESC[]="ESC: Menu principal.\n \r \n \r";    
    char m_ins[]="insertado\n\r";
    char m_EXIT[]="\n\r----------Menu Cerrado----------\n\r";
    
       
    /* VISUALIZACION DE VARIABLES */
    char m1_0[]="* VISUALIZACION DE VARIABLES *\n\r";
    char m1_1[]="Tensión (V): ";
    char m1_2[]="Corriente (A): ";
    char m1_3[]="Potencia Activa (W): ";
    char m1_4[]="Potencia Reactiva (W): ";
    char m1_5[]="Frecuencia (Hz): ";
    char m1_6[]="Factor de potencia: ";
    char m1_7[]="Totalizado(kWh): ";
 
    
    /* char m2 []="2.Editar resistencias en uso\n\r"; */
    char m2_1[]="1.Resistores de VP - VN (ohms): ";
    char m2_10[]="Inserte valor de resitencia en ohms\n\r";
    char m2_2[]="2.Resistor Shunt (ohms): ";
    
    char m3_1[]="1.Editar PGA_IA\n\r";
    char m3_2[]="2.Editar PGA_V\n\r";

    char m3_101[]="Elija ganancia para PGA_IA\n\r";
    char m3_102[]="Elija ganancia para PGA_V\n\r";
    char m3_00[]="1. Ganancia 1  = Fondo de escala ±500   mV\n\r";
    char m3_01[]="2. Ganancia 2  = Fondo de escala ±250   mV\n\r";
    char m3_02[]="3. Ganancia 4  = Fondo de escala ±125   mV\n\r";
    char m3_03[]="4. Ganancia 8  = Fondo de escala ±62.5  mV\n\r";
    char m3_04[]="5. Ganancia 16 = Fondo de escala ±31.25 mV\n\r";
    char m3_05[]="6. Ganancia 22 = Fondo de escala ±22.7  mV\n\r";

    char GM3_0[]="Ganancia 1 (±500 mV)\n\r";
    char GM3_1[]="Ganancia 2 (±250 mV)\n\r";
    char GM3_2[]="Ganancia 4 (±125 mV)\n\r";
    char GM3_3[]="Ganancia 8 (±62.5 mV)\n\r";
    char GM3_4[]="Ganancia 16 (±31.25 mV)\n\r";
    char GM3_5[]="Ganancia 22 (±22.7 mV)\n\r";
    
    char m4_0 []="Menu - Calibracion de variables:\n\r";
    char m4_1 []="1.Editar Registro de calibración de voltaje         : AVGAIN\n\r";
    char m4_2 []="2.Editar Registro de calibración de corriente       : AIGAIN\n\r";
    char m4_3 []="3.Editar Registro de calibración de Potencia activa : WGAIN\n\r";
    char m4_001 []="Valor actual de AVGAIN : ";
    char m4_002 []="Valor actual de AIGAIN : ";
    char m4_003 []="Valor actual de WGAIN : ";
    
    char m4_10 []="Inserte nuevo valor del registro AVGAIN\n\r";
    //    char m4_6[]="Ingrese nuevo valor del registro\n\r";

    char m5_0 []="Menu - Calibracion de salida 4-20 mA:\n\r";
    char m5_1 []="1. Modificar proporcionalidad: ajuste_proporcional \n\r";
    char m5_2 []="2. Modificar offset: ajuste_offset \n\r";
    char m5_E []="((\033[0;32;40mValor_a_representar\033[m*\033[0;36;40majuste_proporcional\033[m)+\033[0;36;40majuste_offset\033[m) = SALIDA_DAC \n\r";
    
    char m5_01 []="Insertar nuevo valor de ajuste_proporcional\n\r Valor actual: \n\r";
    char m5_02 []="Insertar nuevo valor de ajuste_offset\n\r Valor actual: \n\r";

    switch(*P_M_switch){
        case 0:
            if(*P_flag_1){
                Send_Text(FORM_FEED_T);
                Send_Text(color_verde);
                Send_Text(m0);
                Send_Text(color_reset);
                Send_Text(m1); Send_Text(m2);
                Send_Text(m3); Send_Text(m4);
                Send_Text(m5); 
                *P_flag_1=0; }
        
            if(aux_menu=='0'){*P_M_switch=0; *P_flag_1=1; aux_menu=0; }
            if(aux_menu=='1'){*P_M_switch=1; *P_flag_1=1; aux_menu=0; }
            if(aux_menu=='2'){*P_M_switch=2; *P_flag_1=1; aux_menu=0; }
            if(aux_menu=='3'){*P_M_switch=3; *P_flag_1=1; aux_menu=0; }
            if(aux_menu=='4'){*P_M_switch=4; *P_flag_1=1; aux_menu=0; }
            if(aux_menu=='5'){*P_M_switch=5; *P_flag_1=1; aux_menu=0; }
            if(aux_menu=='q'||aux_menu=='Q'||aux_menu==0x1B){
                Send_Text(m_EXIT);
                *P_flag_0=0; }
            break;

        case 1:
            if(*P_flag_1){
            Send_Text(color_verde);
            Send_Text(FORM_FEED_T);
            Send_Text(color_verde);
            Send_Text(m1_0);
            Send_Text(color_reset);

            Send_Text(m1_1);
            muestra_float=*(Mediciones+1) ;//+1 Voltaje RMS
            snprintf(Fbuffer, sizeof(Fbuffer), "%f",  muestra_float);
            Send_Text(Fbuffer); Send_Text(salto1);

            Send_Text(m1_2);
            muestra_float=*(Mediciones+4) ;//+4 Corriente RMS 
            snprintf(Fbuffer, sizeof(Fbuffer), "%f",  muestra_float);
            Send_Text(Fbuffer); Send_Text(salto1);


            Send_Text(m1_3);
            muestra_float=*(Mediciones+5) ;//+5 Potencia Activa
            snprintf(Fbuffer, sizeof(Fbuffer), "%f",  muestra_float);
            Send_Text(Fbuffer); Send_Text(salto1);

            Send_Text(m1_4);
            muestra_float=*(Mediciones+6) ;//+6 Potencia Reactiva
            snprintf(Fbuffer, sizeof(Fbuffer), "%f",  muestra_float);
            Send_Text(Fbuffer); Send_Text(salto1);

            Send_Text(m1_5);
            muestra_float=*(Mediciones+12) ;//+12 Frecuencia
            snprintf(Fbuffer, sizeof(Fbuffer), "%f",  muestra_float);
            Send_Text(Fbuffer); Send_Text(salto1);

            Send_Text(m1_6);
            muestra_float=*(Mediciones+11) ;//+11 Factor de potencia
            snprintf(Fbuffer, sizeof(Fbuffer), "%f",  muestra_float);
            Send_Text(Fbuffer); Send_Text(salto1);

            Send_Text(m1_7);
            muestra_float=*(Mediciones+10) ;//+10 Energia KWh (Totalizado)
            snprintf(Fbuffer, sizeof(Fbuffer), "%f",  muestra_float);
            Send_Text(Fbuffer); Send_Text(salto1);

            Send_Text(salto1);
            Send_Text(color_cyan);
            Send_Text(m_ESC);
            Send_Text(color_reset);
            *P_flag_1=0;}

            contador_medidores++;
            if(contador_medidores>3){  contador_medidores=0;*P_M_switch=1; *P_flag_1=1; }
            if(aux_menu==0x1B){*P_M_switch=0; *P_flag_1=1; aux_menu=0;}
            break;
            
        case 2:
            if(*P_flag_1){
              
                Send_Text(FORM_FEED_T);
                Send_Text(color_verde);
                Send_Text(m2);
                Send_Text(color_reset);
        
                Send_Text(m2_1);
                snprintf(enter, sizeof(enter), "%d", 0, Setup->Resistores_VP_N);
                Send_Text(enter); Send_Text(salto1);
        
                Send_Text(m2_2);
                snprintf(enter, sizeof(enter), "%d", 0, Setup->Shunt_IAP_N);
                Send_Text(enter); Send_Text(salto1);
                
                Send_Text(salto1);
                Send_Text(color_cyan);      
                Send_Text(m_ESC);
                Send_Text(color_reset);      
                
                *P_flag_1=0;
                }
            if(aux_menu=='0'||aux_menu==0x1B){*P_M_switch=0; *P_flag_1=1; aux_menu=0;}
            if(aux_menu=='1'){*P_M_switch=21; *P_flag_1=1; aux_menu=0;} 
            if(aux_menu=='2'){*P_M_switch=22; *P_flag_1=1; aux_menu=0;} 			
            break;
            
        case 21: //Inserte valor, presione * para confirmar q para salir
            if(*P_flag_1){
                Send_Text(salto1);
                Send_Text(m2_10);
                Send_Text(color_cyan);              
                Send_Text(m_ESC);
                Send_Text(color_reset);              
              
                auxiliar2=0;
                *P_flag_1=0;}
        
            if(aux_menu=='*'||aux_menu==0xD){
                Send_Text(color_amarillo);              
                Send_Text(m_ins); 
                Send_Text(color_reset);              
                Setup->Resistores_VP_N=numero_input();//Editar resistores para VP-N
                *P_M_switch=2; 
                *P_flag_1=1; 
                aux_menu=1;}
        
            if(aux_menu=='q'||aux_menu=='Q'||aux_menu==0x1B){
                *P_M_switch=2; 
                *P_flag_1=1; 
                aux_menu=0;}
            break;

        case 22:
            if(*P_flag_1){
                Send_Text(salto1);
                Send_Text(m2_10);
                Send_Text(color_cyan);              
                Send_Text(m_ESC);
                Send_Text(color_reset);                   auxiliar2=0;
                *P_flag_1=0;}

            if(aux_menu=='*'||aux_menu==0xD){
                Send_Text(color_amarillo);              
                Send_Text(m_ins); 
                Send_Text(color_reset);        
                Setup->Shunt_IAP_N=numero_input();//Editar Resistores IA
                *P_M_switch=2; 
                *P_flag_1=1; 
                aux_menu=0;}
        
            if(aux_menu=='q'||aux_menu=='Q'||aux_menu==0x1B){
            *P_M_switch=2; 
            *P_flag_1=1; 
            aux_menu=0;}
            break;
            
        case 3:
            if(*P_flag_1){
                Send_Text(FORM_FEED_T);
                Send_Text(color_verde);
                Send_Text(m3);
                Send_Text(color_reset);
        
                Send_Text(m3_1);
                switch(Setup->PGA_IA){
                    case 1:Send_Text(GM3_1);break;
                    case 2:Send_Text(GM3_2);break;
                    case 3:Send_Text(GM3_3);break;
                    case 4:Send_Text(GM3_4);break;
                    case 5:Send_Text(GM3_5);break; }

                Send_Text(m3_2);
                switch(Setup->PGA_V){
                    case 0:Send_Text(GM3_0);break;
                    case 1:Send_Text(GM3_1);break;
                    case 2:Send_Text(GM3_2);break;
                    case 3:Send_Text(GM3_3);break;
                    case 4:Send_Text(GM3_4);break; }
        
            Send_Text(salto1);
            Send_Text(color_cyan);
            Send_Text(m_ESC);
            Send_Text(color_reset);
            *P_flag_1=0;}
        
            if(aux_menu=='0'|| aux_menu==0x1B){*P_M_switch=0; *P_flag_1=1; aux_menu=0;}
            if(aux_menu=='1'){*P_M_switch=31; *P_flag_1=1; aux_menu=0;}
            if(aux_menu=='2'){*P_M_switch=32; *P_flag_1=1; aux_menu=0;}
            break;

        case 31:
            if(*P_flag_1){
                Send_Text(salto1);
                Send_Text(m3_101);
                Send_Text(m3_01);
                Send_Text(m3_02);
                Send_Text(m3_03);
                Send_Text(m3_04);
                Send_Text(m3_05);
                Send_Text(m_ESC);
                *P_flag_1=0;}
          
            //Cambiar PGA_IA
            if(aux_menu=='2'){Send_Text(salto1);Send_Text(m3_01);Setup->PGA_IA=1;*P_M_switch=3; *P_flag_1=1; aux_menu=0;}
            if(aux_menu=='3'){Send_Text(salto1);Send_Text(m3_02);Setup->PGA_IA=2;*P_M_switch=3; *P_flag_1=1; aux_menu=0;}
            if(aux_menu=='4'){Send_Text(salto1);Send_Text(m3_03);Setup->PGA_IA=3;*P_M_switch=3; *P_flag_1=1; aux_menu=0;}
            if(aux_menu=='5'){Send_Text(salto1);Send_Text(m3_04);Setup->PGA_IA=4;*P_M_switch=3; *P_flag_1=1; aux_menu=0;}
            if(aux_menu=='6'){Send_Text(salto1);Send_Text(m3_05);Setup->PGA_IA=5;*P_M_switch=3; *P_flag_1=1; aux_menu=0;}
            
            if(aux_menu=='q'||aux_menu=='Q'||aux_menu==0x1B){
                *P_M_switch=3; 
                *P_flag_1=1; 
                aux_menu=0;}
            break;
        
        case 32:
            if(*P_flag_1){
                Send_Text(salto1);
                Send_Text(m3_102);
                Send_Text(m3_00);
                Send_Text(m3_01);
                Send_Text(m3_02);
                Send_Text(m3_03);
                Send_Text(m3_04);
                Send_Text(m_ESC);
        
                *P_flag_1=0;}
        
            if(aux_menu=='1'){Send_Text(salto1);Send_Text(m3_00);Setup->PGA_V=0;*P_M_switch=3; *P_flag_1=1; aux_menu=0;}//Cambiar PGA_V
            if(aux_menu=='2'){Send_Text(salto1);Send_Text(m3_01);Setup->PGA_V=1;*P_M_switch=3; *P_flag_1=1; aux_menu=0;}//
            if(aux_menu=='3'){Send_Text(salto1);Send_Text(m3_02);Setup->PGA_V=2;*P_M_switch=3; *P_flag_1=1; aux_menu=0;}//
            if(aux_menu=='4'){Send_Text(salto1);Send_Text(m3_03);Setup->PGA_V=3;*P_M_switch=3; *P_flag_1=1; aux_menu=0;}//
            if(aux_menu=='5'){Send_Text(salto1);Send_Text(m3_04);Setup->PGA_V=4;*P_M_switch=3; *P_flag_1=1; aux_menu=0;}//
        
            if(aux_menu=='q'||aux_menu=='Q'||aux_menu==0x1B){
                *P_M_switch=3; 
                *P_flag_1=1; 
                aux_menu=0;}
            break;
        
        case 4:
            if(*P_flag_1){
                Send_Text(FORM_FEED_T);
                Send_Text(color_verde);
                Send_Text(m4_0);
                Send_Text(color_reset);
    
                Send_Text(m4_1);
                Send_Text(m4_2);
                Send_Text(m4_3);
                
                Send_Text(salto1);
                Send_Text(color_cyan);
                Send_Text(m_ESC);
                Send_Text(color_reset);
                *P_flag_1=0; }
            
            if(aux_menu=='0'||aux_menu==0x1B){*P_M_switch=0; *P_flag_1=1; aux_menu=0; }
            if(aux_menu=='1'){*P_M_switch=41; *P_flag_1=1; aux_menu=0;auxiliar2=0; } 
            if(aux_menu=='2'){*P_M_switch=42; *P_flag_1=1; aux_menu=0;auxiliar2=0;} 			
            if(aux_menu=='3'){*P_M_switch=43; *P_flag_1=1; aux_menu=0;auxiliar2=0;} 			
            break;
            
        case 41: //Calibrar Tension
            if(*P_flag_1){*P_flag_1=0;
            

            Send_Text(FORM_FEED_T);
            Send_Text(m4_1);
            Send_Text(m4_001);
            muestra_int = Setup->voltaje_AVGAIN ;
            snprintf(enter, sizeof(enter), "%ld", 0, muestra_int);
            Send_Text(enter); Send_Text(salto1);
            Send_Text(m4_10);Send_Text(salto1);
            Send_Text(m_ESC);
            
            }

            if(aux_menu=='*'||aux_menu==0xD){
            Setup->voltaje_AVGAIN =numero_input();//numero_input();
            Send_Text(salto1);
            Send_Text(color_amarillo);
            Send_Text(m_ins);
            Send_Text(color_reset);
            //Setup->Rele_Selec=1;
            //Setup->Rele_Value=numero_input();
            *P_M_switch=4; 
            *P_flag_1=1; 
            aux_menu=0;}

            if(aux_menu=='q'||aux_menu=='Q'||aux_menu==0x1B){
            *P_M_switch=4; 
            *P_flag_1=1; 
            aux_menu=0;}
            break;
            
        case 42: //Calibrar Corriente
            if(*P_flag_1){
            Send_Text(FORM_FEED_T);
            Send_Text(m4_2);
            Send_Text(m4_002);
            muestra_int = Setup->corriente_AIGAIN ;
            snprintf(enter, sizeof(enter), "%ld", 0, muestra_int);
            
            Send_Text(enter); Send_Text(salto1);
            Send_Text(m4_10);Send_Text(salto1);
            Send_Text(m_ESC);
            *P_flag_1=0;
            }

            if(aux_menu=='*'||aux_menu==0xD){
            Setup->corriente_AIGAIN =numero_input();
            Send_Text(salto1);
            Send_Text(color_amarillo);
            Send_Text(m_ins);
            Send_Text(color_reset);
            *P_M_switch=4; 
            *P_flag_1=1; 
            aux_menu=0;}

            if(aux_menu=='q'||aux_menu=='Q'||aux_menu==0x1B){
            *P_M_switch=4; 
            *P_flag_1=1; 
            aux_menu=0;}
            break;
            
        case 43: //Calibrar Potencia activa
            if(*P_flag_1){
            Send_Text(FORM_FEED_T);
            Send_Text(m4_3);
            Send_Text(m4_003);
            muestra_int = Setup->potencia_WGAIN ;
            snprintf(enter, sizeof(enter), "%ld", 0, muestra_int);
            Send_Text(enter); Send_Text(salto1);
            Send_Text(m4_10);Send_Text(salto1);
            Send_Text(m_ESC);
            *P_flag_1=0;
            }

            if(aux_menu=='*'||aux_menu==0xD){
            Setup->potencia_WGAIN  =numero_input();
            Send_Text(salto1);
            Send_Text(color_amarillo);
            Send_Text(m_ins);
            Send_Text(color_reset);
            *P_M_switch=4; 
            *P_flag_1=1; 
            aux_menu=0;}

            if(aux_menu=='q'||aux_menu=='Q'||aux_menu==0x1B){
            *P_M_switch=4; 
            *P_flag_1=1; 
            aux_menu=0;}
            break;
            
            
        case 5:
            if(*P_flag_1){
            Send_Text(FORM_FEED_T);
            Send_Text(color_verde);
            Send_Text(m5_0);
            Send_Text(color_reset);

            Send_Text(m5_1);
            Send_Text(m5_2);
            Send_Text(m5_E);

            Send_Text(salto1);
            Send_Text(color_cyan);
            Send_Text(m_ESC);
            Send_Text(color_reset);

            *P_flag_1=0; }

            if(aux_menu=='0'||aux_menu==0x1B){*P_M_switch=0; *P_flag_1=1; aux_menu=0; }
            if(aux_menu=='1'){*P_M_switch=51; *P_flag_1=1; aux_menu=0;auxiliar2=0; } 
            if(aux_menu=='2'){*P_M_switch=52; *P_flag_1=1; aux_menu=0;auxiliar2=0;} 			
            break;
            
        case 51: //Calibrar proporcional
            if(*P_flag_1){
            Send_Text(FORM_FEED_T);
            Send_Text(m5_01);
            muestra_float= Setup->proporcional_4_20_;
            snprintf(enter, sizeof(enter), "%f", muestra_float);
            Send_Text(enter); Send_Text(salto1);
            Send_Text(m_ESC);
            *P_flag_1=0;
            }

            if(aux_menu=='*'||aux_menu==0xD){
            Setup->proporcional_4_20_ =numero_input();
            Send_Text(salto1);
            Send_Text(color_amarillo);
            Send_Text(m_ins);
            Send_Text(color_reset);
            *P_M_switch=5; 
            *P_flag_1=1; 
            aux_menu=0;}

            if(aux_menu=='q'||aux_menu=='Q'||aux_menu==0x1B){
            *P_M_switch=5; 
            *P_flag_1=1; 
            aux_menu=0;}
            break;
            
        case 52: //Calibrar offset
            if(*P_flag_1){
            Send_Text(FORM_FEED_T);
            Send_Text(m5_02);
            muestra_float= Setup->offset_4_20_;
            snprintf(enter, sizeof(enter), "%f", muestra_float);
            Send_Text(enter); Send_Text(salto1);
            Send_Text(m_ESC);
            *P_flag_1=0;
            }

            if(aux_menu=='*'||aux_menu==0xD){
            Setup->offset_4_20_ =numero_input();
            Send_Text(salto1);
            Send_Text(color_amarillo);
            Send_Text(m_ins);
            Send_Text(color_reset);
            *P_M_switch=5; 
            *P_flag_1=1; 
            aux_menu=0;}

            if(aux_menu=='q'||aux_menu=='Q'||aux_menu==0x1B){
            *P_M_switch=5; 
            *P_flag_1=1; 
            aux_menu=0;}
            break;
            
            
    }
}
