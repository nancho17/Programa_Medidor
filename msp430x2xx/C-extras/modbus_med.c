#include <cassert>
#include <string.h>
#include "modbus_med.h"
#include "mod_ascii.h"




/* ----------------------- Type definitions ---------------------------------*/
typedef enum
{
    STATE_RX_INIT,              /*!< Receiver is in initial state. */
    STATE_RX_IDLE,              /*!< Receiver is in idle state. */
    STATE_RX_RCV,               /*!< Frame is beeing received. */
    STATE_RX_ERROR              /*!< If the frame is invalid. */
} eMBRcvState;

typedef enum
{
    STATE_TX_IDLE,              /*!< Transmitter is in idle state. */
    STATE_TX_XMIT               /*!< Transmitter is in transfer state. */
} eMBSndState;

/* ----------------------- Variables estaticas--------------------------------*/

static uint8_t    ucMBAddress;
static eMBMode  eMBCurrentMode;

static enum
{
    STATE_ENABLED,
    STATE_DISABLED,
    STATE_NOT_INITIALIZED
} eMBState = STATE_NOT_INITIALIZED;

uint8_t         ucGIEWasEnabled = false;
uint8_t         ucCriticalNesting = 0x00;

eMBException    prveMBError2Exception( eMBErrorCode eErrorCode );
/* -----------------------  RTU Definiciones--------------------------------*/
#define MB_SER_PDU_SIZE_MIN     4       /*!< Minimum size of a Modbus RTU frame. */
#define MB_SER_PDU_SIZE_MAX     256     /*!< Maximum size of a Modbus RTU frame. */
#define MB_SER_PDU_SIZE_CRC     2       /*!< Size of CRC field in PDU. */
#define MB_SER_PDU_ADDR_OFF     0       /*!< Offset of slave address in Ser-PDU. */
#define MB_SER_PDU_PDU_OFF      1       /*!< Offset of Modbus-PDU in Ser-PDU. */


/* ----------------------- Variables estaticas RTU--------------------------------*/

//static volatile eMBSndState eSndState;
//static volatile eMBRcvState eRcvState;

volatile uint8_t  ucRTUBuf[MB_SER_PDU_SIZE_MAX];

static volatile uint8_t *pucSndBufferCur;
static volatile uint16_t usSndBufferCount;

static volatile uint16_t usRcvBufferPos;


/* Functions pointer which are initialized in eMBInit( ). Depending on the
 * mode (RTU or ASCII) the are set to the correct implementations.
 */

static peMBFrameSend peMBFrameSendCur;
static pvMBFrameStart pvMBFrameStartCur;
//static pvMBFrameStop pvMBFrameStopCur;
static peMBFrameReceive peMBFrameReceiveCur;
//static pvMBFrameClose pvMBFrameCloseCur;

/* Callback functions required by the porting layer. They are called when
 * an external event has happend which includes a timeout or the reception
 * or transmission of a character.
 */

 bool( *pxMBFrameCBByteReceived ) ( void ); //

bool( *pxMBFrameCBTransmitterEmpty ) ( void );
bool( *pxMBPortCBTimerExpired ) ( void );

//BOOL( *pxMBFrameCBReceiveFSMCur ) ( void );
//BOOL( *pxMBFrameCBTransmitFSMCur ) ( void );


//
static volatile eMBSndState eSndState;
static volatile eMBRcvState eRcvState;


/* An array of Modbus functions handlers which associates Modbus function
 * codes with implementing functions.
 */
static xMBFunctionHandler xFuncHandlers[MB_FUNC_HANDLERS_MAX] = {

    {MB_FUNC_OTHER_REPORT_SLAVEID, eMBFuncReportSlaveID},

    // {MB_FUNC_READ_INPUT_REGISTER, eMBFuncReadInputRegister},

    {MB_FUNC_READ_HOLDING_REGISTER, eMBFuncReadHoldingRegister},

};

/* -------------------------- Implementaciones -------------------------------*/
eMBErrorCode
eMBInit( eMBMode eMode, uint8_t ucSlaveAddress, uint8_t ucPort, uint32_t ulBaudRate, eMBParity eParity )
{
    eMBErrorCode    eStatus = MB_ENOERR;

    /* check preconditions */
    if( ( ucSlaveAddress == MB_ADDRESS_BROADCAST ) ||
        ( ucSlaveAddress < MB_ADDRESS_MIN ) || ( ucSlaveAddress > MB_ADDRESS_MAX ) )
    {
        eStatus = MB_EINVAL;
    }
    else
    {
        ucMBAddress = ucSlaveAddress;

        switch ( eMode )
        {

        case MB_RTU:
            pvMBFrameStartCur = eMBRTUStart;
            //pvMBFrameStopCur = eMBRTUStop;
            peMBFrameSendCur = eMBRTUSend;
            peMBFrameReceiveCur = eMBRTUReceive;
            //if(pvMBFrameCloseCur == MB_PORT_HAS_CLOSE){vMBPortClose;}else{NULL;}  
            pxMBFrameCBByteReceived = xMBRTUReceiveFSM;
            pxMBFrameCBTransmitterEmpty = xMBRTUTransmitFSM;
            pxMBPortCBTimerExpired = xMBRTUTimerT35Expired;

            eStatus = eMBRTUInit( ucMBAddress, ucPort, ulBaudRate, eParity );
            break;

        case MB_ASCII:
        	pvMBFrameStartCur = eMBASCIIStart;
            //pvMBFrameStopCur = eMBASCIIStop;
            peMBFrameSendCur = eMBASCIISend;
            peMBFrameReceiveCur = eMBASCIIReceive;
            //if(pvMBFrameCloseCur == MB_PORT_HAS_CLOSE){vMBPortClose;}else{NULL;} 
           	pxMBFrameCBByteReceived = xMBASCIIReceiveFSM;
            pxMBFrameCBTransmitterEmpty = xMBASCIITransmitFSM;
            pxMBPortCBTimerExpired = xMBASCIITimerT1SExpired;

            eStatus = eMBASCIIInit( ucMBAddress, ucPort, ulBaudRate, eParity );
            break;

        default:
            eStatus = MB_EINVAL;
        }

        if( eStatus == MB_ENOERR )
        {
            eMBCurrentMode = eMode;
            eMBState = STATE_DISABLED;
        }
    }
    return eStatus;
}

eMBErrorCode eMBEnable( void )
{
    eMBErrorCode    eStatus = MB_ENOERR;

    if( eMBState == STATE_DISABLED )
    {
        /* Activate the protocol stack. */
        pvMBFrameStartCur(  );
        eMBState = STATE_ENABLED;
    }
    else
    {
        eStatus = MB_EILLSTATE;
    }
    return eStatus;
}

eMBErrorCode eMBPoll( void )
{
    static uint8_t   *ucMBFrame;
    static uint8_t    ucRcvAddress;
    static uint8_t    ucFunctionCode;
    static uint16_t   usLength;
    static eMBException eException;

    int             i;
    eMBErrorCode    eStatus = MB_ENOERR;
    eMBEventType    eEvent;

    /* Check if the protocol stack is ready. */
    if( eMBState != STATE_ENABLED )
    {
        return MB_EILLSTATE;
    }

    /* Check if there is a event available. If not return control to caller.
     * Otherwise we will handle the event. */
    if( xMBPortEventGet( &eEvent ) == true )
    {
        switch ( eEvent )
        {
        case EV_READY:
            break;

        case EV_FRAME_RECEIVED:
            eStatus = peMBFrameReceiveCur( &ucRcvAddress, &ucMBFrame, &usLength );
            if( eStatus == MB_ENOERR )
            {
                /* Check if the frame is for us. If not ignore the frame. */
                if( ( ucRcvAddress == ucMBAddress ) || ( ucRcvAddress == MB_ADDRESS_BROADCAST ) )
                {
                    ( void )xMBPortEventPost( EV_EXECUTE );
                }
            }
            break;

        case EV_EXECUTE:
            ucFunctionCode = ucMBFrame[MB_PDU_FUNC_OFF];
            eException = MB_EX_ILLEGAL_FUNCTION;
            for( i = 0; i < MB_FUNC_HANDLERS_MAX; i++ )
            {
                /* No more function handlers registered. Abort. */
                if( xFuncHandlers[i].ucFunctionCode == 0 )
                {
                    break;
                }
                else if( xFuncHandlers[i].ucFunctionCode == ucFunctionCode )
                {
                    eException = xFuncHandlers[i].pxHandler( ucMBFrame, &usLength );
                    break;
                }
            }

            /* If the request was not sent to the broadcast address we
             * return a reply. */
            if( ucRcvAddress != MB_ADDRESS_BROADCAST )
            {
                if( eException != MB_EX_NONE )
                {
                    /* An exception occured. Build an error frame. */
                    usLength = 0;
                    ucMBFrame[usLength++] = ( uint8_t )( ucFunctionCode | MB_FUNC_ERROR );
                    ucMBFrame[usLength++] = eException;
                }
                if( ( eMBCurrentMode == MB_ASCII ) && MB_ASCII_TIMEOUT_WAIT_BEFORE_SEND_MS )
                {
                    vMBPortTimersDelay( MB_ASCII_TIMEOUT_WAIT_BEFORE_SEND_MS );
                }                
                eStatus = peMBFrameSendCur( ucMBAddress, ucMBFrame, usLength );
            }
            break;

        case EV_FRAME_SENT:
            break;
        }
    }
    return MB_ENOERR;
}

/* ------------------------------- RTU ---------------------------------------*/
/* ---------------------------------------------------------------------------*/
/* ---------------------------------------------------------------------------*/

void eMBRTUStart( void )
{
      #if MODO_CRITICO_INTERRUPT == 1
      EnterCriticalSection(  );
      #endif
      
    /* Inicialmente el recepor esta en el estado  STATE_RX_INIT. 
     *comenzamos el timer y si ningn caracter es recibido en t3.5
     *cambiamos a STATE_RX_IDLE. Esto asegura que retrasemos el startup
     *del protocolo hasta que el bus se libere.
     */
    eRcvState = STATE_RX_INIT;
    vMBPortSerialEnable( true, false );
    vMBPortTimersEnable(  );

    #if MODO_CRITICO_INTERRUPT == 1
    ExitCriticalSection(  );
    #endif
}

eMBErrorCode eMBRTUInit( uint8_t ucSlaveAddress, uint8_t ucPort, uint32_t ulBaudRate, eMBParity eParity )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    uint32_t           usTimerT35_50us;

    ( void )ucSlaveAddress;
      #if MODO_CRITICO_INTERRUPT == 1
      EnterCriticalSection(  );
      #endif

    /* Modbus RTU uses 8 Databits. */
    if( xMBPortSerialInit( ucPort, ulBaudRate, 8, eParity ) != true )
    {
        eStatus = MB_EPORTERR;
    }
    else
    {
        /* If baudrate > 19200 then we should use the fixed timer values
         * t35 = 1750us. Otherwise t35 must be 3.5 times the character time.
         */
        if( ulBaudRate > 19200 )
        {
            usTimerT35_50us = 35;       /* 1800us. */
        }
        else
        {
            /* The timer reload value for a character is given by:
             *
             * ChTimeValue = Ticks_per_1s / ( Baudrate / 11 )
             *             = 11 * Ticks_per_1s / Baudrate
             *             = 11 * 32768 / Baudrate
             * The reload for t3.5 is 1.5 times this value and similary
             * for t3.5.
             */
            usTimerT35_50us = ( 7UL * 360448UL ) / ( 2UL * 9600 );
        }
        if( xMBPortTimersInit( ( uint16_t ) usTimerT35_50us ) != true )
        {
            eStatus = MB_EPORTERR;
        }
    }
    #if MODO_CRITICO_INTERRUPT == 1
    ExitCriticalSection(  );
    #endif
    return eStatus;
}

bool xMBRTUTimerT35Expired( void ){
    bool            xNeedPoll = false;

    switch ( eRcvState )
    {
        /* Timer t35 expired. Startup phase is finished. */
    case STATE_RX_INIT:
        xNeedPoll = xMBPortEventPost( EV_READY );
        break;

        /* A frame was received and t35 expired. Notify the listener that
         * a new frame was received. */
    case STATE_RX_RCV:
        xNeedPoll = xMBPortEventPost( EV_FRAME_RECEIVED );
        
        break;

        /* An error occured while receiving the frame. */
    case STATE_RX_ERROR:
        break;

        /* Function called in an illegal state. */
    default:
        if ( ( eRcvState == STATE_RX_INIT ) ||
            ( eRcvState == STATE_RX_RCV ) || ( eRcvState == STATE_RX_ERROR ) ){xNeedPoll=true;}
        else{xNeedPoll=false;};
    }

    vMBPortTimersDisable(  );
    eRcvState = STATE_RX_IDLE;

    return xNeedPoll;
}

eMBErrorCode eMBRTUSend( uint8_t ucSlaveAddress, const uint8_t * pucFrame, uint16_t usLength )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    uint16_t          usCRC16;

    #if MODO_CRITICO_INTERRUPT == 1
    EnterCriticalSection(  );
    #endif

    /* Check if the receiver is still in idle state. If not we where to
     * slow with processing the received frame and the master sent another
     * frame on the network. We have to abort sending the frame.
     */
    if( eRcvState == STATE_RX_IDLE )
    {
        /* First byte before the Modbus-PDU is the slave address. */
        pucSndBufferCur = ( uint8_t * ) pucFrame - 1;
        usSndBufferCount = 1;

        /* Now copy the Modbus-PDU into the Modbus-Serial-Line-PDU. */
        pucSndBufferCur[MB_SER_PDU_ADDR_OFF] = ucSlaveAddress;
        usSndBufferCount += usLength;

        /* Calculate CRC16 checksum for Modbus-Serial-Line-PDU. */
        usCRC16 = usMBCRC16( ( uint8_t * ) pucSndBufferCur, usSndBufferCount );
        ucRTUBuf[usSndBufferCount++] = ( uint8_t )( usCRC16 & 0xFF );
        ucRTUBuf[usSndBufferCount++] = ( uint8_t )( usCRC16 >> 8 );

        /* Activate the transmitter. */
        vMBPortSerialEnable( false, false ); 
        eSndState = STATE_TX_XMIT;
        vMBPortSerialEnable( false, true );
   
    }
    else
    {
        eStatus = MB_EIO;
    }

#if MODO_CRITICO_INTERRUPT == 1
ExitCriticalSection(  );
#endif
    return eStatus;
}

eMBErrorCode eMBRTUReceive( uint8_t * pucRcvAddress, uint8_t ** pucFrame, uint16_t * pusLength )
{
    bool xFrameReceived= false;
    
    eMBErrorCode    eStatus = MB_ENOERR;


    #if MODO_CRITICO_INTERRUPT == 1
    EnterCriticalSection(  );
    #endif
   
     assert( usRcvBufferPos < MB_SER_PDU_SIZE_MAX );
   // if( usRcvBufferPos < MB_SER_PDU_SIZE_MAX ){true;}else{false;return(MB_EILLSTATE);};
    

    /* Length and CRC check */
    if( ( usRcvBufferPos >= MB_SER_PDU_SIZE_MIN )
        && ( usMBCRC16( ( uint8_t * ) ucRTUBuf, usRcvBufferPos ) == 0 ) )
    {
        /* Save the address field. All frames are passed to the upper layed
         * and the decision if a frame is used is done there.
         */
        *pucRcvAddress = ucRTUBuf[MB_SER_PDU_ADDR_OFF];

        /* Total length of Modbus-PDU is Modbus-Serial-Line-PDU minus
         * size of address field and CRC checksum.
         */
        *pusLength = ( uint16_t )( usRcvBufferPos - MB_SER_PDU_PDU_OFF - MB_SER_PDU_SIZE_CRC );

        /* Return the start of the Modbus PDU to the caller. */
        *pucFrame = ( uint8_t * ) & ucRTUBuf[MB_SER_PDU_PDU_OFF];
        xFrameReceived = true;
    }
    else
    {
        eStatus = MB_EIO;
    }
    
    #if MODO_CRITICO_INTERRUPT == 1
    ExitCriticalSection(  );
    #endif
    
    return eStatus;
}


bool xMBRTUReceiveFSM( void )
{
    bool            xTaskNeedSwitch = false;
    uint8_t           ucByte;

    assert( eSndState == STATE_TX_IDLE );
  //  if( eSndState == STATE_TX_IDLE )
  //      {return xTaskNeedSwitch;}

    /* Always read the character. */
    ( void )xMBPortSerialGetByte( ( char * ) & ucByte );

    switch ( eRcvState )
    {
        /* If we have received a character in the init state we have to
         * wait until the frame is finished.
         */
    case STATE_RX_INIT:
        vMBPortTimersEnable(  );
        break;

        /* In the error state we wait until all characters in the
         * damaged frame are transmitted.
         */
    case STATE_RX_ERROR:
        vMBPortTimersEnable(  );
        break;

        /* In the idle state we wait for a new character. If a character
         * is received the t1.5 and t3.5 timers are started and the
         * receiver is in the state STATE_RX_RECEIVCE.
         */
    case STATE_RX_IDLE:
        usRcvBufferPos = 0;
        ucRTUBuf[usRcvBufferPos++] = ucByte;
        eRcvState = STATE_RX_RCV;

        /* Enable t3.5 timers. */
        vMBPortTimersEnable(  );
        break;

        /* We are currently receiving a frame. Reset the timer after
         * every character received. If more than the maximum possible
         * number of bytes in a modbus frame is received the frame is
         * ignored.
         */
    case STATE_RX_RCV:
        if( usRcvBufferPos < MB_SER_PDU_SIZE_MAX )
        {
            ucRTUBuf[usRcvBufferPos++] = ucByte;
        }
        else
        {
            eRcvState = STATE_RX_ERROR;
        }
        vMBPortTimersEnable(  );
        break;
    }
    return xTaskNeedSwitch;
}

bool xMBRTUTransmitFSM( void ) 
{
    bool xNeedPoll = false;
    assert( eRcvState == STATE_RX_IDLE );

    switch ( eSndState )
    {
        /* We should not get a transmitter event if the transmitter is in
         * idle state.  */
    case STATE_TX_IDLE:
        /* enable receiver/disable transmitter. */
        vMBPortSerialEnable( true, false );
        break;

    case STATE_TX_XMIT:
        /* check if we are finished. */
        if( usSndBufferCount != 0 )
        {
            xMBPortSerialPutByte( ( char )*pucSndBufferCur );
            pucSndBufferCur++;  /* next byte in sendbuffer. */
            usSndBufferCount--;
        }
        else
        {
            xNeedPoll = xMBPortEventPost( EV_FRAME_SENT );
            /* Disable transmitter. This prevents another transmit buffer
             * empty interrupt. */
            vMBPortSerialEnable( true, false );
            eSndState = STATE_TX_IDLE;

        }
        break;
    }

    return xNeedPoll;

}


/* --------------------- Hardware Port Hardware ------------------------------*/
/* --------------------- Hardware Port Hardware ------------------------------*/
/* --------------------- Hardware Port Hardware ------------------------------*/
/* --------------------- Hardware Port Hardware ------------------------------*/

/* ----------------------- Events / Auxiliar ---------------------------------*/
/*
 * Variables
 */
static eMBEventType eQueuedEvent;
static bool     xEventInQueue;

bool xMBPortEventInit( void )
{
    xEventInQueue = false;
    return true;
}

bool xMBPortEventPost( eMBEventType eEvent )
{
    xEventInQueue = true;
    eQueuedEvent = eEvent;
    return true;
}

bool xMBPortEventGet( eMBEventType * eEvent )
{
    bool        xEventHappened = false;

    if( xEventInQueue )
    {
        *eEvent = eQueuedEvent;
        xEventInQueue = false;
        xEventHappened = true;
    }
    return xEventHappened;
}
/* ----------------------- Timer  -----------------------------*/
bool xMBPortTimersInit( uint16_t usTimeOut50us ){
    
  bool            bInitialized = false;
  uint32_t           ulReloadValue = usTimeOut50us;

    if( ulReloadValue <= 1 )
    {
        ulReloadValue = 1;
    }
    else
    {
        ulReloadValue -= 1;
    }

    if( ulReloadValue < 0xFFFE )
    {
        /* Timer B clock source is ACLK, Start disabled. */
        TBCTL = TBSSEL_1;                  // SMCLK, upmode
        TBCCR0 = ( uint16_t ) ulReloadValue;
        /* Enable Timer b interrupt. */
        TBCCTL0 = CCIE;

        bInitialized = true;
    }
    return bInitialized;
}

 


void vMBPortTimersEnable( void )
{
    /* Reset timer counter and set compare interrupt. */
    TBR = 0;
    
    TBCCTL0 |= CCIE;
    TBCTL |= MC_1;
}

void vMBPortTimersDisable( void )
{
    TBCCTL0 &= ~CCIE;
    TBCTL &= ~( MC0 | MC1 );
}

#pragma vector=TIMERB0_VECTOR
__interrupt void prvvMBTimerIRQHandler (void)
{
( void )pxMBPortCBTimerExpired(  );
}

/* ----------------------- Serial  -----------------------------*/
bool xMBPortSerialInit( uint8_t ucPort, uint32_t ulBaudRate, uint8_t ucDataBits, eMBParity eParity ) 
{
    bool            bInitialized = true;
    uint16_t          UxCTL = 0;

    switch ( eParity )
    {
    case MB_PAR_NONE:
        break;
    case MB_PAR_ODD:
        UxCTL |= UCPEN;
        break;
    case MB_PAR_EVEN:
        UxCTL |= UCPEN | UCPAR;
        break;
    }
    switch ( ucDataBits )
    {
    case 8:
        
        break;
    case 7:
        UxCTL |=UC7BIT;
        break;
    default:
        bInitialized = false;
    }
    if( bInitialized )
    {
        #if MODO_CRITICO_INTERRUPT == 1
        EnterCriticalSection(  );
        #endif
        /* Reset USART */
        //UCA1CTL1 |= UCSWRST;
        /* Initialize all UART registers */
        UCA1CTL0 =0;
        UCA1CTL0 |=UxCTL;
        /* CLK = ACLK */
        UCA1CTL1 |= UCSSEL0;
        
        /* Configure Baudrate  */
        //UCA1BR0 = 0x03;                         
        //UCA1BR1 = 0x00;                         
        //UCA1MCTL = UCBRS_3;                     // Modulation UCBRSx = 6 UCBRFx=13 Oversampling  
        /* Enable UART */
        UC1IE |= UCA1RXIE; 
        /* Clear reset flag. */
         UCA1CTL1&= ~UCSWRST;

        /* USART0 TXD/RXD */
        P3SEL |= 0xC0;                          // P3.6,7 = USCI_A1 TXD/RXD

        #if MODO_CRITICO_INTERRUPT == 1
        ExitCriticalSection(  );
        #endif

        
    }
    return bInitialized;
}


void            vMBPortSerialEnable( bool xRxEnable, bool xTxEnable )
{
  #if MODO_CRITICO_INTERRUPT == 1
   EnterCriticalSection(  );
  #endif
  if( xRxEnable )
    {
       UC1IE |= UCA1RXIE;
    }
    else
    {
        UC1IE &= ~UCA1RXIE;
    }
    if( xTxEnable )
    {
        UC1IE |= UCA1TXIE;
        UC1IFG |= UCA1TXIFG ;
    }
    else
    {
        UC1IE &= ~UCA1TXIE;
    }
  
    #if MODO_CRITICO_INTERRUPT == 1
    ExitCriticalSection(  );
    #endif
  
}

/*--------------------Interrupts---------------*/
/* 	HANDLER REESCRITO EN MAIN.C
	HANDLER REESCRITO EN MAIN.C
	HANDLER REESCRITO EN MAIN.C

#pragma vector=USCIAB1RX_VECTOR
__interrupt void prvvMBSerialRXIRQHandler(void)
{
    pxMBFrameCBByteReceived(  );
}
*/

void callInterruptRX(void){
  pxMBFrameCBByteReceived();
}

#pragma vector=USCIAB1TX_VECTOR
__interrupt void prvvMBSerialTXIRQHandler(void)

{
    pxMBFrameCBTransmitterEmpty(  );
}


bool xMBPortSerialGetByte( char * getByte )
{
 
    *getByte =UCA1RXBUF;
    return true;
}



static bool bandera_485=false;
void set_485_flag (bool a) {
    
    bandera_485=a;
   }

bool xMBPortSerialPutByte( char putByte){
//485 rs485 RS485 

    if(bandera_485){
        P2OUT |=0xC0;//send
        __delay_cycles(512); }

    UCA1TXBUF=putByte;

    if(bandera_485){
    __delay_cycles(1024);  
    P2OUT &= ~0xC0; }  //recieve
    return true;
}


/* ------------------ Critical Mode! -----------------------*/
void EnterCriticalSection( void )
{
    uint16_t  usOldSR;
    if( ucCriticalNesting == 0 )
    {   
        usOldSR = GIE;
        __disable_interrupt();
        
        if(usOldSR || GIE){
            ucGIEWasEnabled=true;
            }
        else{ ucGIEWasEnabled=false;}
        
    }
    ucCriticalNesting++;
}

void ExitCriticalSection( void )
{
    ucCriticalNesting--;
    if( ucCriticalNesting == 0 )
    {
        if( ucGIEWasEnabled )
        {
            __enable_interrupt();
        }
    }
}

/*------------------------Modbus Functions------------------------------------*/
/* 
 *Funciones para callbacks 
 */
static uint8_t    ucMBSlaveID[MB_FUNC_OTHER_REP_SLAVEID_BUF];
static uint16_t   usMBSlaveIDLen;

eMBException eMBFuncReportSlaveID( uint8_t * pucFrame, uint16_t * usLen )
{
    memcpy( &pucFrame[MB_PDU_DATA_OFF], &ucMBSlaveID[0], ( size_t )usMBSlaveIDLen );
    *usLen = ( uint16_t )( MB_PDU_DATA_OFF + usMBSlaveIDLen );
    return MB_EX_NONE;
}


eMBException            eMBFuncReadHoldingRegister( uint8_t * pucFrame, uint16_t * usLen )
{
    uint16_t          usRegAddress;
    uint16_t          usRegCount;
    uint8_t          *pucFrameCur;

    eMBException    eStatus = MB_EX_NONE;
    eMBErrorCode    eRegStatus;

    if( *usLen == ( MB_PDU_FUNC_READ_SIZE + MB_PDU_SIZE_MIN ) )
    {
        usRegAddress = ( uint16_t )( pucFrame[MB_PDU_FUNC_READ_ADDR_OFF] << 8 );
        usRegAddress |= ( uint16_t )( pucFrame[MB_PDU_FUNC_READ_ADDR_OFF + 1] );
        usRegAddress++;

        usRegCount = ( uint16_t )( pucFrame[MB_PDU_FUNC_READ_REGCNT_OFF] << 8 );
        usRegCount = ( uint16_t )( pucFrame[MB_PDU_FUNC_READ_REGCNT_OFF + 1] );

        /* Check if the number of registers to read is valid. If not
         * return Modbus illegal data value exception. 
         */
        if( ( usRegCount >= 1 ) && ( usRegCount <= MB_PDU_FUNC_READ_REGCNT_MAX ) )
        {
            /* Set the current PDU data pointer to the beginning. */
            pucFrameCur = &pucFrame[MB_PDU_FUNC_OFF];
            *usLen = MB_PDU_FUNC_OFF;

            /* First byte contains the function code. */
            *pucFrameCur++ = MB_FUNC_READ_HOLDING_REGISTER;
            *usLen += 1;

            /* Second byte in the response contain the number of bytes. */
            *pucFrameCur++ = ( uint8_t ) ( usRegCount * 2 );
            *usLen += 1;

            /* Make callback to fill the buffer. */
            eRegStatus = eMBRegHoldingCB( pucFrameCur, usRegAddress, usRegCount, MB_REG_READ );
            /* If an error occured convert it into a Modbus exception. */
            if( eRegStatus != MB_ENOERR )
            {
                eStatus = prveMBError2Exception( eRegStatus );
            }
            else
            {
                *usLen += usRegCount * 2;
            }
        }
        else
        {
            eStatus = MB_EX_ILLEGAL_DATA_VALUE;
        }
    }
    else
    {
        /* Can't be a valid request because the length is incorrect. */
        eStatus = MB_EX_ILLEGAL_DATA_VALUE;
    }
    return eStatus;
}

eMBException prveMBError2Exception( eMBErrorCode eErrorCode )
{
    eMBException    eStatus;

    switch ( eErrorCode )
    {
        case MB_ENOERR:
            eStatus = MB_EX_NONE;
            break;

        case MB_ENOREG:
            eStatus = MB_EX_ILLEGAL_DATA_ADDRESS;
            break;

        case MB_ETIMEDOUT:
            eStatus = MB_EX_SLAVE_BUSY;
            break;

        default:
            eStatus = MB_EX_SLAVE_DEVICE_FAILURE;
            break;
    }

    return eStatus;
}
/*Manejo de Registro modbus*/
#define REG_HOLDING_NREGS 20    //60 Floats

static uint16_t   usRegHoldingStart = 7000;
static uint16_t   usRegHoldingBuf[REG_HOLDING_NREGS];

uint16_t* GetRegBuff ()
{ return &usRegHoldingBuf[0];}

void SetRegStart (uint16_t start)
{ usRegHoldingStart= start;
}


eMBErrorCode    eMBRegHoldingCB( uint8_t * pucRegBuffer, uint16_t usAddress, uint16_t usNRegs, eMBRegisterMode eMode )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    int             iRegIndex;

    if( ( usAddress >= usRegHoldingStart ) &&
        ( usAddress + usNRegs <= usRegHoldingStart + REG_HOLDING_NREGS ) )
    {
        iRegIndex = ( int )( usAddress - usRegHoldingStart );
        switch ( eMode )
        {
            /* Pass current register values to the protocol stack. */
        case MB_REG_READ:
            while( usNRegs > 0 )
            {
                *pucRegBuffer++ = ( unsigned char )( usRegHoldingBuf[iRegIndex] >> 8 );
                *pucRegBuffer++ = ( unsigned char )( usRegHoldingBuf[iRegIndex] & 0xFF );
                iRegIndex++;
                usNRegs--;
            }
            break;

            /* Update current register values with new values from the
             * protocol stack. */
        case MB_REG_WRITE:
            while( usNRegs > 0 )
            {
                usRegHoldingBuf[iRegIndex] = *pucRegBuffer++ << 8;
                usRegHoldingBuf[iRegIndex] |= *pucRegBuffer++;
                iRegIndex++;
                usNRegs--;
            }
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }
    return eStatus;
}

/* ------------------------------- RTU CRC 16 --------------------------------*/
/* ------------------------------- RTU CRC 16 --------------------------------*/
/* ------------------------------- RTU CRC 16 --------------------------------*/

/* ----------------------- Platform includes --------------------------------*/
static const uint8_t aucCRCHi[] = {
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40
};

static const uint8_t aucCRCLo[] = {
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7,
    0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E,
    0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9,
    0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC,
    0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
    0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32,
    0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D,
    0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 
    0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF,
    0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
    0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1,
    0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4,
    0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 
    0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA,
    0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
    0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0,
    0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97,
    0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E,
    0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89,
    0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83,
    0x41, 0x81, 0x80, 0x40
};


uint16_t usMBCRC16( uint8_t * pucFrame, uint16_t usLen )
{
    uint8_t           ucCRCHi = 0xFF;
    uint8_t           ucCRCLo = 0xFF;
    int             iIndex;

    while( usLen-- )
    {
        iIndex = ucCRCLo ^ *( pucFrame++ );
        ucCRCLo = ( uint8_t )( ucCRCHi ^ aucCRCHi[iIndex] );
        ucCRCHi = aucCRCLo[iIndex];
    }
    return ( uint16_t )( ucCRCHi << 8 | ucCRCLo );
}