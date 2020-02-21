#ifndef __MODBUS_MED_H
#define __MODBUS_MED_H


//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif


#include "stdint.h"
#include "stdbool.h"
#include "msp430.h"  

/*----------------------------- Configs --------------------------------------*/
/*Modo Critico*/
#define MODO_CRITICO_INTERRUPT 1   // 1 Activa los "CriticalSection " , 0 Desactiva;
  
/*! \brief Timeout to wait in ASCII prior to enabling transmitter.
 *
 * If defined the function calls vMBPortSerialDelay with the argument
 * MB_ASCII_TIMEOUT_WAIT_BEFORE_SEND_MS to allow for a delay before
 * the serial transmitter is enabled. This is required because some
 * targets are so fast that there is no time between receiving and
 * transmitting the frame. If the master is to slow with enabling its 
 * receiver then he will not receive the response correctly.
 */
#ifndef MB_ASCII_TIMEOUT_WAIT_BEFORE_SEND_MS
#define MB_ASCII_TIMEOUT_WAIT_BEFORE_SEND_MS    ( 0 )
#endif
   

/*! \brief Maximum number of Modbus functions codes the protocol stack
 *    should support.
 *
 * The maximum number of supported Modbus functions must be greater than
 * the sum of all enabled functions in this file and custom function
 * handlers. If set to small adding more functions will fail.
 */
#define MB_FUNC_HANDLERS_MAX                    ( 16 )
   
   
/*! \brief Number of bytes which should be allocated for the <em>Report Slave ID
 *    </em>command.
 *
 * This number limits the maximum size of the additional segment in the
 * report slave id function. See eMBSetSlaveID(  ) for more information on
 * how to set this value. It is only used if MB_FUNC_OTHER_REP_SLAVEID_ENABLED
 * is set to <code>1</code>.
 */
#define MB_FUNC_OTHER_REP_SLAVEID_BUF           ( 32 )
   
/*------------ Definiciones de las direcciones de los registros --------------*/
  
#define MB_ADDRESS_BROADCAST    ( 0 )   /*! Modbus broadcast address. */
#define MB_ADDRESS_MIN          ( 1 )   /*! Smallest possible slave address. */
#define MB_ADDRESS_MAX          ( 247 ) /*! Biggest possible slave address. */
#define MB_FUNC_NONE                          (  0 )
#define MB_FUNC_READ_COILS                    (  1 )
#define MB_FUNC_READ_DISCRETE_INPUTS          (  2 )
#define MB_FUNC_WRITE_SINGLE_COIL             (  5 )
#define MB_FUNC_WRITE_MULTIPLE_COILS          ( 15 )
#define MB_FUNC_READ_HOLDING_REGISTER         (  3 )
#define MB_FUNC_READ_INPUT_REGISTER           (  4 )
#define MB_FUNC_WRITE_REGISTER                (  6 )
#define MB_FUNC_WRITE_MULTIPLE_REGISTERS      ( 16 )
#define MB_FUNC_READWRITE_MULTIPLE_REGISTERS  ( 23 )
#define MB_FUNC_DIAG_READ_EXCEPTION           (  7 )
#define MB_FUNC_DIAG_DIAGNOSTIC               (  8 )
#define MB_FUNC_DIAG_GET_COM_EVENT_CNT        ( 11 )
#define MB_FUNC_DIAG_GET_COM_EVENT_LOG        ( 12 )
#define MB_FUNC_OTHER_REPORT_SLAVEID          ( 17 )
#define MB_FUNC_ERROR                         ( 128 )

  /* Type definitions */
    typedef enum
{
    MB_EX_NONE = 0x00,
    MB_EX_ILLEGAL_FUNCTION = 0x01,
    MB_EX_ILLEGAL_DATA_ADDRESS = 0x02,
    MB_EX_ILLEGAL_DATA_VALUE = 0x03,
    MB_EX_SLAVE_DEVICE_FAILURE = 0x04,
    MB_EX_ACKNOWLEDGE = 0x05,
    MB_EX_SLAVE_BUSY = 0x06,
    MB_EX_MEMORY_PARITY_ERROR = 0x08,
    MB_EX_GATEWAY_PATH_FAILED = 0x0A,
    MB_EX_GATEWAY_TGT_FAILED = 0x0B
} eMBException;


typedef         eMBException( *pxMBFunctionHandler ) ( uint8_t * pucFrame, uint16_t * pusLength );

typedef struct
{
    uint8_t           ucFunctionCode;
    pxMBFunctionHandler pxHandler;
} xMBFunctionHandler;
  
/* 
 *Funciones para callbacks 
 */
#define MB_PDU_FUNC_READ_ADDR_OFF               ( MB_PDU_DATA_OFF + 0)
#define MB_PDU_FUNC_READ_REGCNT_OFF             ( MB_PDU_DATA_OFF + 2 )
#define MB_PDU_FUNC_READ_SIZE                   ( 4 )
#define MB_PDU_FUNC_READ_REGCNT_MAX             ( 0x007D )
  
eMBException    eMBFuncReadHoldingRegister( uint8_t * pucFrame, uint16_t * usLen );

eMBException eMBFuncReportSlaveID( uint8_t * pucFrame, uint16_t * usLen );

/* ----------------------- Definicion de Tipos ---------------------------------*/
/* 
 * Dos modos de transmision para el Modbus serial, ASCII o RTU. 
 * RTU es mas rapido peri usa mas recursos de hardware. 
 * ASCII es mas lento pero mas confiable en conecciones lentas
 */
  typedef enum
{
    MB_RTU,                     /*!< RTU transmission mode. */
    MB_ASCII,                   /*!< ASCII transmission mode. */
    MB_TCP                      /*!< TCP mode. */
} eMBMode;

/*
  *Este valor se pasa a las funciones callback que soportan valores
  *de escirutra o lectura. Write significa que el registro debe 
  *actalizarse, reading significa que el stack necesita saber el valor
  *del registro actual.
  * eMBRegHoldingCB( ), eMBRegCoilsCB( ), eMBRegDiscreteCB( ) and 
  *   eMBRegInputCB( ).
  */
typedef enum
{
    MB_REG_READ,                /*!< Read register values and pass to protocol stack. */
    MB_REG_WRITE                /*!< Update register values. */
} eMBRegisterMode;
  
/* 
 * Errorcodes.
 */
typedef enum
{
    MB_ENOERR,                  /*!< no error. */
    MB_ENOREG,                  /*!< illegal register address. */
    MB_EINVAL,                  /*!< illegal argument. */
    MB_EPORTERR,                /*!< porting layer error. */
    MB_ENORES,                  /*!< insufficient resources. */
    MB_EIO,                     /*!< I/O error. */
    MB_EILLSTATE,               /*!< protocol stack in illegal state. */
    MB_ETIMEDOUT                /*!< timeout error occurred. */
} eMBErrorCode; 

typedef enum
{
    MB_PAR_NONE,                /*!< No parity. */
    MB_PAR_ODD,                 /*!< Impar . */
    MB_PAR_EVEN                 /*!< par . */
} eMBParity;
   
typedef enum
{
    EV_READY,                   /*!< Startup finished. */
    EV_FRAME_RECEIVED,          /*!< Frame received. */
    EV_EXECUTE,                 /*!< Execute function. */
    EV_FRAME_SENT               /*!< Frame sent. */
} eMBEventType;
/* ----------------------- Function prototypes ------------------------------*/
/*! \ingroup modbus
 * \brief Initialize the Modbus protocol stack.
 *
 *Esta funcion inicialisa el modulo ASCII o RTU y llama las funciones 
 *init para preparar el hardware. Ningun frame Modbus sera procesado 
 *hasta que se llame a eMBEnable( ).
 *
 * \param eMode If ASCII or RTU mode should be used.
 * \param ucSlaveAddress The slave address. Only frames sent to this
 *   address or to the broadcast address are processed.
 * \param ucPort The port to use. E.g. 1 for COM1 on windows. This value
 *   is platform dependent and some ports simply choose to ignore it.
 * \param ulBaudRate The baudrate. E.g. 19200. Supported baudrates depend
 *   on the porting layer.
 * \param eParity Parity used for serial transmission.
 *
 * \return If no error occurs the function returns eMBErrorCode::MB_ENOERR.
 *   The protocol is then in the disabled state and ready for activation
 *   by calling eMBEnable( ). Otherwise one of the following error codes 
 *   is returned:
 *    - eMBErrorCode::MB_EINVAL If the slave address was not valid. Valid
 *        slave addresses are in the range 1 - 247.
 *    - eMBErrorCode::MB_EPORTERR IF the porting layer returned an error.
 */
eMBErrorCode    eMBInit( eMBMode eMode, uint8_t ucSlaveAddress,
                         uint8_t ucPort, uint32_t ulBaudRate, eMBParity eParity );

/*! \ingroup modbus
 * \brief Habilita el stack del protocolo Modbus .
 *
 * This function enables processing of Modbus frames. Enabling the protocol
 * stack is only possible if it is in the disabled state.
 *
 * \return If the protocol stack is now in the state enabled it returns 
 *   eMBErrorCode::MB_ENOERR. If it was not in the disabled state it 
 *   return eMBErrorCode::MB_EILLSTATE.
 */
eMBErrorCode    eMBEnable( void );

/*! \ingroup modbus
 * \brief El pooling loop principal del stack del protocolo Modbus.
 *
 * This function must be called periodically. The timer interval required
 * is given by the application dependent Modbus slave timeout. Internally the
 * function calls xMBPortEventGet() and waits for an event from the receiver or
 * transmitter state machines. 
 *
 * \return If the protocol stack is not in the enabled state the function
 *   returns eMBErrorCode::MB_EILLSTATE. Otherwise it returns 
 *   eMBErrorCode::MB_ENOERR.
 */
eMBErrorCode    eMBPoll( void );



/*! \ingroup modbus_registers
 * \brief Callback function used if a <em>Holding Register</em> value is
 *   read or written by the protocol stack. The starting register address
 *   is given by \c usAddress and the last register is given by
 *   <tt>usAddress + usNRegs - 1</tt>.
 *
 * \param pucRegBuffer If the application registers values should be updated the
 *   buffer points to the new registers values. If the protocol stack needs
 *   to now the current values the callback function should write them into
 *   this buffer.
 * \param usAddress The starting address of the register.
 * \param usNRegs Number of registers to read or write.
 * \param eMode If eMBRegisterMode::MB_REG_WRITE the application register 
 *   values should be updated from the values in the buffer. For example
 *   this would be the case when the Modbus master has issued an 
 *   <b>WRITE SINGLE REGISTER</b> command.
 *   If the value eMBRegisterMode::MB_REG_READ the application should copy 
 *   the current values into the buffer \c pucRegBuffer.
 *
 * \return The function must return one of the following error codes:
 *   - eMBErrorCode::MB_ENOERR If no error occurred. In this case a normal
 *       Modbus response is sent.
 *   - eMBErrorCode::MB_ENOREG If the application can not supply values
 *       for registers within this range. In this case a 
 *       <b>ILLEGAL DATA ADDRESS</b> exception frame is sent as a response.
 *   - eMBErrorCode::MB_ETIMEDOUT If the requested register block is
 *       currently not available and the application dependent response
 *       timeout would be violated. In this case a <b>SLAVE DEVICE BUSY</b>
 *       exception is sent as a response.
 *   - eMBErrorCode::MB_EIO If an unrecoverable error occurred. In this case
 *       a <b>SLAVE DEVICE FAILURE</b> exception is sent as a response.
 */
eMBErrorCode    eMBRegHoldingCB( uint8_t * pucRegBuffer, uint16_t usAddress, uint16_t usNRegs, eMBRegisterMode eMode );


uint16_t * GetRegBuff (); //Convertir a func

void SetRegStart (uint16_t start);



/* ----------------------- Aux functions ----------------------------*/

bool xMBPortEventInit( void );

bool xMBPortEventPost( eMBEventType eEvent );

bool xMBPortEventGet(  /*@out@ */ eMBEventType * eEvent );


/* ----------------------- Serial port functions ----------------------------*/

bool            xMBPortSerialInit( uint8_t ucPort, uint32_t ulBaudRate,
                                   uint8_t ucDataBits, eMBParity eParity );
//void            vMBPortClose( void );

//void            xMBPortSerialClose( void );

void            vMBPortSerialEnable( bool xRxEnable, bool xTxEnable );

bool            xMBPortSerialGetByte( char * pucByte );

bool            xMBPortSerialPutByte( char ucByte );

/*
Callback de Interrupcion RX;
*/
 void callInterruptRX  ( void );
 
/*
 *Critical for msp
 */
void EnterCriticalSection( void );
void ExitCriticalSection( void ); 

/* ----------------------- Timers functions ---------------------------------*/
bool            xMBPortTimersInit( uint16_t usTimeOut50us );

//void            xMBPortTimersClose( void );

void            vMBPortTimersEnable( void );

void            vMBPortTimersDisable( void );

void            vMBPortTimersDelay( uint16_t usTimeOutMS );// Esta vacio, crear para ASCII

/*!
 * Constants which defines the format of a modbus frame. The example is
 * shown for a Modbus RTU/ASCII frame. Note that the Modbus PDU is not
 * dependent on the underlying transport.
 *
 * <code>
 * <------------------------ MODBUS SERIAL LINE PDU (1) ------------------->
 *              <----------- MODBUS PDU (1') ---------------->
 *  +-----------+---------------+----------------------------+-------------+
 *  | Address   | Function Code | Data                       | CRC/LRC     |
 *  +-----------+---------------+----------------------------+-------------+
 *  |           |               |                                   |
 * (2)        (3/2')           (3')                                (4)
 *
 * (1)  ... MB_SER_PDU_SIZE_MAX = 256
 * (2)  ... MB_SER_PDU_ADDR_OFF = 0
 * (3)  ... MB_SER_PDU_PDU_OFF  = 1
 * (4)  ... MB_SER_PDU_SIZE_CRC = 2
 *
 * (1') ... MB_PDU_SIZE_MAX     = 253
 * (2') ... MB_PDU_FUNC_OFF     = 0
 * (3') ... MB_PDU_DATA_OFF     = 1
 * </code>
 */
/* ----------------------- Defines ------------------------------------------*/
#define MB_PDU_SIZE_MAX     253 /*!< Maximum size of a PDU. */
#define MB_PDU_SIZE_MIN     1   /*!< Function Code */
#define MB_PDU_FUNC_OFF     0   /*!< Offset of function code in PDU. */
#define MB_PDU_DATA_OFF     1   /*!< Offset for response data in PDU. */

/* ----------------------- Prototypes  0-------------------------------------*/
typedef void    ( *pvMBFrameStart ) ( void );

typedef void    ( *pvMBFrameStop ) ( void );

typedef eMBErrorCode( *peMBFrameReceive ) ( uint8_t * pucRcvAddress,
                                            uint8_t ** pucFrame,
                                            uint16_t * pusLength );

typedef eMBErrorCode( *peMBFrameSend ) ( uint8_t slaveAddress,
                                         const uint8_t * pucFrame,
                                         uint16_t usLength );

typedef void( *pvMBFrameClose ) ( void );


/*------------------------------------ RTU -------------------------------*/
/*
 *RTU mode init
 */
eMBErrorCode eMBRTUInit( uint8_t slaveAddress, uint8_t ucPort, uint32_t ulBaudRate,
                             eMBParity eParity );

void            eMBRTUStart( void );
//void            eMBRTUStop( void );
eMBErrorCode    eMBRTUReceive( uint8_t * pucRcvAddress, uint8_t ** pucFrame, uint16_t * pusLength );
eMBErrorCode    eMBRTUSend( uint8_t slaveAddress, const uint8_t * pucFrame, uint16_t usLength );
bool            xMBRTUReceiveFSM( void );
bool            xMBRTUTransmitFSM( void );
//BOOL            xMBRTUTimerT15Expired( void );
  bool            xMBRTUTimerT35Expired( void );
  
  
uint16_t          usMBCRC16( uint8_t * pucFrame, uint16_t usLen );


#ifdef __cplusplus
}
#endif

#endif
