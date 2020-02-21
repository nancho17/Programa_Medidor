#ifndef __MOD_ASCII_H
#define __MOD_ASCII_H

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


  /*! \brief The character timeout value for Modbus ASCII.
 *
 * The character timeout value is not fixed for Modbus ASCII and is therefore
 * a configuration option. It should be set to the maximum expected delay
 * time of the network.
 */
#define MB_ASCII_TIMEOUT_SEC                    (  1 )


 
eMBErrorCode    eMBASCIIInit( uint8_t slaveAddress, uint8_t ucPort, uint32_t ulBaudRate, eMBParity eParity );

void            eMBASCIIStart( void );
//void            eMBASCIIStop( void );


eMBErrorCode    eMBASCIIReceive( uint8_t * pucRcvAddress, uint8_t ** pucFrame, uint16_t * pusLength );

eMBErrorCode    eMBASCIISend( uint8_t slaveAddress, const uint8_t * pucFrame, uint16_t usLength );

bool            xMBASCIIReceiveFSM( void );


bool            xMBASCIITransmitFSM( void );
bool            xMBASCIITimerT1SExpired( void );  
  

#ifdef __cplusplus
}
#endif

#endif