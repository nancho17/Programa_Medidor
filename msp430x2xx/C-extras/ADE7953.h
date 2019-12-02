#ifndef __ADE7953_H
#define __ADE7953_H

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

//*****************************************************************************
//
// Definiciones de las direcciones de los registros.
// 
//
//*****************************************************************************

//#define ADC10_A_CLOCKSOURCE_MCLK                                  (ADC10SSEL_2)
//#define ADC10_A_CLOCKSOURCE_SMCLK                                 (ADC10SSEL_3)

//*****************************************************************************
//
// Prototypos.
//
//*****************************************************************************

//extern bool ADC10_A_init(uint16_t baseAddress,
//                         uint16_t sampleHoldSignalSourceSelect,
//                         uint8_t clockSourceSelect,
//                         uint16_t clockSourceDivider);
  
#include "stdint.h"
#include "stdbool.h"
#include "msp430.h"  
  
extern void ADE_Lectura_1ms_TIMING( uint8_t* );

extern void ADE_Lectura_0ms5_TIMING( uint8_t* );

extern uint8_t ADE_Interruptor_RX( int );

extern bool Escritura_ADE795 ( uint8_t );

int get_wellsended(void);

void set_wellsended(int);

#ifdef __cplusplus
}
#endif

#endif
