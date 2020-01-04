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

//8-bit Registers
#define SAGCYC_8 0x000 //SAGCYC, (R/W) Default: 0x00, Unsigned, Sag lines Cycle 
#define DISNOLOAD_8 0x001 //DISNOLOAD, (R/W) Default: 0x00, Unsigned, No-load detection disable* 
#define LCYCMODE_8 0x004 //LCYCMODE, (R/W) Default: 0x40, Unsigned, Line cycle accumulation mode configuration **
#define PGA_V_8 0x007 //PGA_V, (R/W) Default: 0x00, Unsigned, Voltage channel gain configuration (Bits[2:0])  
#define PGA_IA_8 0x008 //PGA_IA, (R/W) Default: 0x00, Unsigned, Current Channel A gain configuration (Bits[2:0])  
#define PGA_IB_8 0x009 //PGA_IB, (R/W) Default: 0x00, Unsigned, Current Channel B gain configuration (Bits[2:0]) 
#define WRITE_PROTECT_8 0x040 //WRITE_PROTECT, (R/W) Default: 0x00, Unsigned, Write protection bits (Bits[2:0]) 
#define LAST_OP_8 0x0FD //LAST_OP, (R/W) Default: 0x00, Unsigned, Contains the type (read or write) of the last successful communication (0x35 read 0xCA = write) 
#define LAST_RWDATA_8 0x0FF //LAST_RWDATA_8, (R/W) Default: 0x00, Unsigned, Contains the data from the last successful 8-bit register communication  
#define Version_8 0x702 //Version, (R/W) Default: N/A, Unsigned, Contains the silicon version number 
#define EX_REF_8 0x800 //EX_REF, (R/W) Default: 0x00, Unsigned, Reference input configuration:0 = internal 1 = external 

//16-bit Registers
#define ZXTOUT_16 0x100 //ZXTOUT, (R/W) Default:0xFFFF, Unsigned,Zero-crossing timeout
#define LINECYC_16 0x101 //LINCYC, (R/W) Default:0x0000, Unsigned,Number of half line cycles for line cycle energy accumulation mode
#define CONFIG_16 0x102 //CONFIG, (R/W) Default:0x8004, Unsigned,Configuration register***
#define CF1DEN_16 0x103 //CF1DEN, (R/W) Default:0x003F, Unsigned,CF1 frequency divider denominator. When modifying this register, two sequential write operations must be performed to ensure that the write is successful. 
#define CF2DEN_16 0x104 //CF2DEN, (R/W) Default:0x003F, Unsigned,CF2 frequency divider denominator. When modifying this register, two sequential write operations must be performed to ensure that the write is successful. 
#define CFMODE_16 0x107 //CFMODE, (R/W) Default:0x0300, Unsigned, CF output selection */
#define PHCALA_16 0x108 //PHCALA, (R/W) Default:0x0000, Signed,Phase calibration register (Current Channel A). This register is in sign magnitude format. 
#define PHCALB_16 0x109 //PHCALB, (R/W) Default:0x0000, Signed,Phase calibration register (Current Channel B). This register is in sign magnitude format. 
#define PFA_16 0x10A //PFA, (R) Default:0x0000, Signed,Power factor (Current Channel A) 
#define PFB_16 0x10B //PFB, (R) Default:0x0000, Signed,Power factor (Current Channel B) 
#define ANGLE_A_16 0x10C //ANGLE_A, (R) Default:0x0000, Signed,Angle between the voltage input and the Current Channel A input 
#define ANGLE_B_16 0x10D //ANGLE_B, (R) Default:0x0000, Signed,Angle between the voltage input and the Current Channel B input 
#define Period_16 0x11E //Period, (R) Default:0x0000, Unsigned, Period register 
#define ALT_OUTPUT_16 0x110 //ALT_OUTPUT, (R/W) Default:0x0000, Unsigned,Alternative output functions**/
#define LAST_ADD_16 0x1FE //LAST_ADD, (R) Default:0x0000, Unsigned, Contains the address of the last successful communication 
#define LAST_RWDATA_16 0x1FF //LAST_RWDATA_16, (R) Default:0x0000, Unsigned,Contains the data from the last successful 16-bit register communication 
#define Reserved_16 0x120 //Reserved, (R/W) Default:0x0000, Unsigned,This register should be set to 30h to meet the performance specified in Table 1. To modify this register, it must be unlocked by setting Register Address 0xFE to 0xAD immediately prior. (16 bit)

//24-bit and 32-bit registers
#define SAGLVL_24 0x200 //SAGLVL, (R/W) Default: 0x000000, Unsigned, Sag Voltage Level (24 bit)
#define SAGLVL_32 0x300 //SAGLVL, (R/W) Default: 0x000000, Unsigned, Sag Voltage Level (32 bit)
#define ACCMODE_24 0x201 //ACCMODE, (R/W) Default:0x000000, Unsigned, Accumulation mode(24 bit)
#define ACCMODE_32 0x301 //ACCMODE, (R/W) Default: 0x000000, Unsigned, Accumulation mode(32 bit)
#define AP_NOLOAD_24 0x203 //AP_NOLOAD, (R/W) Default: 0x00E419, Unsigned,Active power no-load level(24 bit)
#define AP_NOLOAD_32 0x303 //AP_NOLOAD, (R/W) Default: 0x00E419, Unsigned,Active power no-load level(32 bit)
#define VAR_NOLOAD_24 0x204 //VAR_NOLOAD, (R/W) Default: 0x000000, Unsigned,Reactive power no-load level(24 bit)
#define VAR_NOLOAD_32 0x304 //VAR_NOLOAD, (R/W) Default: 0x000000, Unsigned,Reactive power no-load level(32 bit)
#define VA_NOLOAD_24 0x205 //VA_NOLOAD, (R/W) Default: 0x000000, Unsigned,Apparent power no-load level(24 bit)
#define VA_NOLOAD_32 0x305 //VA_NOLOAD, (R/W) Default: 0x000000, Unsigned,Apparent power no-load level(32 bit)
#define AVA_24 0x210 //AVA, (R) Default: 0x000000, Signed,Instantaneous apparent power (Current Channel A)(24 bit)
#define AVA_32 0x310 //AVA, (R) Default: 0x000000, Signed,Instantaneous apparent power (Current Channel A)(32 bit)
#define BVA_24 0x211 //BVA, (R) Default: 0x000000, Signed,Instantaneous apparent power (Current Channel B)(24 bit)
#define BVA_32 0x311 //BVA, (R) Default: 0x000000, Signed,Instantaneous apparent power (Current Channel B)(32 bit)
#define AWATT_24 0x212 //AWATT, (R) Default: 0x000000, Signed,Instantaneous active power (Current Channel A)(24 bit)
#define AWATT_32 0x312 //AWATT, (R) Default: 0x000000, Signed,Instantaneous active power (Current Channel A)(32 bit)
#define BWATT_24 0x213 //BWATT, (R) Default: 0x000000, Signed,Instantaneous active power (Current Channel B)(24 bit)
#define BWATT_32 0x313 //BWATT, (R) Default: 0x000000, Signed,Instantaneous active power (Current Channel B)(32 bit)
#define AVAR_24 0x214 //AVAR, (R) Default: 0x000000, Signed,Instantaneous reactive power (Current Channel A)(24 bit)
#define AVAR_32 0x314 //AVAR, (R) Default: 0x000000, Signed,Instantaneous reactive power (Current Channel A)(32 bit)
#define BVAR_24 0x215 //BVAR, (R) Default: 0x000000, Signed,Instantaneous reactive power (Current Channel B)(24 bit)
#define BVAR_32 0x315 //BVAR, (R) Default: 0x000000, Signed,Instantaneous reactive power (Current Channel B)(32 bit)
#define IA_24 0x216 //IA, (R) Default: 0x000000, Signed, Instantaneous current (Current Channel A)(24 bit)
#define IA_32 0x316 //IA, (R) Default: 0x000000, Signed,Instantaneous current (Current Channel A)(32 bit)
#define IB_24 0x217 //IB, (R) Default: 0x000000, Signed,Instantaneous current (Current Channel B)(24 bit)
#define IB_32 0x317 //IB, (R) Default: 0x000000, Signed,Instantaneous current (Current Channel B)(32 bit)
#define V_24 0x218 //V, (R) Default: 0x000000, Signed,Instantaneous voltage (voltage channel)(24 bit)
#define V_32 0x318 //V, (R) Default: 0x000000, Signed,Instantaneous voltage (voltage channel)(32 bit)
#define IRMSA_24 0x21A //IRMSA, (R) Default: 0x000000, Unsigned,IRMS register (Current Channel A)(24 bit)
#define IRMSA_32 0x31A //IRMSA, (R) Default: 0x000000, Unsigned,IRMS register (Current Channel A)(32 bit)
#define IRMSB_24 0x21B //IRMSB, (R) Default: 0x000000, Unsigned,IRMS register (Current Channel B)(24 bit)
#define IRMSB_32 0x31B //IRMSB, (R) Default: 0x000000, Unsigned,IRMS register (Current Channel B)(32 bit)
#define VRMS_24 0x21C //VRMS, (R) Default: 0x000000, Unsigned, VRMS register (24 bit)
#define VRMS_32 0x31C //VRMS, (R) Default: 0x000000, Unsigned, VRMS register (32 bit)

#define AENERGYA_24 0x21E //AENERYGA, (R) Default: 0x000000, Signed,Active energy (Current Channel A) (24 bit)
#define AENERGYA_32 0x31E //AENERYGA, (R) Default: 0x000000, Signed,Active energy (Current Channel A)(32 bit)
#define AENERGYB_24 0x21F //AENERYGB, (R) Default: 0x000000, Signed,Active energy (Current Channel B)(24 bit)
#define AENERGYB_32 0x31F //AENERYGB, (R) Default: 0x000000, Signed,Active energy (Current Channel B)(32 bit)
#define RENERGYA_24 0x220 //RENERGYA, (R) Default: 0x000000, Signed,Reactive energy (Current Channel A) (24 bit)
#define RENERGYA_32 0x320 //RENERGYA, (R) Default: 0x000000, Signed,Reactive energy (Current Channel A)(32 bit)
#define RENERGYB_24 0x221 //RENERGYB, (R) Default: 0x000000, Signed,Reactive energy (Current Channel B) (24 bit)
#define RENERGYB_32 0x321 //RENERGYB, (R) Default: 0x000000, Signed,Reactive energy (Current Channel B)(32 bit)
#define APENERGYA_24 0x222 //APENERGYA, (R) Default: 0x000000, Signed,Apparent energy (Current Channel A) (24 bit)
#define APENERGYA_32 0x322 //APENERGYA, (R) Default: 0x000000, Signed,Apparent energy (Current Channel A)(32 bit)
#define APENERGYB_24 0x223 //APENERGYB, (R) Default: 0x000000, Signed,Apparent energy (Current Channel B)(24 bit)
#define APENERGYB_32 0x323 //APENERGYB, (R) Default: 0x000000, Signed,Apparent energy (Current Channel B)(32 bit)
#define OVLVL_24 0x224 //OVLVL, (R/W) Default: 0xFFFFFF, Unsigned, Overvoltage level(24 bit)
#define OVLVL_32 0x324 //OVLVL, (R/W) Default: 0xFFFFFF, Unsigned,Overvoltage level(32 bit)

#define OILVL_24 0x225 //OILVL, (R/W) Default: 0xFFFFFF,Unsigned, Overcurrent level (24 bit)
#define OILVL_32 0x325 //OILVL, (R/W) Default: 0xFFFFFF, Unsigned,Overcurrent level (32 bit)

#define VPEAK_24 0x226 //VPEAK, (R) Default: 0x000000, Unsigned, Voltage channel peak(24 bit)
#define VPEAK_32 0x326 //VPEAK, (R) Default: 0x000000, Unsigned,Voltage channel peak(32 bit)
#define RSTVPEAK_24 0x227 //RSTVPEAK, (R) Default: 0x000000, Unsigned,Read voltage peak with reset (24 bit)
#define RSTVPEAK_32 0x327 //RSTVPEAK, (R) Default: 0x000000, Unsigned,Read voltage peak with reset(32 bit)
#define IAPEAK_24 0x228 //IAPEAK, (R) Default: 0x000000, Unsigned,Current Channel A peak(24 bit)
#define IAPEAK_32 0x328 //IAPEAK, (R) Default: 0x000000, Unsigned,Current Channel A peak(32 bit)
#define RSTIAPEAK_24 0x229 //RSTIAPEAK, (R) Default: 0x000000, Unsigned, Read Current Channel A peak with reset(24 bit)
#define RSTIAPEAK_32 0x329 //RSTIAPEAK, (R) Default: 0x000000, Unsigned,Read Current Channel A peak with reset(32 bit)
#define IBPEAK_24 0x22A //IBPEAK, (R) Default: 0x000000, Unsigned, Current Channel B peak(24 bit)
#define IBPEAK_32 0x32A //IBPEAK, (R) Default: 0x000000, Unsigned,Current Channel B peak(32 bit)
#define RSTIBPEAK_24 0x22B //RSTIBPEAK, (R) Default: 0x000000, Unsigned, Read Current Channel B peak with reset(24 bit)
#define RSTIBPEAK_32 0x32B //RSTIBPEAK, (R) Default: 0x000000, Unsigned,Read Current Channel B peak with reset(32 bit)
#define IRQENA_24 0x22C //IRQENA, (R/W) Default: 0x100000, Unsigned,Interrupt enable (Current Channel A (24 bit)
#define IRQENA_32 0x32C //IRQENA, (R/W) Default: 0x100000, Unsigned,Interrupt enable (Current Channel A(32 bit)
#define IRQSTATA_24 0x22D //IRQSTATA, (R) Default: 0x000000, Unsigned, Interrupt status (Current Channel A(24 bit)
#define IRQSTATA_32 0x32D //IRQSTATA, (R) Default: 0x000000, Unsigned,Interrupt status (Current Channel A(32 bit)
#define RSTIRQSTATA_24 0x22E //RSTIRQSTATA, (R) Default: 0x000000, Unsigned, Reset interrupt status (Current Channel A) (24 bit)
#define RSTIRQSTATA_32 0x32E //RSTIRQSTATA, (R) Default: 0x000000, Unsigned,Reset interrupt status (Current Channel A)(32 bit)
#define IRQENB_24 0x22F //IRQENB, (R/W) Default: 0x000000, Unsigned,Interrupt enable (Current Channel B (24 bit)
#define IRQENB_32 0x32F //IRQENB, (R/W) Default: 0x000000, Unsigned,Interrupt enable (Current Channel B (32 bit)
#define IRQSTATB_24 0x230 //IRQSTATB, (R) Default: 0x000000, Unsigned, Interrupt status (Current Channel B(24 bit)
#define IRQSTATB_32 0x330 //IRQSTATB, (R) Default: 0x000000, Unsigned,Interrupt status (Current Channel B(32 bit)
#define RSTIRQSTATB_24 0x231 //RSTIRQSTATB, (R) Default: 0x000000, Unsigned,Reset interrupt status (Current Channel B) (24 bit)
#define RSTIRQSTATB_32 0x331 //RSTIRQSTATB, (R) Default: 0x000000, Unsigned, Reset interrupt status (Current Channel B)(32 bit)
#define CRC_24 0x000 //CRC, (R) Default: 0x000000, Unsigned, Checksum(24 bit)
#define CRC_32 0x37F //CRC, (R) Default: 0xFFFFFF, Unsigned,Checksum(32 bit)
#define AIGAIN_24 0x280 //AIGAIN, (R/W) Default: 0x400000, Unsigned, Current channel gain (Current Channel A)(24 bit)
#define AIGAIN_32 0x380 //AIGAIN, (R/W) Default: 0x400000, Unsigned,Current channel gain (Current Channel A)(32 bit)
#define AVGAIN_24 0x281 //AVGAIN, (R/W) Default: 0x400000, Unsigned, Voltage channel gain(24 bit)
#define AVGAIN_32 0x381 //AVGAIN, (R/W) Default: 0x400000, Unsigned,Voltage channel gain(32 bit)
#define AWGAIN_24 0x282 //AWGAIN, (R/W) Default: 0x400000, Unsigned,Active power gain (Current Channel A)(24 bit)
#define AWGAIN_32 0x382 //AWGAIN, (R/W) Default: 0x400000, Unsigned,Active power gain (Current Channel A)(32 bit)
#define AVARGAIN_24 0x283 //AVARGAIN, (R/W) Default: 0x400000, Unsigned, Reactive power gain (Current Channel A)(24 bit)
#define AVARGAIN_32 0x383 //AVARGAIN, (R/W) Default: 0x400000, Unsigned, Reactive power gain (Current Channel A)(32 bit)
#define AVAGAIN_24 0x284 //AVAGAIN, (R/W) Default: 0x400000, Unsigned, Apparent power gain (Current Channel A) (24 bit)
#define AVAGAIN_32 0x384 //AVAGAIN, (R/W) Default: 0x400000, Unsigned,Apparent power gain (Current Channel A)(32 bit)
#define Reserved_24 0x285 //Reserved, (R/W) Default: 0x000000, Signed,This register should not be modified (24 bit)
#define Reserved_32 0x385 //Reserved, (R/W) Default: 0x000000, Signed,This register should not be modified(32 bit)


#define AIRMSOS_24 0x286 //AIRMSOS, (R/W) Default: 0x000000, Signed,IRMS offset (Current Channel A) (24 bit)
#define AIRMSOS_32 0x386 //AIRMSOS, (R/W) Default: 0x000000, Signed,IRMS offset (Current Channel A)(32 bit)
#define Reserved1_24 0x287 //Reserved, (R/W) Default: 0x000000, Signed,This register should not be modified (24 bit)
#define Reserved1_32 0x387 //Reserved, (R/W) Default: 0x000000, Signed,This register should not be modified(32 bit)

#define VRMSOS_24 0x288 //VRMSOS, (R/W) Default: 0x000000, Signed, VRMS offset(24 bit)
#define VRMSOS_32 0x388 //VRMSOS, (R/W) Default: 0x000000, Signed,VRMS offset(32 bit)
#define AWATTOS_24 0x289 //AWATTOS, (R/W) Default: 0x000000, Signed, Active power offset correction (Current Channel A)(24 bit)
#define AWATTOS_32 0x389 //AWATTOS, (R/W) Default: 0x000000, Signed,Active power offset correction (Current Channel A)(32 bit)
#define AVAROS_24 0x28A //AVAROS, (R/W) Default: 0x000000, Signed, Reactive power offset correction (Current Channel A)(24 bit)
#define AVAROS_32 0x38A //AVAROS, (R/W) Default: 0x000000, Signed, Reactive power offset correction (Current Channel A)(32 bit)
#define AVAOS_24 0x28B //AVAOS, (R/W) Default: 0x000000, Signed, Apparent power offset correction (Current Channel A(24 bit)
#define AVAOS_32 0x38B //AVAOS, (R/W) Default: 0x000000, Signed,Apparent power offset correction (Current Channel A(32 bit)
#define BIGAIN_24 0x28C //BIGAIN, (R/W) Default: 0x400000, Unsigned,Current channel gain (Current Channel B) (24 bit)
#define BIGAIN_32 0x38C //BIGAIN, (R/W) Default: 0x400000, Unsigned,Current channel gain (Current Channel B)(32 bit)
#define BVGAIN_24 0x28D //BVGAIN, (R/W) Default: 0x400000, Unsigned, This register should not be modified(24 bit)
#define BVGAIN_32 0x38D //BVGAIN, (R/W) Default: 0x400000, Unsigned,This register should not be modified(32 bit)
#define BWGAIN_24 0x28E //BWGAIN, (R/W) Default: 0x400000, Unsigned, Active power gain (Current Channel B)(24 bit)
#define BWGAIN_32 0x38E //BWGAIN, (R/W) Default: 0x400000, Unsigned,Active power gain (Current Channel B)(32 bit)
#define BVARGAIN_24 0x28F //BVARGAIN, (R/W) Default: 0x400000, Unsigned, Reactive power gain (Current Channel B)(24 bit)
#define BVARGAIN_32 0x38F //BVARGAIN, (R/W) Default: 0x400000, Unsigned,Reactive power gain (Current Channel B)(32 bit)
#define BVAGAIN_24 0x290 //BVAGAIN, (R/W) Default: 0x400000, Unsigned, Apparent power gain (Current Channel B)(24 bit)
#define BVAGAIN_32 0x390 //BVAGAIN, (R/W) Default: 0x400000, Unsigned,Apparent power gain (Current Channel B)(32 bit)

#define Reserved2_24 0x291 //Reserved, (R/W) Default: 0x000000, Signed,This register should not be modified (24 bit)
#define Reserved2_32 0x391 //Reserved, (R/W) Default: 0x000000, Signed,This register should not be modified(32 bit)

#define BIRMSOS_24 0x292 //BIRMSOS, (R/W) Default: 0x000000, Unsigned, IRMS offset (Current Channel B)(24 bit)
#define BIRMSOS_32 0x392 //BIRMSOS, (R/W) Default: 0x000000, Unsigned,IRMS offset (Current Channel B)(32 bit)

#define Reserved3_24 0x293 //Reserved, (R/W) Default: 0x000000, Signed,This register should not be modified (24 bit)
#define Reserved3_32 0x393 //Reserved, (R/W) Default: 0x000000, Signed,This register should not be modified(32 bit)
#define Reserved4_24 0x294 //Reserved, (R/W) Default: 0x000000, Signed,This register should not be modified (24 bit)
#define Reserved4_32 0x394 //Reserved, (R/W) Default: 0x000000, Signed,This register should not be modified(32 bit)

#define BWATTOS_24 0x295 //BWATTOS, (R/W) Default: 0x000000, Unsigned, Active power offset correction (Current Channel B)(24 bit)
#define BWATTOS_32 0x395 //BWATTOS, (R/W) Default: 0x000000, Unsigned,Active power offset correction (Current Channel B)(32 bit)
#define BVAROS_24 0x296 //BVAROS, (R/W) Default: 0x000000, Unsigned,Reactive power offset correction (Current Channel B)(24 bit)
#define BVAROS_32 0x396 //BVAROS, (R/W) Default: 0x000000, Unsigned,Reactive power offset correction (Current Channel B)(32 bit)
#define BVAOS_24 0x297 //BVAOS, (R/W) Default: 0x000000, Unsigned, Apparent power offset correction (Current Channel B)(24 bit)
#define BVAOS_32 0x397 //BVAOS, (R/W) Default: 0x000000, Unsigned,Apparent power offset correction (Current Channel B)(32 bit)
#define LAST_RWDATA_24 0x2FF //LAST_RWDATA, (R) Default: 0x000000, Unsigned, Contains the data from the last successful 24-bit/32-bit register communication(24 bit)
#define LAST_RWDATA_32 0x3FF //LAST_RWDATA, (R) Default: 0x000000, Unsigned, Contains the data from the last successful 24-bit/32-bit register communication(32 bit)

  
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

extern int Lector_Dir_24(uint16_t , uint8_t* );

extern int Lector_Dir_16(uint16_t , uint8_t* );

extern int Lector_Dir_8(uint16_t , uint8_t* );

int get_wellsended(void);

void set_wellsended(int);





#ifdef __cplusplus
}
#endif

#endif
