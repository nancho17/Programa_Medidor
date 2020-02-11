#include "ADE7953.h"


static bool wellsended=0;

int get_wellsended(){
    return wellsended;
}

void set_wellsended(int well){
    wellsended=well;
}


bool Escritura_ADE795 ( uint8_t alfa){    
    /*UCAxTXIFG is automatically reset if a character is written to UCAxTXBUF.*/
    if (IFG2&UCA0TXIFG){ 
        UCA0TXBUF =alfa;	
        return 1;
    }
    return 0;
}

/* Funciones Escritura */

int Escritor_Dir_8(uint16_t direccionn, uint8_t dato, uint8_t* a)
{
   switch(*(a)){
        case 0x00: if(Escritura_ADE795(0xCA))                          {*(a)=0x01;} else{*(a)=0x07; wellsended=0;} break; //Lectura 35  Escritura CA
        case 0x02: if(Escritura_ADE795(Direccion_Byte(direccionn,1) ) ){*(a)=0x03;} else{*(a)=0x07; wellsended=0;} break; //
        case 0x04: if(Escritura_ADE795(Direccion_Byte(direccionn,0) ) ){*(a)=0x05;} else{*(a)=0x07; wellsended=0;} break; //
        case 0x06: if(       Escritura_ADE795(dato) )				{wellsended=1;}else{wellsended=0;}; *(a)=0x07; break; //
        		
        case 0x0D: *(a)=0x00; return 1; break;
                                
        default  : *(a)=*(a)+1; break;
    }
return 0;
}

int Escritor_Dir_16(uint16_t direccionn, uint16_t dato, uint8_t* a)
{
   switch(*(a)){
        case 0x00: if(Escritura_ADE795(0xCA))						   {*(a)=0x01;} else{*(a)=0x09; wellsended=0;} break; //Lectura 35  Escritura CA
        case 0x02: if(Escritura_ADE795(Direccion_Byte(direccionn,1)))  {*(a)=0x03;} else{*(a)=0x09; wellsended=0;} break; //
        case 0x04: if(Escritura_ADE795(Direccion_Byte(direccionn,0)))  {*(a)=0x05;} else{*(a)=0x09; wellsended=0;} break; //
        case 0x06: if(Escritura_ADE795(Direccion_Byte(dato,0)) )	   {*(a)=0x07;} else{*(a)=0x09; wellsended=0;} break; //
        case 0x08: if(Escritura_ADE795(Direccion_Byte(dato,1)) )	 {wellsended=1;}else{wellsended=0;};*(a)=0x09; break; //
		
        case 0x0F: *(a)=0x00; return 1; break;
                                
        default  : *(a)=*(a)+1; break;
    }
return 0;
}

int Escritor_Dir_24(uint16_t direccionn, uint32_t dato, uint8_t* a)
{
   switch(*(a)){
        case 0x00: if(Escritura_ADE795(0xCA))                          {*(a)=0x01;} else{*(a)=0x0B; wellsended=0;} break; //Lectura 35  Escritura CA
        case 0x02: if(Escritura_ADE795(Direccion_Byte(direccionn,1) ) ){*(a)=0x03;} else{*(a)=0x0B; wellsended=0;} break; //
        case 0x04: if(Escritura_ADE795(Direccion_Byte(direccionn,0) ) ){*(a)=0x05;} else{*(a)=0x0B; wellsended=0;} break; //
        case 0x06: if(Escritura_ADE795(Direccion_Byte(dato,0) ) )      {*(a)=0x07;} else{*(a)=0x0B; wellsended=0;} break; //
        case 0x08: if(Escritura_ADE795(Direccion_Byte(dato,1) ) )      {*(a)=0x09;} else{*(a)=0x0B; wellsended=0;} break; //
		case 0x0A: if(Escritura_ADE795(Direccion_Byte(dato,2) ) )   {wellsended=1;}else{wellsended=0;}; *(a)=0x0B; break; //
		
        case 0x11: *(a)=0x00; return 1; break;
                                
        default  : *(a)=*(a)+1; break;
    }
return 0;
}


uint8_t ADE_Interruptor_RX(int auxiliar){
    
    if (wellsended){
        //wellsended=0;
        return UCA0RXBUF;
    }
    else{UCA0RXBUF;
    return 255;
    }
}


uint8_t Direccion_Byte (int dir, uint8_t posbyte )
{
 int a = (dir>>(8*posbyte)) & 0xFF;
  return a;
}

/* Funciones Lectura */

// Frame delay is 2.08 ms Max delay 4 ms
// 1,0070800 + 1,0070800 2,014
// 1,0070800 + 2,014     3,021
int Lector_Dir_24(uint16_t direccionn, uint8_t* a)//24
{
   switch(*(a)){
        case 0x00: if(Escritura_ADE795(0x35)){*(a)=0x01;}else{*(a)=0x05;   wellsended=0;} break; //Lectura 35  Escritura CA
        case 0x02: if(Escritura_ADE795(Direccion_Byte(direccionn,1) ) ){*(a)=0x03;}else{*(a)=0x05;   wellsended=0;} break; //
        case 0x04: if(Escritura_ADE795(Direccion_Byte(direccionn,0) ) ){*(a)=0x05;wellsended=1;}else{wellsended=0;} break; //

        case 0x13: *(a)=0x00; return 1; break;
                                
        default  : *(a)=*(a)+1; break;
    }
return 0;
}

int Lector_Dir_8(uint16_t direccionn, uint8_t* a)
{
   switch(*(a)){
        case 0x00: if(Escritura_ADE795(0x35)){*(a)=0x01;}else{*(a)=0x05;   wellsended=0;} break; //Lectura 35  Escritura CA
        case 0x02: if(Escritura_ADE795(Direccion_Byte(direccionn,1) ) ){*(a)=0x03;}else{*(a)=0x05;   wellsended=0;} break; //
        case 0x04: if(Escritura_ADE795(Direccion_Byte(direccionn,0) ) ){*(a)=0x05;wellsended=1;}else{wellsended=0;} break; //

        case 0x0D: *(a)=0x00; 
		if(wellsended){return 1;}else{return 0;}; 
		break;
                                
        default  : *(a)=*(a)+1;  break;
    }
return 0;
}

int Lector_Dir_16(uint16_t direccionn, uint8_t* a)
{
   switch(*(a)){
        case 0x00: if(Escritura_ADE795(0x35)){*(a)=0x01; }else{*(a)=0x05;   wellsended=0;} break; //Lectura 35  Escritura CA
        case 0x02: if(Escritura_ADE795(Direccion_Byte(direccionn,1) ) ){*(a)=0x03;}else{*(a)=0x05;   wellsended=0;} break; //
        case 0x04: if(Escritura_ADE795(Direccion_Byte(direccionn,0) ) ){*(a)=0x05;wellsended=1;}else{wellsended=0;} break; //
		
        case 0x10: *(a)=0x00; return 1; break;
                                
        default  : *(a)=*(a)+1; break;
    }
return 0;
}

//uint32_t ADE7953::i2cAlgorithm32_read(byte MSB, byte LSB)
/*Armar Funcion lectora con MSB y LSB*/
//

//byte ADE7953::functionBitVal(int addr, uint8_t byteVal){
//Returns as integer an address of a specified byte - basically a byte controlled shift register with "byteVal" controlling the byte that is read and returned
//  int x = ((addr >> (8*byteVal)) & 0xff);
//return x;



//EJEMPLOS


/// return i2cAlgorithm8_read(functionBitVal(Version_8,1), functionBitVal(Version_8,0))
//	value=i2cAlgorithm16_read((functionBitVal(PFA_16,1)),(functionBitVal(PFA_16,0)));
//value=i2cAlgorithm32_read((functionBitVal(AP_NOLOAD_32,1)),(functionBitVal(AP_NOLOAD_32,0)));