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

void ADE_Lectura_1ms_TIMING( uint8_t* a)
{
    switch(*(a)){
        case 0x00: if(Escritura_ADE795(0x35)){*(a)=0x01;}else{*(a)=0x0B;   wellsended=0;} break; //Lectura 35 Escritura CA
        case 0x05: if(Escritura_ADE795(0x00)){*(a)=0x06;}else{*(a)=0x0B;   wellsended=0;} break; //
        case 0x0A: if(Escritura_ADE795(0xFF)){*(a)=0x0B;wellsended=1;}else{wellsended=0;} break; //
        //0x8004 2 bytes
        //case 0x1C: *(a)=0x00;   break;
        case 0x17: *(a)=0x00; break;
                                
        default  : *(a)=*(a)+1; break;
    }   
}

void ADE_Lectura_0ms5_TIMING( uint8_t* a)
{//ZXTOUT x100
    switch(*(a)){
        case 0x00: if(Escritura_ADE795(0x35)){*(a)=0x01;}else{*(a)=0x0B;   wellsended=0;} break; //Lectura 35 Escritura CA
        case 0x05: if(Escritura_ADE795(0x00)){*(a)=0x06;}else{*(a)=0x0B;   wellsended=0;} break; //
        case 0x0A: if(Escritura_ADE795(0xFF)){*(a)=0x0B;wellsended=1;}else{wellsended=0;} break; //
        //0x8004 2 bytes
        //case 0x1C: *(a)=0x00;   break;
        case 0x17: *(a)=0x00; break;
                                
        default  : *(a)=*(a)+1; break;
    }   
}

uint8_t ADE_Interruptor_RX(int auxiliar){
    
    if (wellsended){
        if (auxiliar>3){
        auxiliar=0;
        wellsended=0;}else{
        auxiliar++;}
        
        return UCA0RXBUF;
        
    
    }
    else{UCA0RXBUF;
    auxiliar=0;
    return 0;
    }
}