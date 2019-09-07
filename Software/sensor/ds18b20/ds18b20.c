/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/
#ifdef LORA_SENSOR_ENABLE
#include "hw.h"
#include "ds18b20.h"

/************************   ReadBit *******************************************/

uint8_t OneWire_R_Bit() //***  Read a single bit off the 1-WIRE bus. ***
    {
        CyDelayUs(Tinact);  // Inactive before next signal........#define Tinact 20   //in us. (Inactive before next signal)
        ds18b20_pin_Write(0);       // Drive it low.                                             варианты: http://www.cypress.com/?rID=38854&cache=0
        CyDelayUs(TMtr1);   // If its a 1 to write. ....   #define TMtr1 10    //Data bit 1 read,write 1-15us
        ds18b20_pin_Write(1);       // Back High.
	    CyDelayUs(5);       
	return(ds18b20_pin_Read());     
    //return(1);
	}
    
/**************************   WriteBit   **************************************/

void OneWire_W_Bit(uint8_t payload) // Write a single bit to the 1-WIRE bus. 
    {
	    CyDelayUs(Tinact);  //Inactive before next signal.
	    ds18b20_pin_Write(0);       //Drive it low.
	        if(payload==0)		CyDelayUs(TMtr0); //If its a 0 to write.
	        else     		    CyDelayUs(TMtr1); //If its a 1 to write.
	    ds18b20_pin_Write(1);       //Back High.
    }

/*************************   Bus Reset   **************************************/
   
uint8_t OneWire_BusReset() //Pulldown the required "Bus Reset". *******
    {
uint8_t BusPin=0 ;       
        ds18b20_pin_Write(0);       // Drive it low.
	    CyDelayUs(RST_MAX); 
	    ds18b20_pin_Write(1);       // Back High.
        CyDelayUs(5);       
        if (ds18b20_pin_Read()==0x00)  	return(0xFF);   
        CyDelayUs(55);      
        BusPin = ds18b20_pin_Read();   
        CyDelayUs(200);     
return(BusPin);        
    }
 
/**************************   Write a byte   **********************************/
void OneWire_Write8(uint8_t payload) //Write a byte to the slave.
    {
	    uint8_t BitPayload,  shiftcount;
	    for (shiftcount=0; shiftcount<=7;shiftcount++) 
        {
            BitPayload = (payload >> shiftcount) & 0x01; 
            OneWire_W_Bit(BitPayload);
        }
    }  
    
/****************************   Read 8 bits  **********************************/

 uint8_t OneWire_Read8() //Read 8 bits "clocked" out by the slave. 
{
	uint8_t IncomingByte=0, shiftcount=0 ;
	for (shiftcount=0; shiftcount<=7; shiftcount++) 
    {
       IncomingByte |= (OneWire_R_Bit())<<shiftcount;
    }
    return(IncomingByte);
}

bool DS18B20_Init(void)
{
  ds18b20_pin_Write(1);
        
  DS18B20_Rst();
  
  return DS18B20_Presence();
}

void DS18B20_Rst(void)
{
    ds18b20_pin_Write(0);

    CyDelayUs(750);
    
    ds18b20_pin_Write(1);
           
    CyDelayUs(15);
}

bool DS18B20_Presence(void)
{
    uint8_t pulse_time = 0;

    while( ds18b20_pin_Read() && pulse_time<100 )
    {
            pulse_time++;
            CyDelayUs(1);
    }        

    if( pulse_time >=100 )
            return 1;
    else
            pulse_time = 0;
    
			
    while( !ds18b20_pin_Read() && pulse_time<240 )
    {
            pulse_time++;
            CyDelayUs(1);
    }        
    if( pulse_time >=240 )
            return 1;
    else
            return 0;
}

void DS18B20_SkipRom( void )
{
    DS18B20_Rst();                   
    DS18B20_Presence();                 
    OneWire_Write8(0XCC);       
}

float DS18B20_GetTemp_SkipRom ( void )
{
    uint8_t tpmsb, tplsb;
    short s_tem;
    float f_tem;
    
    if(DS18B20_Init()==0)
    {
        DS18B20_SkipRom();
        OneWire_Write8(0X44);                                
        
        CyDelay(750);
	
        DS18B20_SkipRom ();
        OneWire_Write8(0XBE);                                
        
        tplsb = OneWire_Read8();                 
        tpmsb = OneWire_Read8(); 
        
        
        s_tem = tpmsb<<8;
        s_tem = s_tem | tplsb;
        
        if( s_tem < 0 )                
          f_tem = (~s_tem+1) * -0.0625;        
        else
          f_tem = s_tem * 0.0625;
        
    }
    else
    {
	  f_tem=327.67;
    }	
        
    return f_tem;
}
#endif
/* [] END OF FILE */

