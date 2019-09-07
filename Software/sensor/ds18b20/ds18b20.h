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
#ifndef __DS18B20_H__
#define __DS18B20_H__    

#include "stdio.h"
    
#define Tinact  20          //in us. (Inactive before next signal)
#define TMtr0   50          //Data bit 0 read,write 15-60us
#define TMtr1   10          //Data bit 1 read,write 1-15us
#define RST_MAX 480         //in us.(минимум 480)

uint8_t OneWire_R_Bit();                 //******   ReadBit ************

void OneWire_W_Bit(uint8_t payload);     //******   WriteBit ***********

uint8_t OneWire_BusReset();              //******** Bus Reset **********

void OneWire_Write8(uint8_t payload);    //******* Write a byte *********

uint8_t OneWire_Read8();              // чтение байта (не проверено!!!)

bool DS18B20_Init(void);
void DS18B20_Rst(void);
bool DS18B20_Presence(void);
uint8_t DS18B20_ReadBit(void);
uint8_t DS18B20_ReadByte(void);
void DS18B20_WriteByte(uint8_t dat);
void DS18B20_SkipRom( void );
float DS18B20_GetTemp_SkipRom ( void );

#endif // __DS18B20_H__
/* [] END OF FILE */
