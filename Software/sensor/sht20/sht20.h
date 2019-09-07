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
#ifndef __SHT20_H__
#define __SHT20_H__    

#include "stdio.h"

/* I2C Slave address to communicate with */
#define I2C_SLAVE_ADDR      (0x40u)

bool SHT20_READ_USER_REGISTER(void);
float SHT20_GET_TEMP(uint8_t cmd);
float SHT20_GET_HUM(uint8_t cmd);

#endif // __SHT20_H__
/* [] END OF FILE */
