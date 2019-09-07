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
#include "sht20.h"

bool SHT20_READ_USER_REGISTER(void)
{
     uint8  buffer_cmd[1],sht20_is_connented=0;
    
    buffer_cmd[0]=0xe7;
    
    (void) I2CM_I2CMasterClearStatus();
    
    /* Start I2C write and check status*/
    if(I2CM_I2C_MSTR_NO_ERROR == I2CM_I2CMasterWriteBuf(I2C_SLAVE_ADDR,
                                    buffer_cmd, 1,
                                    I2CM_I2C_MODE_COMPLETE_XFER))
    {
        /*If I2C write started without errors, 
        / wait until I2C Master completes write transfer 
        */
        while (0u == (I2CM_I2CMasterStatus() & I2CM_I2C_MSTAT_WR_CMPLT))
        {
            /* Wait */
        }
        
        /* Display transfer status */
        if (0u == (I2CM_I2CMasterStatus() & I2CM_I2C_MSTAT_ERR_XFER))
        {
            /* Check if all bytes was written */
            if (I2CM_I2CMasterGetWriteBufSize() == 1)
            {
               sht20_is_connented=1;
            }
            else
            sht20_is_connented=0;
        }
    }

    (void) I2CM_I2CMasterClearStatus();
    
    return sht20_is_connented;
}

float SHT20_GET_TEMP(uint8_t cmd)
{
    uint8_t  buffer_cmd[1],buffer_data[2];
    uint16_t sht20_temp_code;
    float tem;
    
    /* Initialize buffer with packet */
   
    buffer_cmd[0] = cmd;

    (void) I2CM_I2CMasterClearStatus();
    
    /* Start I2C write and check status*/
    if(I2CM_I2C_MSTR_NO_ERROR == I2CM_I2CMasterWriteBuf(I2C_SLAVE_ADDR,
                                    buffer_cmd, 1,
                                    I2CM_I2C_MODE_COMPLETE_XFER))
    {
        /*If I2C write started without errors, 
        / wait until I2C Master completes write transfer 
        */
        while (0u == (I2CM_I2CMasterStatus() & I2CM_I2C_MSTAT_WR_CMPLT))
        {
            /* Wait */
        }
        
        /* Display transfer status */
        if (0u == (I2CM_I2CMasterStatus() & I2CM_I2C_MSTAT_ERR_XFER))
        {
            /* Check if all bytes was written */
            if (I2CM_I2CMasterGetWriteBufSize() == 1)
            {
                //PRINTF_RAW("sht20_write ok\r\n");
            }
        }
    }

    (void) I2CM_I2CMasterClearStatus();
    CyDelay(85);
    
    if(I2CM_I2C_MSTR_NO_ERROR ==  I2CM_I2CMasterReadBuf(I2C_SLAVE_ADDR,
                                    buffer_data, 2,
                                    I2CM_I2C_MODE_COMPLETE_XFER))
    {
        /* If I2C read started without errors, 
        / wait until master complete read transfer */
        while (0u == (I2CM_I2CMasterStatus() & I2CM_I2C_MSTAT_RD_CMPLT))
        {
            /* Wait */
        }
        
        /* Display transfer status */
        if (0u == (I2CM_I2C_MSTAT_ERR_XFER & I2CM_I2CMasterStatus()))
        {
            /* Check packet structure */
            if (I2CM_I2CMasterGetReadBufSize() == 2)
            {           
                //PRINTF_RAW("sht20_read ok\r\n");   
            }
        }
    }
    
       sht20_temp_code=(buffer_data[0]<<8)|buffer_data[1];
	   sht20_temp_code &=~0x0003;
	   tem=sht20_temp_code*175.72/65536-46.85;
                
    (void) I2CM_I2CMasterClearStatus();
    
    return (tem);
}

float SHT20_GET_HUM(uint8_t cmd)
{
    uint8_t  buffer_cmd[1],buffer_data[2];
    uint16_t sht20_hum_code;
    float hum;
    
    /* Initialize buffer with packet */
   
    buffer_cmd[0] = cmd;

    (void) I2CM_I2CMasterClearStatus();
    
    /* Start I2C write and check status*/
    if(I2CM_I2C_MSTR_NO_ERROR == I2CM_I2CMasterWriteBuf(I2C_SLAVE_ADDR,
                                    buffer_cmd, 1,
                                    I2CM_I2C_MODE_COMPLETE_XFER))
    {
        /*If I2C write started without errors, 
        / wait until I2C Master completes write transfer 
        */
        while (0u == (I2CM_I2CMasterStatus() & I2CM_I2C_MSTAT_WR_CMPLT))
        {
            /* Wait */
        }
        
        /* Display transfer status */
        if (0u == (I2CM_I2CMasterStatus() & I2CM_I2C_MSTAT_ERR_XFER))
        {
            /* Check if all bytes was written */
            if (I2CM_I2CMasterGetWriteBufSize() == 1)
            {
              //PRINTF_RAW("sht20_write ok\r\n");  
            }
        }
    }

    (void) I2CM_I2CMasterClearStatus();
    CyDelay(30);
    
    if(I2CM_I2C_MSTR_NO_ERROR ==  I2CM_I2CMasterReadBuf(I2C_SLAVE_ADDR,
                                    buffer_data, 2,
                                    I2CM_I2C_MODE_COMPLETE_XFER))
    {
        /* If I2C read started without errors, 
        / wait until master complete read transfer */
        while (0u == (I2CM_I2CMasterStatus() & I2CM_I2C_MSTAT_RD_CMPLT))
        {
            /* Wait */
        }
        
        /* Display transfer status */
        if (0u == (I2CM_I2C_MSTAT_ERR_XFER & I2CM_I2CMasterStatus()))
        {
            /* Check packet structure */
            if (I2CM_I2CMasterGetReadBufSize() == 2)
            {           
               //PRINTF_RAW("sht20_read ok\r\n");    
            }
        }
    }
    
       sht20_hum_code=(buffer_data[0]<<8)|buffer_data[1];
	   sht20_hum_code &=~0x0003;
	   hum=sht20_hum_code*125.0/65536-6;
                
    (void) I2CM_I2CMasterClearStatus();
    
    return (hum);
}
#endif
/* [] END OF FILE */

