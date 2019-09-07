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

#if defined(LORA_SENSOR_ENABLE)
#include "hw.h"
#include "bsp.h"
#include "ds18b20.h"
#include "sht20.h"
bool ds18b20_init_error=0,sht20_init_error=0;

void BSP_sensor_Read( sensor_t *sensor_data)
{
  #if defined(LORA_SENSOR_ENABLE) 
  
    uint16_t ADC_channel_1_mv=0;
    uint16_t ADC_channel_1_code=0;
    
      ADC_SAR_Seq_StartConvert();
      CyDelay(10);
          /* When conversion of sequencing channels has completed */
      if(0u != ADC_SAR_Seq_IsEndConversion(ADC_SAR_Seq_RETURN_STATUS))
      {
        ADC_channel_1_code=ADC_SAR_Seq_GetResult16(0);
        ADC_channel_1_mv = ADC_SAR_Seq_CountsTo_mVolts(0, ADC_channel_1_code);
      }
    
      sensor_data->adc1=ADC_channel_1_mv;
    
      sensor_data->temp_ds18b20=DS18B20_GetTemp_SkipRom();

      if(SHT20_READ_USER_REGISTER()==1)
      {
        sensor_data->temp_sht20=SHT20_GET_TEMP(0xf3);
        sensor_data->hum_sht20=SHT20_GET_HUM(0xf5);
      }
      else
      {
        sensor_data->temp_sht20=327.67;
        sensor_data->hum_sht20=3276.7;
      }

      sensor_data->digital_input_status=digital_input_pin_Read();
    
  #endif
}

void  BSP_sensor_Init( void  )
{
  ADC_SAR_Seq_Start();
  I2CM_Start();
}
#endif
/* [] END OF FILE */
