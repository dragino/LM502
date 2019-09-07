/*
 * Copyright (C) 2015-2017 Alibaba Group Holding Limited
 */

#include "hw.h"
#include "low_power.h"
#include "linkwan_ica_at.h"
#include "linkwan.h"
#include "timeServer.h"
//#include "version.h"
#include "radio.h"
//#include "sx1276Regs-Fsk.h"
//#include "sx1276Regs-LoRa.h"
#include "board_test.h"
#include "delay.h"
#include "board.h"

#include "LoRaMac.h"
#include "Region.h"
#include "commissioning.h"

#include <k_api.h>
#include "lwan_config.h"

#if defined(LORA_SENSOR_ENABLE)
#include "bsp.h"
#endif

#define APP_TX_DUTYCYCLE 30000
#define LORAWAN_ADR_ON 1
#define LORAWAN_APP_PORT 100
#define JOINREQ_NBTRIALS 3

#if defined(LORA_SENSOR_ENABLE)
extern uint8_t pin_wakeup_status;
#endif

static void LoraTxData(lora_AppData_t *AppData);
static void LoraRxData(lora_AppData_t *AppData);
uint8_t BoardGetBatteryLevel()
{
    return 100;
}
void BoardGetUniqueId(uint8_t *id)
{
}
uint32_t BoardGetRandomSeed()
{
    return 0;
}
static LoRaMainCallback_t LoRaMainCallbacks = {
    BoardGetBatteryLevel,
    BoardGetUniqueId,
    BoardGetRandomSeed,
    LoraTxData,
    LoraRxData
};

static void lora_task_entry(void *arg)
{
    BoardInitMcu();
    #ifdef LORA_SENSOR_ENABLE
    BSP_sensor_Init();
    #endif
    lora_init(&LoRaMainCallbacks);  
    lora_fsm( );
}

int application_start( void )
{
    lora_task_entry(NULL);
    return 0;
}

static void LoraTxData( lora_AppData_t *AppData)
{
    uint8_t port;
    lwan_mac_config_get(MAC_CONFIG_APP_PORT, &port);
    
    #if defined(LORA_SENSOR_ENABLE)
    sensor_t sensor_data;
    
    BSP_sensor_Read(&sensor_data);

    uint16_t i=0;
    AppData->Buff[i++]=(int)(sensor_data.adc1)>>8;
    AppData->Buff[i++]=(int)(sensor_data.adc1);
    
    AppData->Buff[i++]=(int)(sensor_data.temp_ds18b20*100)>>8;
    AppData->Buff[i++]=(int)(sensor_data.temp_ds18b20*100);
    
    AppData->Buff[i++]=(int)(sensor_data.temp_sht20*100)>>8;
    AppData->Buff[i++]=(int)(sensor_data.temp_sht20*100);
    AppData->Buff[i++]=(int)(sensor_data.hum_sht20*10)>>8;
    AppData->Buff[i++]=(int)(sensor_data.hum_sht20*10);
    
    if(pin_wakeup_status==1)
    {
        pin_wakeup_status=0;
        AppData->Buff[i++]=(int)(sensor_data.digital_input_status<<1)|0x01;//Digital Input and pin_wakeup status
    }
    else
    AppData->Buff[i++]=(int)(sensor_data.digital_input_status<<1)|0x00;
    
    AppData->Port = port;
    AppData->BuffSize=i;
    #else
    AppData->BuffSize = sprintf( (char *) AppData->Buff, "12345678");
    PRINTF_RAW("tx: %s\r\n", AppData->Buff );
    AppData->Port = LORAWAN_APP_PORT;
    #endif
}

static void LoraRxData( lora_AppData_t *AppData )
{
    AppData->Buff[AppData->BuffSize] = '\0';
    PRINTF_RAW( "rx: port = %d, len = %d\r\n", AppData->Port, AppData->BuffSize);
    int i;
    for (i = 0; i < AppData->BuffSize; i++) {
        PRINTF_RAW("0x%x ", AppData->Buff[i] );
    }
    PRINTF_RAW("\r\n");
    
    #if defined(LORA_SENSOR_ENABLE)
    switch(AppData->Buff[0] & 0xff)
    {   
      case 1:
	  {
		if( AppData->BuffSize == 3 )
		{
			if(AppData->Buff[1]==0x00&&AppData->Buff[2]==0x00)
			{
				DBG_LINKWAN("TDC setting error\n\r");
			}
			else
			{
                uint8_t mode=1;
		        uint16_t ServerSetTDC=( AppData->Buff[1]<<8 | AppData->Buff[2] );//S
				if(ServerSetTDC<5)
				{
				  DBG_LINKWAN("TDC setting needs to be high than 4s\n\r");
				}
				else
				{
                  lwan_mac_config_set(MAC_CONFIG_REPORT_MODE, (void *)&mode);
                  lwan_mac_config_set(MAC_CONFIG_REPORT_INTERVAL, &ServerSetTDC);
                  SFlash_store_lora_config();
				}
			}
		}
		break;
	  }
                
      case 2:
	  {
        
		if( AppData->BuffSize == 2 )
		{
		  if(AppData->Buff[1]==0x01)
		  {
		    led0_Write(1);
		  }
          else if(AppData->Buff[1]==0x00)
          {
            led0_Write(0);
          }
	    }
		break;
	  }
      
      case 4:
	  {
		if( AppData->BuffSize == 2 )
		{
		  if(AppData->Buff[1]==0xFF)
		  {
		    NVIC_SystemReset();
		  }
	    }
		break;
	  }
      default:break;
    }
    #endif
}
