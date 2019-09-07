/*
 * Copyright (C) 2015-2017 Alibaba Group Holding Limited
 */
#include "commissioning.h"
#include "utilities.h"
#include "LoRaMacCrypto.h"
#include "LoRaMac.h"
#include "LoRaMacTest.h"
#include "LoRaMacClassB.h"
#include "timeServer.h"
#include "hw.h"
#include "low_power.h"
#include "radio.h"
#include "linkwan_ica_at.h"
#include <uart_port.h>
#include "hal/soc/flash.h"
#include "hal/soc/uart.h"
#include <aos/aos.h>
#ifdef AOS_KV
#include <assert.h>
#include "kvmgr.h"
#endif
#include "lwan_config.h"  
#include "linkwan.h"

#define MAX_BEACON_RETRY_TIMES 2
#define LORA_KEYS_MAGIC_NUM 0xABABBABA 

static uint8_t tx_buf[LORAWAN_APP_DATA_BUFF_SIZE];
static lora_AppData_t tx_data = {tx_buf, 1, 10};
static uint8_t rx_buf[LORAWAN_APP_DATA_BUFF_SIZE];
static lora_AppData_t rx_data = {rx_buf, 0, 0};

static LoRaMacPrimitives_t LoRaMacPrimitives;
static LoRaMacCallback_t LoRaMacCallbacks;
static LoRaMainCallback_t *app_callbacks;

#if defined(LORA_SENSOR_ENABLE)
extern uint8_t pin_wakeup_status;
#endif

static volatile bool next_tx = true;
static volatile bool rejoin_flag = true;

static uint8_t gGatewayID[3] ={0};
static uint8_t g_beacon_retry_times = 0;

static uint32_t g_ack_index = 0;
static uint8_t g_data_send_nbtrials = 0;
static int8_t g_data_send_msg_type = -1;
#ifdef CONFIG_LINKWAN
static uint8_t g_freqband_num = 0;
#endif    

static TimerEvent_t TxNextPacketTimer;
volatile DeviceState_t g_lwan_device_state = DEVICE_STATE_INIT;
static DeviceStatus_t g_lwan_device_status = DEVICE_STATUS_IDLE;

bool g_lora_debug = false;
static LWanDevConfig_t *g_lwan_dev_config_p = NULL;
static LWanMacConfig_t *g_lwan_mac_config_p = NULL;
static LWanDevKeys_t *g_lwan_dev_keys_p = NULL;
static LWanProdctConfig_t *g_lwan_prodct_config_p = NULL;
static void start_dutycycle_timer(void);

#ifdef CONFIG_PROJECT_CAINIAO
static void notify_host()
{
    pin_wakeup_SetDriveMode(pin_wakeup_DM_STRONG);
    pin_wakeup_Write(1);
    CyDelay(5);
    pin_wakeup_Write(0);
}
#else
static void notify_host()
{
}  
#endif    

static bool send_frame(void)
{
    McpsReq_t mcpsReq;
    LoRaMacTxInfo_t txInfo;
    uint8_t send_msg_type;

    if (LoRaMacQueryTxPossible(tx_data.BuffSize, &txInfo) != LORAMAC_STATUS_OK) {
        return true;
    }

    if(g_lwan_mac_config_p->modes.linkcheck_mode == 2) {
        MlmeReq_t mlmeReq;
        mlmeReq.Type = MLME_LINK_CHECK;
        LoRaMacMlmeRequest(&mlmeReq);
    }
    
    send_msg_type = g_data_send_msg_type>=0?g_data_send_msg_type:g_lwan_mac_config_p->modes.confirmed_msg;
    if (send_msg_type == LORAWAN_UNCONFIRMED_MSG) {
        MibRequestConfirm_t mibReq;
        mibReq.Type = MIB_CHANNELS_NB_REP;
        mibReq.Param.ChannelNbRep = g_data_send_nbtrials?g_data_send_nbtrials:
                                                    g_lwan_mac_config_p->nbtrials.unconf + 1;
        LoRaMacMibSetRequestConfirm(&mibReq);
    
        mcpsReq.Type = MCPS_UNCONFIRMED;
        mcpsReq.Req.Unconfirmed.fPort = g_lwan_mac_config_p->port;
        mcpsReq.Req.Unconfirmed.fBuffer = tx_data.Buff;
        mcpsReq.Req.Unconfirmed.fBufferSize = tx_data.BuffSize;
        mcpsReq.Req.Unconfirmed.Datarate = g_lwan_mac_config_p->datarate;
    } else {
        mcpsReq.Type = MCPS_CONFIRMED;
        mcpsReq.Req.Confirmed.fPort = g_lwan_mac_config_p->port;
        mcpsReq.Req.Confirmed.fBuffer = tx_data.Buff;
        mcpsReq.Req.Confirmed.fBufferSize = tx_data.BuffSize;
        mcpsReq.Req.Confirmed.NbTrials = g_data_send_nbtrials?g_data_send_nbtrials:
                                                    g_lwan_mac_config_p->nbtrials.conf+1;
        mcpsReq.Req.Confirmed.Datarate = g_lwan_mac_config_p->datarate; 
    }

    g_data_send_nbtrials = 0;
    g_data_send_msg_type = -1;
    
    if (LoRaMacMcpsRequest(&mcpsReq) == LORAMAC_STATUS_OK) {
        return false;
    }

    return true;
}

static void prepare_tx_frame(void)
{
    if (g_lwan_mac_config_p->modes.report_mode == TX_ON_TIMER) {
        app_callbacks->LoraTxData(&tx_data);
    }
}

#ifdef CONFIG_LINKWAN
static uint8_t get_freqband_num(void)
{
    uint8_t num = 0;
    uint16_t mask = g_lwan_dev_config_p->freqband_mask;

    for (uint8_t i = 0; i < 16; i++) {
        if ((mask & (1 << i)) && i != 1) {
            num++;
        }
    }
    return num;
}
static uint8_t get_next_freqband(void)
{
    uint8_t freqband[16];
    uint8_t freqnum = 0;
    uint16_t mask = g_lwan_dev_config_p->freqband_mask;

    freqband[freqnum++] = 1; //1A2
    for (uint8_t i = 0; i < 16; i++) {
        if ((mask & (1 << i)) && i != 1) {
            freqband[freqnum++] = i;
        }
    }
    
    return freqband[randr(0,freqnum-1)];
}
#endif

static void reset_join_state(void)
{
    g_lwan_device_state = DEVICE_STATE_JOIN;
}
static void on_tx_next_packet_timer_event(void)
{
    MibRequestConfirm_t mib_req;
    LoRaMacStatus_t status;

    TimerStop(&TxNextPacketTimer);

    mib_req.Type = MIB_NETWORK_JOINED;
    status = LoRaMacMibGetRequestConfirm(&mib_req);

    if (status == LORAMAC_STATUS_OK) {
        if (mib_req.Param.IsNetworkJoined == true) {
            g_lwan_device_state = DEVICE_STATE_SEND;
        } else {
            rejoin_flag = true;
            g_lwan_device_state = DEVICE_STATE_JOIN;
        }
    }
}

static void mcps_confirm(McpsConfirm_t *mcpsConfirm)
{
    if (mcpsConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK) {
#ifdef CONFIG_LINKWAN_AT   
        notify_host();
        PRINTF_AT("\r\nOK+SENT:%02X\r\n", mcpsConfirm->NbRetries);
#endif        
    } else {
#ifdef CONFIG_LINKWAN_AT   
        notify_host();
        PRINTF_AT("\r\nERR+SENT:%02X\r\n", mcpsConfirm->NbRetries);
#endif  

#if 0
        if(mcpsConfirm->McpsRequest ==MCPS_CONFIRMED) {
            if(g_lwan_dev_config_p->modes.join_mode == JOIN_MODE_OTAA) {
                reset_join_state();
#ifdef CONFIG_LINKWAN            
                MibRequestConfirm_t mibReq;
                uint16_t channelsMaskTemp[8] = {0};

                g_lwan_dev_config_p->join_settings.join_method = JOIN_METHOD_DEF;
                for (uint8_t i = 0; i < 16; i++) {
                    if ((g_lwan_dev_config_p->freqband_mask & (1 << i)) != 0) {
                        channelsMaskTemp[i / 2] |= (0xFF << ((i % 2) * 8));
                    }
                }
                channelsMaskTemp[0] |= 0XFF00;
                mibReq.Type = MIB_CHANNELS_MASK;
                mibReq.Param.ChannelsMask = channelsMaskTemp;
                LoRaMacMibSetRequestConfirm(&mibReq);
#endif    
                DBG_LINKWAN("Not receive Ack,Start to Join...\r\n");
            }else{
#ifdef CONFIG_LINKWAN
                MibRequestConfirm_t mibReq;
                mibReq.Type = MIB_FREQ_BAND;
                mibReq.Param.freqband = get_next_freqband();
                LoRaMacMibSetRequestConfirm(&mibReq);
#endif  
            }
        }
#endif         
    }
    next_tx = true;
}

static void mcps_indication(McpsIndication_t *mcpsIndication)
{
    if ( mcpsIndication->Status != LORAMAC_EVENT_INFO_STATUS_OK ) {
        return;
    }
    // Check Multicast
    // Check Port
    // Check Datarate
    // Check FramePending
    // Check Buffer
    // Check BufferSize
    // Check Rssi
    // Check Snr
    // Check RxSlot
    DBG_PRINTF( "receive data: rssi = %d, snr = %d, datarate = %d\r\n", mcpsIndication->Rssi, mcpsIndication->Snr,
                 mcpsIndication->RxDatarate);
    lwan_dev_status_set(DEVICE_STATUS_SEND_PASS_WITH_DL);
    if (mcpsIndication->RxData == true) {
        switch ( mcpsIndication->Port ) {
            case 224:
                break;
            default: {            
                rx_data.Port = mcpsIndication->Port;
                rx_data.BuffSize = mcpsIndication->BufferSize;
                memcpy1(rx_data.Buff, mcpsIndication->Buffer, rx_data.BuffSize);
                app_callbacks->LoraRxData(&rx_data);
                break;
            }
        }
#ifdef CONFIG_DEBUG_LINKWAN
    } else if (mcpsIndication->AckReceived) {
        DBG_LINKWAN( "rx, ACK, index %u\r\n", (unsigned int)g_ack_index++);
#endif
    }
#ifdef CONFIG_LINKWAN_AT   
    uint8_t confirm = 0;
    if(mcpsIndication->McpsIndication==MCPS_UNCONFIRMED)
        confirm = 0;
    else if(mcpsIndication->McpsIndication==MCPS_CONFIRMED)
        confirm = 1;
    uint8_t type = confirm | mcpsIndication->AckReceived<<1 | 
                   mcpsIndication->LinkCheckAnsReceived<<2 | mcpsIndication->DevTimeAnsReceived<<3;
    notify_host();
    PRINTF_AT("\r\nOK+RECV:%02X,%02X,%02X", type, mcpsIndication->Port, mcpsIndication->BufferSize);
    if(mcpsIndication->BufferSize) {
        PRINTF_AT(",");
        for(int i=0; i<mcpsIndication->BufferSize; i++) {
            PRINTF_AT("%02X", mcpsIndication->Buffer[i]);
        }
    }
    PRINTF_AT("\r\n");
#endif

#ifdef CONFIG_LWAN    
    if(mcpsIndication->UplinkNeeded) {
        g_lwan_device_state = DEVICE_STATE_SEND_MAC;
    }
#endif 
}

static uint32_t generate_rejoin_delay(void)
{
    uint32_t rejoin_delay = 0;

    while (rejoin_delay < g_lwan_dev_config_p->join_settings.join_interval*1000) {
        rejoin_delay += (rand1() % 250);
    }

    return rejoin_delay;
}

static void mlme_confirm( MlmeConfirm_t *mlmeConfirm )
{
    uint32_t rejoin_delay = 8*1000;
    MibRequestConfirm_t mibReq;

    switch ( mlmeConfirm->MlmeRequest ) {
        case MLME_JOIN: {
            if (mlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK) {
                // Status is OK, node has joined the network
                g_lwan_device_state = DEVICE_STATE_JOINED;
                lwan_dev_status_set(DEVICE_STATUS_JOIN_PASS);
#ifdef CONFIG_LINKWAN_AT
                notify_host();
                PRINTF_AT("%s:OK\r\n", LORA_AT_CJOIN);
#endif                
            } else {
                lwan_dev_status_set(DEVICE_STATUS_JOIN_FAIL);
                
#ifdef CONFIG_LINKWAN                
                // Join was not successful. Try to join again
                reset_join_state();
                if (g_lwan_dev_config_p->join_settings.join_method != JOIN_METHOD_SCAN) {
                    g_lwan_dev_config_p->join_settings.join_method = 
                        (g_lwan_dev_config_p->join_settings.join_method + 1) % JOIN_METHOD_NUM;
                    rejoin_delay = generate_rejoin_delay();
                    if (g_lwan_dev_config_p->join_settings.join_method == JOIN_METHOD_SCAN) {
                        g_freqband_num = get_freqband_num();
                    }
                }

                if (g_lwan_dev_config_p->join_settings.join_method == JOIN_METHOD_SCAN) {
                    if (g_freqband_num == 0) {
                        g_lwan_dev_config_p->join_settings.join_method = JOIN_METHOD_DEF;
                        rejoin_delay = 60 * 60 * 1000;  // 1 hour
#ifdef CONFIG_LINKWAN_AT      
                        notify_host();
                        PRINTF_AT("%s:FAIL\r\n", LORA_AT_CJOIN);
#endif                        
                        DBG_LINKWAN("Wait 1 hour for new round of scan\r\n");
                    } else {
                        g_freqband_num--;
                        rejoin_delay = generate_rejoin_delay();
                    }
                }
#else
#ifdef CONFIG_LINKWAN_AT                          
                PRINTF_AT("%s:FAIL\r\n", LORA_AT_CJOIN);
#endif          
                rejoin_delay = generate_rejoin_delay();
#endif    
                TimerSetValue(&TxNextPacketTimer, rejoin_delay);
                TimerStart(&TxNextPacketTimer);
                rejoin_flag = false;
            }
            break;
        }
        case MLME_LINK_CHECK: {
#ifdef CONFIG_LINKWAN_AT
            notify_host();
            PRINTF_AT("+CLINKCHECK: %d, %d, %d, %d, %d\r\n", mlmeConfirm->Status, mlmeConfirm->DemodMargin, mlmeConfirm->NbGateways, mlmeConfirm->Rssi, mlmeConfirm->Snr);
#endif            
            if ( mlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK ) {
                // Check DemodMargin
                // Check NbGateways
            } else {
                lwan_dev_status_set(DEVICE_STATUS_NETWORK_ABNORMAL);
            }
            break;
        }
        case MLME_DEVICE_TIME:
        {
            if( mlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK ){
                // Switch to the next state immediately
                g_lwan_device_state = DEVICE_STATE_BEACON_ACQUISITION;
                next_tx = true;
            } else {
                //No device time Ans
                g_lwan_device_state = DEVICE_STATE_SLEEP;
            }
            
            break;
        }
        case MLME_BEACON_ACQUISITION:
        {
            if( mlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK ) {
                //beacon received
                g_lwan_device_state = DEVICE_STATE_REQ_PINGSLOT_ACK;
                g_beacon_retry_times = 0;
            } else {
                //beacon lost
                if(g_beacon_retry_times < MAX_BEACON_RETRY_TIMES) {
                    g_beacon_retry_times ++;
                    g_lwan_device_state = DEVICE_STATE_REQ_DEVICE_TIME;
                } else {
                    g_beacon_retry_times = 0;
                    g_lwan_device_state = DEVICE_STATE_SLEEP;
                }
            }
            break;
        }
        case MLME_PING_SLOT_INFO:
        {
            if( mlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
            {
                mibReq.Type = MIB_DEVICE_CLASS;
                mibReq.Param.Class = CLASS_B;
                LoRaMacMibSetRequestConfirm( &mibReq );
                
                mibReq.Type = MIB_PING_SLOT_DATARATE;
                mibReq.Param.PingSlotDatarate = g_lwan_dev_config_p->classb_param.pslot_dr;
                LoRaMacMibSetRequestConfirm( &mibReq );

                g_lwan_device_state = DEVICE_STATE_SEND;
                next_tx = true;
            }
            else
            {
                g_lwan_device_state = DEVICE_STATE_REQ_PINGSLOT_ACK;
            }
            break;
        }
        default:
            break;
    }
    next_tx = true;
}

static void mlme_indication( MlmeIndication_t *mlmeIndication )
{
    MibRequestConfirm_t mibReq;

    switch( mlmeIndication->MlmeIndication )
    {
        case MLME_SCHEDULE_UPLINK:
        {// The MAC signals that we shall provide an uplink as soon as possible
            g_lwan_device_state = DEVICE_STATE_SEND_MAC;
            break;
        }
        case MLME_BEACON_LOST:
        {
            mibReq.Type = MIB_DEVICE_CLASS;
            mibReq.Param.Class = CLASS_A;
            LoRaMacMibSetRequestConfirm( &mibReq );

            // Switch to class A again
            g_lwan_device_state = DEVICE_STATE_REQ_DEVICE_TIME;
            break;
        }
        case MLME_BEACON:
        {
            if( mlmeIndication->Status == LORAMAC_EVENT_INFO_STATUS_BEACON_LOCKED )
            {
                if(mlmeIndication->BeaconInfo.GwSpecific.InfoDesc==3){ //NetID+GatewayID
                    uint8_t *info = mlmeIndication->BeaconInfo.GwSpecific.Info;
                    if((gGatewayID[0]|gGatewayID[1]|gGatewayID[2]) 
                    && (memcmp(&info[3],gGatewayID,3)!=0)){//GatewayID not 0 and changed
                        //send an uplink in [0:120] seconds
                        TimerStop(&TxNextPacketTimer);
                        TimerSetValue(&TxNextPacketTimer,randr(0,120000));
                        TimerStart(&TxNextPacketTimer);                       
                    }
                    memcpy(gGatewayID,&info[3],3);
                }
            }
            break;
        }
        default:
            break;
    }
}


static void start_dutycycle_timer(void)
{
    MibRequestConfirm_t mib_req;
    LoRaMacStatus_t status;

    TimerStop(&TxNextPacketTimer);
    mib_req.Type = MIB_NETWORK_JOINED;
    status = LoRaMacMibGetRequestConfirm(&mib_req);
    if (status == LORAMAC_STATUS_OK) {
        if (mib_req.Param.IsNetworkJoined == true &&
            g_lwan_mac_config_p->modes.report_mode == TX_ON_TIMER && g_lwan_mac_config_p->report_interval != 0) {
            TimerSetValue(&TxNextPacketTimer, g_lwan_mac_config_p->report_interval*1000);
            TimerStart(&TxNextPacketTimer);
            return;
        }
    }
    if (g_lwan_mac_config_p->report_interval == 0 && g_lwan_mac_config_p->modes.report_mode == TX_ON_TIMER) {
        g_lwan_mac_config_p->modes.report_mode = TX_ON_NONE;
    }
}

MulticastParams_t *get_lora_cur_multicast(void)
{
    MibRequestConfirm_t mib_req;
    LoRaMacStatus_t status;

    mib_req.Type = MIB_MULTICAST_CHANNEL;
    status = LoRaMacMibGetRequestConfirm(&mib_req);
    if (status == LORAMAC_STATUS_OK) {
        return mib_req.Param.MulticastList;
    }
    return NULL;
}

static void print_dev_info(void)
{
    DBG_LINKWAN("Frequency Band: ");
    #if defined( REGION_AS923 )
      PRINTF_RAW("AS923 v1.0\n\r");
    #elif defined( REGION_AU915 )
      PRINTF_RAW("AU915 v1.0\n\r");
    #elif defined( REGION_CN470 )
      PRINTF_RAW("CN470 v1.0\n\r");
    #elif defined( REGION_CN779 )
      PRINTF_RAW("CN779 v1.0\n\r");
    #elif defined( REGION_EU433 )
      PRINTF_RAW("EU433 v1.0\n\r");
    #elif defined( REGION_IN865 )
      PRINTF_RAW("IN865 v1.0\n\r");
    #elif defined( REGION_EU868 )
      PRINTF_RAW("EU868 v1.0\n\r");
    #elif defined( REGION_KR920 )
      PRINTF_RAW("KR920 v1.0\n\r");
    #elif defined( REGION_US915 )
      PRINTF_RAW("US915 v1.0\n\r");
    #else
        #error "Please define a region in the compiler options."
    #endif
    
    if(g_lwan_dev_config_p->modes.join_mode == JOIN_MODE_OTAA){
        
        DBG_LINKWAN("OTAA\r\n" );
        DBG_LINKWAN("DevEui= %02X%02X%02X%02X%02X%02X%02X%02X\r\n",
                    g_lwan_dev_keys_p->ota.deveui[0], g_lwan_dev_keys_p->ota.deveui[1], g_lwan_dev_keys_p->ota.deveui[2], g_lwan_dev_keys_p->ota.deveui[3], \
                    g_lwan_dev_keys_p->ota.deveui[4], g_lwan_dev_keys_p->ota.deveui[5], g_lwan_dev_keys_p->ota.deveui[6], g_lwan_dev_keys_p->ota.deveui[7]);
        DBG_LINKWAN("AppEui= %02X%02X%02X%02X%02X%02X%02X%02X\r\n",
                    g_lwan_dev_keys_p->ota.appeui[0], g_lwan_dev_keys_p->ota.appeui[1], g_lwan_dev_keys_p->ota.appeui[2], g_lwan_dev_keys_p->ota.appeui[3], \
                    g_lwan_dev_keys_p->ota.appeui[4], g_lwan_dev_keys_p->ota.appeui[5], g_lwan_dev_keys_p->ota.appeui[6], g_lwan_dev_keys_p->ota.appeui[7]);
        DBG_LINKWAN("AppKey= %02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X\r\n",
                    g_lwan_dev_keys_p->ota.appkey[0], g_lwan_dev_keys_p->ota.appkey[1], g_lwan_dev_keys_p->ota.appkey[2], g_lwan_dev_keys_p->ota.appkey[3], \
                    g_lwan_dev_keys_p->ota.appkey[4], g_lwan_dev_keys_p->ota.appkey[5], g_lwan_dev_keys_p->ota.appkey[6], g_lwan_dev_keys_p->ota.appkey[7], \
                    g_lwan_dev_keys_p->ota.appkey[8], g_lwan_dev_keys_p->ota.appkey[9], g_lwan_dev_keys_p->ota.appkey[10], g_lwan_dev_keys_p->ota.appkey[11], \
                    g_lwan_dev_keys_p->ota.appkey[12], g_lwan_dev_keys_p->ota.appkey[13], g_lwan_dev_keys_p->ota.appkey[14], g_lwan_dev_keys_p->ota.appkey[15]);
    } else if(g_lwan_dev_config_p->modes.join_mode == JOIN_MODE_ABP){
        DBG_LINKWAN("ABP\r\n");
        DBG_LINKWAN("DevAddr= %08X\r\n", (unsigned int)g_lwan_dev_keys_p->abp.devaddr);
        DBG_LINKWAN("NwkSKey= ");
        for (int i = 0; i < LORA_KEY_LENGTH; i++) {
            PRINTF_RAW("%02X", g_lwan_dev_keys_p->abp.nwkskey[i]);
        };
        PRINTF_RAW("\r\n");
        DBG_LINKWAN("AppSKey= ");
        for (int i = 0; i < LORA_KEY_LENGTH; i++) {
            PRINTF_RAW("%02X", g_lwan_dev_keys_p->abp.appskey[i]);
        };
        PRINTF_RAW("\r\n");
    }
    DBG_LINKWAN("class type %c\r\n", 'A' + g_lwan_dev_config_p->modes.class_mode);
    DBG_LINKWAN("freq mode %s\r\n", g_lwan_dev_config_p->modes.uldl_mode == ULDL_MODE_INTER ? "inter" : "intra");
    
    #if defined( REGION_US915 ) || defined( REGION_US915_HYBRID ) || defined ( REGION_AU915 ) || defined ( REGION_CN470 )
    DBG_LINKWAN("scan chn mask 0x%04x\r\n", g_lwan_dev_config_p->freqband_mask);
    #endif
}

void init_lwan_configs() 
{
    LWanDevKeys_t default_keys = LWAN_DEV_KEYS_DEFAULT;
    LWanDevConfig_t default_dev_config = LWAN_DEV_CONFIG_DEFAULT;
    LWanMacConfig_t default_mac_config = LWAN_MAC_CONFIG_DEFAULT;
    LWanProdctConfig_t default_prodct_config = LWAN_PRODCT_CONFIG_DEFAULT;
    g_lwan_dev_keys_p = lwan_dev_keys_init(&default_keys);
    g_lwan_dev_config_p = lwan_dev_config_init(&default_dev_config);
    g_lwan_mac_config_p = lwan_mac_config_init(&default_mac_config);
    g_lwan_prodct_config_p = lwan_prodct_config_init(&default_prodct_config);
    
        #if defined(REGION_EU868)
        MibRequestConfirm_t mibReq;
        LoRaMacTestSetDutyCycleOn( DISABLE );
        mibReq.Type = MIB_RX2_DEFAULT_CHANNEL;
        mibReq.Param.Rx2DefaultChannel = ( Rx2ChannelParams_t ){ 869525000, DR_3 };
        LoRaMacMibSetRequestConfirm( &mibReq );

        mibReq.Type = MIB_RX2_CHANNEL;
        mibReq.Param.Rx2Channel = ( Rx2ChannelParams_t ){ 869525000, DR_3 };
        LoRaMacMibSetRequestConfirm( &mibReq );
        #endif
        
    uint8_t FDR_status=(*((uint8_t *) (0x0ffff500u)));
   
    if(FDR_status==0x00)
    {                     
      uint8_t init_cfm=0,init_nb_value=1;
      #if defined(LORA_SENSOR_ENABLE)
      uint8_t init_reportMode=1;
      uint16_t init_reportInterval=60;//60s
      #endif
      
      #if defined(REGION_US915) || defined(REGION_AU915)
      {
        uint16_t freqband_mask;
        freqband_mask=0x0002;
        lwan_dev_config_set(DEV_CONFIG_FREQBAND_MASK, &freqband_mask);              
      }
      #elif defined(REGION_CN470)
      {
        uint16_t freqband_mask;
        freqband_mask=0x0400;
        lwan_dev_config_set(DEV_CONFIG_FREQBAND_MASK, &freqband_mask);
      }
      #endif
    
      lwan_mac_config_set(MAC_CONFIG_UNCONF_NBTRIALS, (void *)&init_nb_value);
      lwan_mac_config_set(MAC_CONFIG_CONFIRM_MSG, (void *)&init_cfm);
    
      #if defined(LORA_SENSOR_ENABLE)
      lwan_mac_config_set(MAC_CONFIG_REPORT_MODE, (void *)&init_reportMode);
      lwan_mac_config_set(MAC_CONFIG_REPORT_INTERVAL, (void *)&init_reportInterval);
      #endif
    
      CyDelay(100);
      SFlash_store_lora_config();
    }
    
    SFlash_read_lora_key();
    SFlash_read_lora_config();
}


void lora_init(LoRaMainCallback_t *callbacks)
{
    g_lwan_device_state = DEVICE_STATE_INIT;
    app_callbacks = callbacks;

#ifdef AOS_KV
    assert(aos_kv_init() == 0);
#endif

#ifdef CONFIG_LINKWAN_AT
    linkwan_at_init();
#endif
}

void lora_fsm( void )
{
    while (1) {
#ifdef CONFIG_LINKWAN_AT
        linkwan_at_process();
#endif
        if (Radio.IrqProcess != NULL) {
            Radio.IrqProcess();
        }
        
        #if defined(LORA_SENSOR_ENABLE)
        if( pin_wakeup_status==1)
        {
            MibRequestConfirm_t mib_req;
            LoRaMacStatus_t status;
   
            mib_req.Type = MIB_NETWORK_JOINED;
            status = LoRaMacMibGetRequestConfirm(&mib_req);
            if (status == LORAMAC_STATUS_OK) {
            if (mib_req.Param.IsNetworkJoined == true){
            
              app_callbacks->LoraTxData(&tx_data);
              send_frame();
              }
          else
           pin_wakeup_status=0;
         }
        }
        #endif
        switch (g_lwan_device_state) {
            case DEVICE_STATE_INIT: { 
                LoRaMacPrimitives.MacMcpsConfirm = mcps_confirm;
                LoRaMacPrimitives.MacMcpsIndication = mcps_indication;
                LoRaMacPrimitives.MacMlmeConfirm = mlme_confirm;
                LoRaMacPrimitives.MacMlmeIndication = mlme_indication;
                LoRaMacCallbacks.GetBatteryLevel = app_callbacks->BoardGetBatteryLevel;
                LoRaMacCallbacks.GetTemperatureLevel = NULL;
#if defined(REGION_AS923)
                LoRaMacInitialization(&LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_AS923);
#elif defined(REGION_AU915)
                LoRaMacInitialization(&LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_AU915);
#elif defined(REGION_CN470)
                LoRaMacInitialization(&LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_CN470);
#elif defined(REGION_CN779)
                LoRaMacInitialization(&LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_CN779);
#elif defined(REGION_EU433)
                LoRaMacInitialization(&LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_EU433);
#elif defined(REGION_IN865)
                LoRaMacInitialization(&LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_IN865);
#elif defined(REGION_EU868)
                LoRaMacInitialization(&LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_EU868);
#elif defined(REGION_KR920)
                LoRaMacInitialization(&LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_KR920);
#elif defined(REGION_US915)
                LoRaMacInitialization(&LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_US915);
#elif defined(REGION_US915_HYBRID)
                LoRaMacInitialization(&LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_US915_HYBRID);
#elif defined( REGION_CN470A )
                LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_CN470A);
#else
#error "Please define a region in the compiler options."
#endif
                init_lwan_configs();
                if(!lwan_is_key_valid(g_lwan_dev_keys_p->pkey, LORA_KEY_LENGTH))
                    print_dev_info();
                
                TimerInit( &TxNextPacketTimer, on_tx_next_packet_timer_event );

                lwan_dev_params_update();
                
                #if defined(REGION_US915) || defined ( REGION_AU915 ) || defined ( REGION_CN470 )
                MibRequestConfirm_t mibReq;
                uint16_t channelsMaskTemp[6] = {0,0,0,0,0,0};
                uint16_t freqband_mask;
                lwan_dev_config_get(DEV_CONFIG_FREQBAND_MASK, &freqband_mask);
                
                #if defined(REGION_US915) || defined ( REGION_AU915 )
                {
                    if(freqband_mask>0x0080)
                    {
                        freqband_mask=0x0002;
                        lwan_dev_config_set(DEV_CONFIG_FREQBAND_MASK, (void *)&freqband_mask);
                    }
                }
                #elif defined(REGION_CN470)
                {
                    if(freqband_mask>0x0800)
                    {
                        freqband_mask=0x0002;
                        lwan_dev_config_set(DEV_CONFIG_FREQBAND_MASK, (void *)&freqband_mask);
                    }
                }
                #endif
                
                if(freqband_mask==0)
                {
                  #if defined(REGION_US915) || defined ( REGION_AU915 )
                  {
                      for(int i=0;i<4;i++)
                      {
                        channelsMaskTemp[i]=0xFFFF;
                      }
                      channelsMaskTemp[4]=0x00FF;
                      channelsMaskTemp[5]=0x0000;
                  }
                  #elif defined(REGION_CN470)
                  {
                      for(int i=0;i<6;i++)
                      {
                        channelsMaskTemp[i]=0xFFFF;
                      }  
                  }
                  #endif
                
                  mibReq.Type = MIB_CHANNELS_MASK;                    
                  mibReq.Param.ChannelsMask = channelsMaskTemp;
                  LoRaMacMibSetRequestConfirm(&mibReq);
                }else if(freqband_mask!=2)
                {
                    mibReq.Type = MIB_CHANNELS_MASK;
                    LoRaMacMibGetRequestConfirm(&mibReq);
                    for(int i=0;i<6;i++)
                    {
                      channelsMaskTemp[i]=mibReq.Param.ChannelsMask[i];
                    }
                  
                  mibReq.Type = MIB_CHANNELS_MASK;
                  channelsMaskTemp[0] &=0x00FF;
                  channelsMaskTemp[4] =0x00FF;
                  mibReq.Param.ChannelsMask = channelsMaskTemp;
                  LoRaMacMibSetRequestConfirm(&mibReq);
                }
                else
                {
                  mibReq.Type = MIB_CHANNELS_MASK;
                  LoRaMacMibGetRequestConfirm(&mibReq);
                  for(int i=0;i<6;i++)
                  {
                    channelsMaskTemp[i]=mibReq.Param.ChannelsMask[i];
                  }
                  mibReq.Type = MIB_CHANNELS_MASK;
                  channelsMaskTemp[4] =0x00FF; 
                  mibReq.Param.ChannelsMask = channelsMaskTemp;
                  LoRaMacMibSetRequestConfirm(&mibReq);
                }
                #endif                                             
                
                if(g_lwan_dev_config_p->modes.join_mode == JOIN_MODE_ABP){
                    MibRequestConfirm_t mibReq;
                    mibReq.Type = MIB_NET_ID;
                    mibReq.Param.NetID = LORAWAN_NETWORK_ID;
                    LoRaMacMibSetRequestConfirm(&mibReq);
                    mibReq.Type = MIB_DEV_ADDR;
                    mibReq.Param.DevAddr = g_lwan_dev_keys_p->abp.devaddr;
                    LoRaMacMibSetRequestConfirm(&mibReq);    
                    mibReq.Type = MIB_NWK_SKEY;
                    mibReq.Param.NwkSKey = g_lwan_dev_keys_p->abp.nwkskey;
                    LoRaMacMibSetRequestConfirm(&mibReq);
                    mibReq.Type = MIB_APP_SKEY;
                    mibReq.Param.AppSKey = g_lwan_dev_keys_p->abp.appskey;
                    LoRaMacMibSetRequestConfirm(&mibReq);
#ifdef CONFIG_LINKWAN                    
                    mibReq.Type = MIB_FREQ_BAND;
                    mibReq.Param.freqband = get_next_freqband();
                    LoRaMacMibSetRequestConfirm(&mibReq);
#endif                    
                    mibReq.Type = MIB_NETWORK_JOINED;
                    mibReq.Param.IsNetworkJoined = true;
                    LoRaMacMibSetRequestConfirm(&mibReq);
                    
                    lwan_mac_params_update();   
#ifdef CONFIG_LORA_VERIFY 
                    g_lwan_device_state = DEVICE_STATE_SEND;
#else
                    g_lwan_device_state = DEVICE_STATE_SLEEP;
#endif 
		        }
               
              #ifdef LORA_SENSOR_ENABLE
            
              if(g_lwan_dev_config_p->modes.join_mode == JOIN_MODE_OTAA)
              {
                g_lwan_device_state = DEVICE_STATE_JOIN;
              }
              else if(g_lwan_dev_config_p->modes.join_mode == JOIN_MODE_ABP)
                {
                    g_lwan_device_state = DEVICE_STATE_JOINED;
                }
                            
              #else 
              linkwan_at_prompt_print();
              g_lwan_device_state = DEVICE_STATE_SLEEP;
              
            #endif           
                
                lwan_dev_status_set(DEVICE_STATUS_IDLE);
                break;
            }
                
            case DEVICE_STATE_JOIN: {
                if(g_lwan_dev_config_p->modes.join_mode == JOIN_MODE_OTAA){
                    MlmeReq_t mlmeReq;

                    mlmeReq.Type = MLME_JOIN;
                    mlmeReq.Req.Join.DevEui = g_lwan_dev_keys_p->ota.deveui;
                    mlmeReq.Req.Join.AppEui = g_lwan_dev_keys_p->ota.appeui;
                    mlmeReq.Req.Join.AppKey = g_lwan_dev_keys_p->ota.appkey;
#ifdef CONFIG_LINKWAN    
                    mlmeReq.Req.Join.method = g_lwan_dev_config_p->join_settings.join_method;
                    if (g_lwan_dev_config_p->join_settings.join_method == JOIN_METHOD_STORED) {
                        mlmeReq.Req.Join.freqband = g_lwan_dev_config_p->join_settings.stored_freqband;
                        mlmeReq.Req.Join.datarate = g_lwan_dev_config_p->join_settings.stored_datarate;
                        mlmeReq.Req.Join.NbTrials = 3;
                    } else {
                        mlmeReq.Req.Join.NbTrials = g_lwan_dev_config_p->join_settings.join_trials;
                    }
#else
                    mlmeReq.Req.Join.NbTrials = g_lwan_dev_config_p->join_settings.join_trials;
#endif

                    if (next_tx == true && rejoin_flag == true) {
                        if (LoRaMacMlmeRequest(&mlmeReq) == LORAMAC_STATUS_OK) {
                            next_tx = false;
                        }
#ifdef CONFIG_LINKWAN                        
                        DBG_LINKWAN("Start to Join, method %d, nb_trials:%d\r\n",
                                    g_lwan_dev_config_p->join_settings.join_method, mlmeReq.Req.Join.NbTrials);
#else
                        DBG_LINKWAN("Start to Join, nb_trials:%d\r\n", mlmeReq.Req.Join.NbTrials);
#endif                        
    
                    }
		        }
                g_lwan_device_state = DEVICE_STATE_SLEEP;
                break;
            }
            case DEVICE_STATE_JOINED: {
                DBG_LINKWAN("Joined\n\r");
#ifdef CONFIG_LINKWAN                
                JoinSettings_t join_settings;
                lwan_dev_config_get(DEV_CONFIG_JOIN_SETTINGS, &join_settings);
                    
                MibRequestConfirm_t mib_req;
                mib_req.Type = MIB_FREQ_BAND;
                LoRaMacMibGetRequestConfirm(&mib_req);
                join_settings.stored_freqband = mib_req.Param.freqband;
                mib_req.Type = MIB_CHANNELS_DATARATE;
                LoRaMacMibGetRequestConfirm(&mib_req);
                join_settings.stored_datarate = mib_req.Param.ChannelsDatarate;
                join_settings.join_method = JOIN_METHOD_STORED;
                
                lwan_dev_config_set(DEV_CONFIG_JOIN_SETTINGS, &join_settings);
#endif                
                
                lwan_mac_params_update();
                
                if(g_lwan_dev_config_p->modes.class_mode == CLASS_B) {
                    g_lwan_device_state = DEVICE_STATE_REQ_DEVICE_TIME;
                }else{
#ifdef CONFIG_LORA_VERIFY                    
                    g_lwan_device_state = DEVICE_STATE_SEND;
#else
                    g_lwan_device_state = DEVICE_STATE_SLEEP;
#endif    
                }
                break;
            }
            case DEVICE_STATE_REQ_DEVICE_TIME: {
                MlmeReq_t mlmeReq;
                MibRequestConfirm_t mib_req;

                mib_req.Type = MIB_NETWORK_JOINED;
                LoRaMacMibGetRequestConfirm(&mib_req);
                if (mib_req.Param.IsNetworkJoined == true) {
                    if( next_tx == true ) {
                        mlmeReq.Type = MLME_DEVICE_TIME;
                        LoRaMacMlmeRequest( &mlmeReq );
                    }
                    g_lwan_device_state = DEVICE_STATE_SEND_MAC;
                } else {
                    g_lwan_device_state = DEVICE_STATE_SLEEP;
                }
                
                break;
            }
            case DEVICE_STATE_BEACON_ACQUISITION: {
                MlmeReq_t mlmeReq;

                if( next_tx == true ) {
                    if(g_lwan_dev_config_p->classb_param.beacon_freq)
                        LoRaMacClassBBeaconFreqReq(g_lwan_dev_config_p->classb_param.beacon_freq);
                    if(g_lwan_dev_config_p->classb_param.pslot_freq)
                        LoRaMacClassBPingSlotChannelReq(g_lwan_dev_config_p->classb_param.pslot_dr, g_lwan_dev_config_p->classb_param.pslot_freq);
                    mlmeReq.Type = MLME_BEACON_ACQUISITION;
                    LoRaMacMlmeRequest( &mlmeReq );
                }
                g_lwan_device_state = DEVICE_STATE_SLEEP;
                break;
            }
            case DEVICE_STATE_REQ_PINGSLOT_ACK: {
                MlmeReq_t mlmeReq;

                if( next_tx == true ) {
                    mlmeReq.Type = MLME_PING_SLOT_INFO;
                    mlmeReq.Req.PingSlotInfo.PingSlot.Fields.Periodicity = g_lwan_dev_config_p->classb_param.periodicity;
                    mlmeReq.Req.PingSlotInfo.PingSlot.Fields.RFU = 0;
                    LoRaMacMlmeRequest( &mlmeReq );
                }
                g_lwan_device_state = DEVICE_STATE_SEND_MAC;
                break;
            }
            case DEVICE_STATE_SEND: {
                if (g_lwan_mac_config_p->modes.report_mode == TX_ON_TIMER) {
                    start_dutycycle_timer();
                }
                if (next_tx == true) {
                    prepare_tx_frame();
                    next_tx = send_frame();
                }
                
                g_lwan_device_state = DEVICE_STATE_SLEEP;
                break;
            }
            case DEVICE_STATE_SEND_MAC: {
                if (next_tx == true) {
                    tx_data.BuffSize = 0;
                    next_tx = send_frame();
                }
                g_lwan_device_state = DEVICE_STATE_SLEEP;
                break;
            }
            case DEVICE_STATE_SLEEP: {       
#ifndef LOW_POWER_DISABLE
                LowPower_Handler( );
#endif
                break;
            }
            default: {
                g_lwan_device_state = DEVICE_STATE_INIT;
                break;
            }
        }
    }
}

DeviceState_t lwan_dev_state_get( void )
{
    return g_lwan_device_state;
}

void lwan_dev_state_set(DeviceState_t state)
{
    if (g_lwan_device_state == DEVICE_STATE_SLEEP) {
        TimerStop(&TxNextPacketTimer);
    }
    g_lwan_device_state = state;
}

bool lwan_dev_status_set(DeviceStatus_t ds)
{
    g_lwan_device_status = ds;
    return true;
}
DeviceStatus_t lwan_dev_status_get(void)
{
    return g_lwan_device_status;
}

bool lwan_is_dev_busy()
{
    MibRequestConfirm_t mibReq;
    mibReq.Type = MIB_MAC_STATE;
    LoRaMacMibGetRequestConfirm(&mibReq);
    
    if(g_lwan_device_state == DEVICE_STATE_SLEEP 
        && mibReq.Param.LoRaMacState == 0)
        return false;
    
    return true;
}

int lwan_mac_req_send(int type, void *param)
{
    MlmeReq_t mlmeReq;
    int ret = LWAN_SUCCESS;
    
    switch(type) {
        case MAC_REQ_LINKCHECK: {
            mlmeReq.Type = MLME_LINK_CHECK;
            break;
        }
        case MAC_REQ_DEVICE_TIME: {
            break;
        }
        case MAC_REQ_PSLOT_INFO: {
            uint8_t periodicity = *(uint8_t *)param;
            if(periodicity>7) 
                return LWAN_ERROR;
            
            mlmeReq.Type = MLME_PING_SLOT_INFO;
            mlmeReq.Req.PingSlotInfo.PingSlot.Fields.Periodicity = periodicity;
            mlmeReq.Req.PingSlotInfo.PingSlot.Fields.RFU = 0;
            
            ClassBParam_t classb_param;
            lwan_dev_config_get(DEV_CONFIG_CLASSB_PARAM, &classb_param);
            classb_param.periodicity = periodicity;
            lwan_dev_config_set(DEV_CONFIG_CLASSB_PARAM, &classb_param);
            break;
        }
        default: {
            ret = LWAN_ERROR;
            break;
        }
    }
    
    if (LoRaMacMlmeRequest(&mlmeReq) == LORAMAC_STATUS_OK) {
        g_lwan_device_state = DEVICE_STATE_SEND_MAC;
    }
    

    return ret;
}

int lwan_join(uint8_t bJoin, uint8_t bAutoJoin, uint16_t joinInterval, uint16_t joinRetryCnt)
{
    int ret = LWAN_SUCCESS;
    JoinSettings_t join_settings;
    lwan_dev_config_get(DEV_CONFIG_JOIN_SETTINGS, &join_settings);
    join_settings.auto_join = bAutoJoin;
    if(joinInterval>=7 && joinInterval<=255)
        join_settings.join_interval = joinInterval;
    if(joinRetryCnt>=1 && joinRetryCnt<=255)
        join_settings.join_trials = joinRetryCnt;
    lwan_dev_config_set(DEV_CONFIG_JOIN_SETTINGS, &join_settings);
        
    if(bJoin == 0){//stop join
        TimerStop(&TxNextPacketTimer);
        MibRequestConfirm_t mib_req;
        LoRaMacStatus_t status;
        mib_req.Type = MIB_NETWORK_JOINED;
        mib_req.Param.IsNetworkJoined = false;
        status = LoRaMacMibSetRequestConfirm(&mib_req);

        if (status != LORAMAC_STATUS_OK)
            return LWAN_ERROR;
        g_lwan_device_state = DEVICE_STATE_SLEEP;
        rejoin_flag = bAutoJoin;
    } else if(bJoin == 1){
        MibRequestConfirm_t mib_req;
        mib_req.Type = MIB_NETWORK_JOINED;
        LoRaMacStatus_t status = LoRaMacMibGetRequestConfirm(&mib_req);
        if (status != LORAMAC_STATUS_OK) 
            return LWAN_ERROR;
        
        if (mib_req.Param.IsNetworkJoined == true) {
            mib_req.Type = MIB_NETWORK_JOINED;
            mib_req.Param.IsNetworkJoined = false;
            status = LoRaMacMibSetRequestConfirm(&mib_req);
            if(status  != LORAMAC_STATUS_OK) {
                return LWAN_ERROR;
            }
            DBG_LINKWAN("Rejoin again...\r");
        }
        
        TimerStop(&TxNextPacketTimer);   
        rejoin_flag = true;
        reset_join_state();
    } else{
        ret = LWAN_ERROR;
    }
    
    return ret;
}

int lwan_data_send(uint8_t confirm, uint8_t Nbtrials, uint8_t *payload, uint8_t len)
{
    MibRequestConfirm_t mib_req;
    LoRaMacStatus_t status;

    TimerStop(&TxNextPacketTimer);

    mib_req.Type = MIB_NETWORK_JOINED;
    status = LoRaMacMibGetRequestConfirm(&mib_req);
    if (status == LORAMAC_STATUS_OK) {
        if (mib_req.Param.IsNetworkJoined == true) {
            g_data_send_msg_type = confirm;
            memcpy(tx_data.Buff, payload, len);
            tx_data.BuffSize = len;
            g_data_send_nbtrials = Nbtrials;
            g_lwan_device_state = DEVICE_STATE_SEND;
            return LWAN_SUCCESS;
        }
    }
    return LWAN_ERROR;
}

int lwan_data_recv(uint8_t *port, uint8_t **payload, uint8_t *size)
{
    if(!port || !payload || !size)
        return LWAN_ERROR;
    *port = rx_data.Port;
    *size = rx_data.BuffSize;
    *payload = rx_data.Buff;
    
    rx_data.BuffSize = 0;
    return LWAN_SUCCESS;
}

uint8_t lwan_dev_battery_get()
{
    return app_callbacks->BoardGetBatteryLevel();
}

int lwan_dev_rssi_get(uint8_t band, int16_t *channel_rssi)
{
    //CN470A Only
    uint8_t FreqBandStartChannelNum[16] = {0, 8, 16, 24, 100, 108, 116, 124, 68, 76, 84, 92, 166, 174, 182, 190};
    if(band>=16) 
        return LWAN_ERROR;

    Radio.SetModem(MODEM_LORA);
    for (uint8_t i = 0; i < 8; i++) {
        uint32_t freq = 470300000 + (FreqBandStartChannelNum[band] + i) * 200000;
        
        Radio.SetChannel(freq);
        Radio.Rx( 0 );
        CyDelay(3);
        
        channel_rssi[i] = Radio.Rssi(MODEM_LORA);
    }
    
    Radio.Sleep();
    
    return LWAN_SUCCESS;
}


bool lwan_multicast_add(void *multicastInfo )
{
    MibRequestConfirm_t mibset;
    LoRaMacStatus_t status;
    MulticastParams_t *pmulticastInfo;

    pmulticastInfo = aos_malloc(sizeof(MulticastParams_t));
    if (!pmulticastInfo)
        return false;
    memcpy(pmulticastInfo, multicastInfo, sizeof(MulticastParams_t));
    mibset.Type = MIB_MULTICAST_CHANNEL;
    mibset.Param.MulticastList = pmulticastInfo;
    status = LoRaMacMibSetRequestConfirm(&mibset);
    if (status !=  LORAMAC_STATUS_OK) {
        return false;
    }
    return true;
}

bool lwan_multicast_del(uint32_t dev_addr)
{
    MulticastParams_t *multiCastNode = get_lora_cur_multicast();
    if (multiCastNode == NULL) {
        return false;
    }
    
    while (multiCastNode != NULL) {
        if (dev_addr == multiCastNode->Address) {
            MibRequestConfirm_t mibset;
            LoRaMacStatus_t status;
            mibset.Type = MIB_MULTICAST_CHANNEL_DEL;
            mibset.Param.MulticastList = multiCastNode;
            status = LoRaMacMibSetRequestConfirm(&mibset);
            if (status !=  LORAMAC_STATUS_OK) {
                return false;
            } else {
                aos_free(mibset.Param.MulticastList);
                return true;
            }
        }
        multiCastNode = multiCastNode->Next;

    }
    return false;
}

uint8_t lwan_multicast_num_get(void)
{
    MulticastParams_t *multiCastNode = get_lora_cur_multicast();
    if (multiCastNode == NULL) {
        return 0;
    }
    uint8_t num = 0;
    while (multiCastNode != NULL) {
        num++;
        multiCastNode = multiCastNode->Next;
    }
    return num;
}

void lwan_sys_reboot(int8_t mode)
{
    if (mode == 0) {
	    HW_Reset(0);
    } else if (mode == 1) {
        if (next_tx == true) {
            prepare_tx_frame();
            next_tx = send_frame();
            HW_Reset(0);
        }
    } else if (mode == 7) {
        HW_Reset(1);
    }
}

void SFlash_store_lora_key(void)
{
    uint8_t store_data_in_sflash[CY_SFLASH_SIZEOF_USERROW]={0};
    uint8_t buf[16]={0};
    
    lwan_dev_keys_get(DEV_KEYS_OTA_DEVEUI, buf);
    for(uint8_t i=0,j=0;i<8;i++,j++)
    {
     store_data_in_sflash[i]= buf[j];
    }
    
    lwan_dev_keys_get(DEV_KEYS_OTA_APPEUI, buf);
     for(uint8_t i=8,j=0;i<16;i++,j++)
    {
      store_data_in_sflash[i]=buf[j];
    }
    
    lwan_dev_keys_get(DEV_KEYS_OTA_APPKEY, buf);
    for(uint8_t i=16,j=0;i<32;i++,j++)
    {
      store_data_in_sflash[i]=buf[j];
    }
    
    uint32_t devaddr;
    lwan_dev_keys_get(DEV_KEYS_ABP_DEVADDR, &devaddr);
    for(uint8_t i=32,j=24;i<36;i++,j-=8)
    {
      store_data_in_sflash[i]=(devaddr>>j)&0xff;
    }
    
    lwan_dev_keys_get(DEV_KEYS_ABP_NWKSKEY, buf);
    for(uint8_t i=36,j=0;i<52;i++,j++)
    {
      store_data_in_sflash[i]=buf[j];
    }
    
    lwan_dev_keys_get(DEV_KEYS_ABP_APPSKEY, buf);
    for(uint8_t i=52,j=0;i<68;i++,j++)
    {
      store_data_in_sflash[i]=buf[j];
    }
    
    CySysSFlashWriteUserRow(0,store_data_in_sflash);
}

void SFlash_read_lora_key(void)
{
    uint8_t read_data_in_sflash[CY_SFLASH_SIZEOF_USERROW]={0};
    uint8_t buf[16]={0};
    
    for (uint16_t i = 0u; i < CY_SFLASH_SIZEOF_USERROW; i++)
    {
      read_data_in_sflash[i]=(*((uint8_t *) (CY_SFLASH_USERBASE + i)));
      //PRINTF_RAW("%02X ",read_data_in_sflash[i]);
    }
    //PRINTF_RAW("\r");
        
    for(uint8_t i=0,j=0;i<8;i++,j++)
    {
      buf[j]=read_data_in_sflash[i];
    }
    lwan_dev_keys_set(DEV_KEYS_OTA_DEVEUI, buf);
    
    for(uint8_t i=8,j=0;i<16;i++,j++)
    {
      buf[j]=read_data_in_sflash[i];
    }
    lwan_dev_keys_set(DEV_KEYS_OTA_APPEUI, buf);
    
    for(uint8_t i=16,j=0;i<32;i++,j++)
    {
      buf[j]=read_data_in_sflash[i];
    }
    lwan_dev_keys_set(DEV_KEYS_OTA_APPKEY, buf);
    
    for(uint8_t i=32,j=0;i<36;i++,j++)
    {
      buf[j]=read_data_in_sflash[i];
    }
    uint32_t devaddr = buf[0] << 24 | buf[1] << 16 | buf[2] <<8 | buf[3];
    lwan_dev_keys_set(DEV_KEYS_ABP_DEVADDR, &devaddr);
    
    for(uint8_t i=36,j=0;i<52;i++,j++)
    {
      buf[j]=read_data_in_sflash[i];
    }
    lwan_dev_keys_set(DEV_KEYS_ABP_NWKSKEY, buf);
    
    for(uint8_t i=52,j=0;i<68;i++,j++)
    {
      buf[j]=read_data_in_sflash[i];
    }
    lwan_dev_keys_set(DEV_KEYS_ABP_APPSKEY, buf);
}

void SFlash_store_lora_config(void)
{  
    uint8_t store_data_in_sflash[CY_SFLASH_SIZEOF_USERROW]={0};
    
    store_data_in_sflash[0]=0x11;//FDR_status
    
    uint8_t join_mode=0;
    if(lwan_dev_config_get(DEV_CONFIG_JOIN_MODE, (void *)&join_mode)==LWAN_SUCCESS)
    store_data_in_sflash[1]=join_mode;

    uint8_t uldl_mode;
    lwan_dev_config_get(DEV_CONFIG_ULDL_MODE, &uldl_mode);
    store_data_in_sflash[2]=uldl_mode;
    
    uint8_t work_mode;
    lwan_dev_config_get(DEV_CONFIG_WORK_MODE, &work_mode);
    store_data_in_sflash[3]=work_mode;
    
    uint8_t class;
    lwan_dev_config_get(DEV_CONFIG_CLASS, &class);
    store_data_in_sflash[4]=class;
    
    ClassBParam_t classb_p;
    lwan_dev_config_get(DEV_CONFIG_CLASSB_PARAM, &classb_p);

    store_data_in_sflash[5]=classb_p.periodicity;
    store_data_in_sflash[6]=classb_p.beacon_dr;
    store_data_in_sflash[7]=classb_p.pslot_dr;
    store_data_in_sflash[8]=(classb_p.beacon_freq>>24)&0XFF;
    store_data_in_sflash[9]=(classb_p.beacon_freq>>16)&0XFF;
    store_data_in_sflash[10]=(classb_p.beacon_freq>>8)&0XFF;
    store_data_in_sflash[11]=(classb_p.beacon_freq)&0XFF;
    store_data_in_sflash[12]=(classb_p.pslot_freq>>24)&0XFF;
    store_data_in_sflash[13]=(classb_p.pslot_freq>>16)&0XFF;
    store_data_in_sflash[14]=(classb_p.pslot_freq>>8)&0XFF;
    store_data_in_sflash[15]=(classb_p.pslot_freq)&0XFF;
    
    JoinSettings_t join_set;
    lwan_dev_config_get(DEV_CONFIG_JOIN_SETTINGS, &join_set);
    store_data_in_sflash[16]=join_set.auto_join;
    store_data_in_sflash[17]=join_set.join_interval;
    store_data_in_sflash[18]=join_set.join_trials;
    store_data_in_sflash[19]=join_set.join_method;
    store_data_in_sflash[20]=join_set.stored_freqband;
    store_data_in_sflash[21]=join_set.stored_datarate;
    
    uint16_t freqband_mask;
    lwan_dev_config_get(DEV_CONFIG_FREQBAND_MASK, &freqband_mask);
    store_data_in_sflash[22]=(freqband_mask>>8)&0xFF;
    store_data_in_sflash[23]=freqband_mask&0xFF;
    
    uint8_t cfm;
    lwan_mac_config_get(MAC_CONFIG_CONFIRM_MSG, &cfm);
    store_data_in_sflash[24]=cfm;
    
    uint8_t reportMode;
    lwan_mac_config_get(MAC_CONFIG_REPORT_MODE, &reportMode);
    store_data_in_sflash[25]=reportMode;
    
    uint8_t checkValue;
    lwan_mac_config_get(MAC_CONFIG_CHECK_MODE, &checkValue);
    store_data_in_sflash[26]=checkValue;
    
    uint8_t adr;
    lwan_mac_config_get(MAC_CONFIG_ADR_ENABLE, &adr);
    store_data_in_sflash[27]=adr;
    
    uint8_t port;
    lwan_mac_config_get(MAC_CONFIG_APP_PORT, &port);
    store_data_in_sflash[28]=port;
    
    uint8_t datarate;
    lwan_mac_config_get(MAC_CONFIG_DATARATE, &datarate);
    store_data_in_sflash[29]=datarate;
    
    uint8_t tx_power;
    lwan_mac_config_get(MAC_CONFIG_TX_POWER, &tx_power);
    store_data_in_sflash[30]=tx_power;
    
    uint16_t rx1delay;
    lwan_mac_config_get(MAC_CONFIG_RX1_DELAY, &rx1delay);
    store_data_in_sflash[31]=(rx1delay>>8)&0xFF;
    store_data_in_sflash[32]=rx1delay&0xFF;
    
    RxParams_t rx_params;
    lwan_mac_config_get(MAC_CONFIG_RX_PARAM, &rx_params);
    store_data_in_sflash[33]=rx_params.rx1_dr_offset;
    store_data_in_sflash[34]=rx_params.rx2_dr;
    store_data_in_sflash[35]=(rx_params.rx2_freq>>24)&0xFF;
    store_data_in_sflash[36]=(rx_params.rx2_freq>>16)&0xFF;
    store_data_in_sflash[37]=(rx_params.rx2_freq>>8)&0xFF;
    store_data_in_sflash[38]=rx_params.rx2_freq&0xFF;
    
    uint16_t reportInterval;
    lwan_mac_config_get(MAC_CONFIG_REPORT_INTERVAL, &reportInterval);
    store_data_in_sflash[39]=(reportInterval>>8)&0xFF;
    store_data_in_sflash[40]=reportInterval&0xFF;
    
    uint8_t Nbtrials;
    lwan_mac_config_get(MAC_CONFIG_UNCONF_NBTRIALS, &Nbtrials);
    store_data_in_sflash[41]=Nbtrials;
    
    CySysSFlashWriteUserRow(1,store_data_in_sflash);
}

void SFlash_read_lora_config(void)
{
    uint8_t read_data_in_sflash[CY_SFLASH_SIZEOF_USERROW]={0};
    
    for (uint16_t i = 0u; i < CY_SFLASH_SIZEOF_USERROW; i++)
    {
      read_data_in_sflash[i]=(*((uint8_t *) (CYREG_SFLASH_MACRO_0_FREE_SFLASH256 + i)));
      //PRINTF_RAW("%02X ",read_data_in_sflash[i]);
    }
    //PRINTF_RAW("\r");
    
    //read_data_in_sflash[0];//FDR status
    
    uint8_t join_mode;
    join_mode=read_data_in_sflash[1];
    lwan_dev_config_set(DEV_CONFIG_JOIN_MODE, &join_mode);
    
    uint8_t uldl_mode;
    uldl_mode=read_data_in_sflash[2];
    lwan_dev_config_set(DEV_CONFIG_ULDL_MODE, &uldl_mode);
    
    uint8_t work_mode;
    work_mode=read_data_in_sflash[3];
    lwan_dev_config_set(DEV_CONFIG_WORK_MODE, &work_mode);
    
    uint8_t class;
    class=read_data_in_sflash[4];
    lwan_dev_config_set(DEV_CONFIG_CLASS, &class);
    
    ClassBParam_t classb_p;
    classb_p.periodicity=read_data_in_sflash[5];
    classb_p.beacon_dr=read_data_in_sflash[6];
    classb_p.pslot_dr=read_data_in_sflash[7];
    classb_p.beacon_freq=(uint32_t)((read_data_in_sflash[8]<<24)|(read_data_in_sflash[9]<<16)|(read_data_in_sflash[10]<<8)|read_data_in_sflash[11]);
    classb_p.pslot_freq=(uint32_t)((read_data_in_sflash[12]<<24)|(read_data_in_sflash[13]<<16)|(read_data_in_sflash[14]<<8)|read_data_in_sflash[15]);
    lwan_dev_config_set(DEV_CONFIG_CLASSB_PARAM, &classb_p);   
    
    JoinSettings_t join_set;
    join_set.auto_join=read_data_in_sflash[16];
    join_set.join_interval=read_data_in_sflash[17];
    join_set.join_trials=read_data_in_sflash[18];
    join_set.join_method=read_data_in_sflash[19];
    join_set.stored_freqband=read_data_in_sflash[20];
    join_set.stored_datarate=read_data_in_sflash[21];
    lwan_dev_config_set(DEV_CONFIG_JOIN_SETTINGS, &join_set);
    
    uint16_t freqband_mask;
    freqband_mask=(uint16_t)(read_data_in_sflash[22]<<8)|read_data_in_sflash[23];
    lwan_dev_config_set(DEV_CONFIG_FREQBAND_MASK, &freqband_mask);
    
    uint8_t cfm;
    cfm=read_data_in_sflash[24];
    lwan_mac_config_set(MAC_CONFIG_CONFIRM_MSG, &cfm);
    
    uint8_t reportMode;
    reportMode=read_data_in_sflash[25];
    lwan_mac_config_set(MAC_CONFIG_REPORT_MODE, &reportMode);
    
    uint8_t checkValue;
    checkValue=read_data_in_sflash[26];
    lwan_mac_config_set(MAC_CONFIG_CHECK_MODE, &checkValue);
    
    uint8_t adr;
    adr=read_data_in_sflash[27];
    lwan_mac_config_set(MAC_CONFIG_ADR_ENABLE, &adr);
        
    uint8_t port;
    port=read_data_in_sflash[28];
    lwan_mac_config_set(MAC_CONFIG_APP_PORT, &port);
    
    uint8_t datarate;
    datarate=read_data_in_sflash[29];
    lwan_mac_config_set(MAC_CONFIG_DATARATE, &datarate);
    
    uint8_t tx_power;
    tx_power=read_data_in_sflash[30];
    lwan_mac_config_set(MAC_CONFIG_TX_POWER, &tx_power);
    
    uint16_t rx1delay;
    rx1delay=(uint16_t)((read_data_in_sflash[31]<<8)|read_data_in_sflash[32]);
    lwan_mac_config_set(MAC_CONFIG_RX1_DELAY, &rx1delay);
    
    RxParams_t rx_params;
    rx_params.rx1_dr_offset=read_data_in_sflash[33];
    rx_params.rx2_dr=read_data_in_sflash[34];
    rx_params.rx2_freq=(uint32_t)((read_data_in_sflash[35]<<24)|(read_data_in_sflash[36]<<16)|(read_data_in_sflash[37]<<8)|(read_data_in_sflash[38]));
    lwan_mac_config_set(MAC_CONFIG_RX_PARAM, &rx_params);
    
    uint16_t reportInterval;
    reportInterval=(read_data_in_sflash[39]<<8)|read_data_in_sflash[40];
    lwan_mac_config_set(MAC_CONFIG_REPORT_INTERVAL, &reportInterval);   
    
    uint8_t Nbtrials;
    Nbtrials=read_data_in_sflash[41];
    lwan_mac_config_set(MAC_CONFIG_UNCONF_NBTRIALS, &Nbtrials);
}
