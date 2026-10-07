/**
 ****************************************************************************************
 *
 * @file appm_task.c
 *
 * @brief RW APP Task implementation
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"          // SW configuration

#if (BLE_APP_PRESENT)
#include <string.h>
#include "app_task.h"              // Application Manager Task API
#include "app.h"                      // Application Manager Definition
#include "gapc_task.h"            // GAP Controller Task API
#include "gapm_task.h"          // GAP Manager Task API
#include "gattc_task.h"
#include "arch.h"                    // Platform Definitions

#include "ke_timer.h"             // Kernel timer
#include "app_dis.h"              // Device Information Module Definition
#include "diss_task.h"
#include "app_batt.h"             // Battery Module Definition
#include "bass_task.h"
#include "app_oads.h"             
#include "oads_task.h"              
#include "gpio.h"
#include "audio.h"
#include "uart.h"
#include "BK3435_reg.h"
#include "icu.h"
#include "reg_ble_em_cs.h"
#include "lld.h"
#include "app_ancsc.h"
#include "ancsc_task.h"
#include "app_sec.h"              // Security Module Definition
#include "wdt.h"
#include "app_fff0.h"
#include "ZenoRtc.h"
#include "Watch.h"
#include "spi.h"
#include "SPI_Transfer.h"
#include "rc32k_cal.h"
#include "gapc_task.h"
#include "i2cs.h"
//static volatile RTC real_time;
extern void BLE_SendTimerEvent(void);
extern uint8_t SPI_RxBuf[SPI_CMD_LENG_MAX];

uint16_t recvCnt;
uint8_t NowConnIntvSlow;
extern uint8_t spi_bt2mcu;
extern uint8_t spi_mcu2bt;
extern uint8_t spi_cmd_RxBuf[SPI_CMD_LENG_MAX];
extern unsigned char    *p_spi_rxdata;
extern uint8_t packetLen;
extern uint8_t reDiscoverCnt;
extern uint8_t treatedNtfCnt;
extern uint8_t ancs_busy;
extern uint8_t ntfBufIndex;
extern uint32_t m_notification_buf[NOTI_BUF_MAX];
extern struct gapc_conn_param ConnParamFast;
extern struct gapc_conn_param ConnParamSlow;
extern uint8_t IsFastConnect;
uint16_t disconnectCnt;
extern uint8_t hrMeasureTick;
extern uint8_t app_sec_buff[sizeof(APP_SEC_DATA_T)];
extern hrs3300_results_t alg_results;
extern uint8_t onWrist;
extern volatile uint8_t feed3dSensor;
extern volatile uint16_t hrMeasureTime;
extern volatile uint8_t inHrMode;

extern unsigned char ToStopBluetooth;

extern uint8_t rtc_notify;
extern void SPI_InitMcu(void);
/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
 
 extern void heart_rate_meas_timeout_handler(void);

static uint8_t appm_get_handler(const struct ke_state_handler *handler_list,
                                ke_msg_id_t msgid,
                                void *param,
                                ke_task_id_t src_id)
{
    // Counter
    uint8_t counter;

    // Get the message handler function by parsing the message table
    for (counter = handler_list->msg_cnt; 0 < counter; counter--)
    {
			
        struct ke_msg_handler handler = (*(handler_list->msg_table + counter - 1));
			
        if ((handler.id == msgid) ||
            (handler.id == KE_MSG_DEFAULT_HANDLER))
        {
            // If handler is NULL, message should not have been received in this state
            ASSERT_ERR(handler.func);

            return (uint8_t)(handler.func(msgid, param, TASK_APP, src_id));
        }
    }

    // If we are here no handler has been found, drop the message
    return (KE_MSG_CONSUMED);
}

/*
 * MESSAGE HANDLERS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Handles ready indication from the GAP. - Reset the stack
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapm_device_ready_ind_handler(ke_msg_id_t const msgid,
                                         void const *param,
                                         ke_task_id_t const dest_id,
                                         ke_task_id_t const src_id)
{
    // Application has not been initialized
    ASSERT_ERR(ke_state_get(dest_id) == APPM_INIT);

    // Reset the stack
    struct gapm_reset_cmd* cmd = KE_MSG_ALLOC(GAPM_RESET_CMD,
                                              TASK_GAPM, TASK_APP,
                                              gapm_reset_cmd);

    cmd->operation = GAPM_RESET;

    ke_msg_send(cmd);

    return (KE_MSG_CONSUMED);
}


/**
 ****************************************************************************************
 * @brief Handles GAP manager command complete events.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapm_cmp_evt_handler(ke_msg_id_t const msgid,
                                struct gapm_cmp_evt const *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id)
{
	UART_PRINTF("param->operation = 0x%x, param->status = 0x%x \r\n", param->operation, param->status);
	switch(param->operation)
	{
		// Reset completed
	case (GAPM_RESET):
	{
		if(param->status == GAP_ERR_NO_ERROR)
		{
			WatchIniti();
			// Set Device configuration
			struct gapm_set_dev_config_cmd* cmd = KE_MSG_ALLOC(GAPM_SET_DEV_CONFIG_CMD,
			                                      TASK_GAPM, TASK_APP,
			                                      gapm_set_dev_config_cmd);
			// Set the operation
			cmd->operation = GAPM_SET_DEV_CONFIG;
			// Set the device role - Peripheral
			cmd->role      = GAP_ROLE_PERIPHERAL;
			// Set Data length parameters
			cmd->sugg_max_tx_octets = BLE_MIN_OCTETS;
			cmd->sugg_max_tx_time   = BLE_MIN_TIME;

			cmd->max_mtu = 131;//BLE_MIN_OCTETS;
			//Do not support secure connections
			cmd->pairing_mode = GAPM_PAIRING_LEGACY;

			//cmd->addr_type   = GAPM_CFG_ADDR_HOST_PRIVACY; //2017-10-24 by alen
			// load IRK
			memcpy(cmd->irk.key, app_env.loc_irk, KEY_LEN);

			app_env.next_svc = 0;

			// Send message
			ke_msg_send(cmd);
			
		}
		else
		{
			ASSERT_ERR(0);
		}
	}
	break;
	case (GAPM_PROFILE_TASK_ADD):
	{
		// Add the next requested service
		if (!appm_add_svc())
		{
			// Go to the ready state
			ke_state_set(TASK_APP, APPM_READY);
			
			//#ifndef	POWERON_SYNC
			//appm_start_advertising();
			//#endif
			//ke_timer_set(APP_POWERON_SYNC_TIMER, TASK_APP,80);
		}
	}
	break;
	// Device Configuration updated
	case (GAPM_SET_DEV_CONFIG):
	{
		ASSERT_INFO(param->status == GAP_ERR_NO_ERROR, param->operation, param->status);

		// Go to the create db state
		ke_state_set(TASK_APP, APPM_CREATE_DB);

		// Add the first required service in the database
		// and wait for the PROFILE_ADDED_IND
		appm_add_svc();
	}
	break;

	case (GAPM_ADV_NON_CONN):
	case (GAPM_ADV_UNDIRECT):
	case (GAPM_ADV_DIRECT):
	case (GAPM_UPDATE_ADVERTISE_DATA):
	case (GAPM_ADV_DIRECT_LDC):
	{
		if (param->status == GAP_ERR_TIMEOUT)
		{
			ke_state_set(TASK_APP, APPM_READY);

			//device not bonded, start general adv
			appm_start_advertising();
		}
		else if(param->status == GAP_ERR_CANCELED)
		{
			Watch_BleStateChangeEvt(BT_IDLE);
		}
	}
	break;

	default:
	{
		// Drop the message
	}
	break;
	}

    return (KE_MSG_CONSUMED);
}

static int gapc_get_dev_info_req_ind_handler(ke_msg_id_t const msgid,
        struct gapc_get_dev_info_req_ind const *param,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{
    switch(param->req)
    {
        case GAPC_DEV_NAME:
        {
            struct gapc_get_dev_info_cfm * cfm = KE_MSG_ALLOC_DYN(GAPC_GET_DEV_INFO_CFM,
                                                    src_id, dest_id,
                                                    gapc_get_dev_info_cfm, APP_DEVICE_NAME_MAX_LEN);
            cfm->req = param->req;
            cfm->info.name.length = appm_get_dev_name(cfm->info.name.value);

            // Send message
            ke_msg_send(cfm);
        } break;

        case GAPC_DEV_APPEARANCE:
        {
            // Allocate message
            struct gapc_get_dev_info_cfm *cfm = KE_MSG_ALLOC(GAPC_GET_DEV_INFO_CFM,
                                                    src_id, dest_id,
                                                    gapc_get_dev_info_cfm);
            cfm->req = param->req;
            
            // No appearance
            cfm->info.appearance = 0;

            // Send message
            ke_msg_send(cfm);
        } break;

	case GAPC_DEV_SLV_PREF_PARAMS:
	{
		// Allocate message
		struct gapc_get_dev_info_cfm *cfm = KE_MSG_ALLOC(GAPC_GET_DEV_INFO_CFM,
		                                    src_id, dest_id,
		                                    gapc_get_dev_info_cfm);
		cfm->req = param->req;
		// Slave preferred Connection interval Min
		cfm->info.slv_params.con_intv_min = 24;
		// Slave preferred Connection interval Max
		cfm->info.slv_params.con_intv_max = 40;
		// Slave preferred Connection latency
		cfm->info.slv_params.slave_latency  = 19;
		// Slave preferred Link supervision timeout
		cfm->info.slv_params.conn_timeout  = 600;  // 6s (600*10ms)

            // Send message
            ke_msg_send(cfm);
        } break;

        default: /* Do Nothing */
			break;
    }


    return (KE_MSG_CONSUMED);
}
/**
 ****************************************************************************************
 * @brief Handles GAPC_SET_DEV_INFO_REQ_IND message.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapc_set_dev_info_req_ind_handler(ke_msg_id_t const msgid,
        struct gapc_set_dev_info_req_ind const *param,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{
	// Set Device configuration
	struct gapc_set_dev_info_cfm* cfm = KE_MSG_ALLOC(GAPC_SET_DEV_INFO_CFM, src_id, dest_id,
                                                 gapc_set_dev_info_cfm);
	// Reject to change parameters
	cfm->status = GAP_ERR_REJECTED;
	cfm->req = param->req;
	// Send message
	ke_msg_send(cfm);

	return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles connection complete event from the GAP. Enable all required profiles
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapc_connection_req_ind_handler(ke_msg_id_t const msgid,
                                           struct gapc_connection_req_ind const *param,
                                           ke_task_id_t const dest_id,
                                           ke_task_id_t const src_id)
{	
	UART_PRINTF("%s\r\n", __func__);
	
    app_env.conidx = KE_IDX_GET(src_id);
    // Check if the received Connection Handle was valid
    if (app_env.conidx != GAP_INVALID_CONIDX)
    {
        // Retrieve the connection info from the parameters
        app_env.conhdl = param->conhdl;

        // Send connection confirmation
        struct gapc_connection_cfm *cfm = KE_MSG_ALLOC(GAPC_CONNECTION_CFM,
                KE_BUILD_ID(TASK_GAPC, app_env.conidx), TASK_APP,
                gapc_connection_cfm);

		//cfm->auth = app_sec_get_bond_status() ? GAP_AUTH_REQ_NO_MITM_BOND : GAP_AUTH_REQ_NO_MITM_NO_BOND;
		APP_SEC_DATA_P app_sec_para = (APP_SEC_DATA_P)app_sec_buff;
		cfm->auth = app_sec_para->auth;
		// Send the message
		ke_msg_send(cfm);

		/*--------------------------------------------------------------
		 * ENABLE REQUIRED PROFILES
		 *--------------------------------------------------------------*/

		// Enable Battery Service
		//app_batt_enable_prf(app_env.conhdl);

		// We are now in connected State
		ke_state_set(dest_id, APPM_CONNECTED);
		
		Watch_BleStateChangeEvt(BT_CONNECTED);
		reDiscoverCnt = 0;

		app_sec_env.bonded = false;
		app_sec_env.peer_pairing = false;
		app_sec_env.peer_encrypt = false;
		ke_timer_clear(APP_ADV_TO_TIMER,TASK_APP);
		//ke_timer_set(APP_SEND_SECURITY_REQ,TASK_APP,150);

	    ke_timer_set(APP_ANCS_REQ_IND,TASK_APP,100); //wait for the ios to prepare the ancs service
		ke_timer_set(APP_CONNECTED_TIMER,TASK_APP,100);

#if UPDATE_CONNENCT_PARAM
		ke_timer_set(APP_PARAM_UPDATE_REQ_IND,TASK_APP,100);
#endif

	}
	else
	{
		// No connection has been establish, restart advertising
		appm_start_advertising();
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles GAP controller command complete events.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapc_cmp_evt_handler(ke_msg_id_t const msgid,
                                struct gapc_cmp_evt const *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id)
{
	UART_PRINTF("gapc_cmp_evt_handler operation = %x\r\n",param->operation);
	switch(param->operation)
	{
	case (GAPC_UPDATE_PARAMS):  //0x09
	{
		if (param->status != GAP_ERR_NO_ERROR)
		{
			UART_PRINTF("gapc update params fail !\r\n");
		}
		else
		{
			UART_PRINTF("gapc update params ok !\r\n");
		}

	}
	break;
	case (GAPC_DISCONNECT): //0x01
	{
		if(param->status == GAP_ERR_NO_ERROR)
		{
			UART_PRINTF("pairing_fail = 0x%x\r\n",app_sec_env.pairing_fail);
			//bonding info lost and pairing fail
			if(app_sec_env.pairing_fail)
			{
				app_sec_env.pairing_fail = false;

				UART_PRINTF("restart advertising\r\n");

				ke_state_set(TASK_APP, APPM_READY);

				//restart advertising
				//appm_start_advertising();
			}
		}
	}
	break;
	case (GAPC_SECURITY_REQ): //0x0c
	{
		if (param->status != GAP_ERR_NO_ERROR)
		{
			UART_PRINTF("gapc security req fail !\r\n");
		}
		else
		{
			UART_PRINTF("gapc security req ok !\r\n");
		}
	}
	break;
	case (GAPC_BOND): // 0xa
	{
		if (param->status != GAP_ERR_NO_ERROR)
		{
			UART_PRINTF("gapc bond fail !\r\n");
		}
		else
		{
			UART_PRINTF("gapc bond ok !\r\n");
			app_ancsc_enable_prf(app_env.conhdl);
		}
	}break;

	case (GAPC_ENCRYPT): // 0xb
	{
		if (param->status != GAP_ERR_NO_ERROR)
		{
			UART_PRINTF("gapc encrypt start fail !\r\n");
		}
		else
		{
			UART_PRINTF("gapc encrypt start ok !\r\n");
			app_ancsc_enable_prf(app_env.conhdl);
		}
	}
	break;


	default:
		break;
	}

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles disconnection complete event from the GAP.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapc_disconnect_ind_handler(ke_msg_id_t const msgid,
                                      struct gapc_disconnect_ind const *param,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id)
{
	UART_PRINTF("disconnect link reason = 0x%x\r\n",param->reason);
	disconnectCnt++;
	// Go to the ready state
	ke_state_set(TASK_APP, APPM_READY);

	wdt_disable_flag = 1;
	
	// Restart Advertising
	if(ToStopBluetooth == 0)
		appm_start_advertising();
	else
	{
		ToStopBluetooth = 0;
		//spi_setBtState(BT_IDLE);
		Watch_BleStateChangeEvt(BT_IDLE);
	}
	
	treatedNtfCnt = 0;
	ntfBufIndex = 0;
	ancs_busy = 0;

	return (KE_MSG_CONSUMED);
}


/**
 ****************************************************************************************
 * @brief Handles profile add indication from the GAP.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapm_profile_added_ind_handler(ke_msg_id_t const msgid,
                                          struct gapm_profile_added_ind *param,
                                          ke_task_id_t const dest_id,
                                          ke_task_id_t const src_id)
{
    // Current State
    uint8_t state = ke_state_get(dest_id);

    if (state == APPM_CREATE_DB)
    {
        switch (param->prf_task_id)
        {
            default: 
			break;
        }
    }
    else
    {
        ASSERT_INFO(0, state, src_id);
    }

    return KE_MSG_CONSUMED;
}
#if (RC_CALIBRATE)
static int app_calibrate_timer_handler(ke_msg_id_t const msgid,
                                    void *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id)
{
    
    uint32_t timer_625us,timer_1us,timer_ms;
	
	if(getCurBleState() != BT_IDLE)
	{
    lld_evt_time_get_us(&timer_625us,&timer_1us);
    //timer_625us这个值最大为0x07FFFFFF，超过又从0开始
    timer_ms=((uint64_t)timer_625us*625+timer_1us)/1000;

    //UART_PRINTF("%d\r\n", timer_ms);        
    user_timer_init();


    ke_timer_set(APP_RC_CALIBRATE_TIMER,TASK_APP,30000);
		
	}
	return KE_MSG_CONSUMED;
}
#endif  

/*******************************************************************************
 * Function: app_period_timer_handler
 * Description: app period timer process
 * Input: msgid -Id of the message received.
 *		  param -Pointer to the parameters of the message.
 *		  dest_id -ID of the receiving task instance (TASK_GAP).
 *		  ID of the sending task instance.
 * Return: If the message was consumed or not.
 * Others: void
*******************************************************************************/
static int app_period_timer_handler(ke_msg_id_t const msgid,
                                          void *param,
                                          ke_task_id_t const dest_id,
                                          ke_task_id_t const src_id)
{
	//RTC rtc;
	//for(uint8_t i = 0; i< 10;i++)
	//UART_PRINTF("%x,%x,%x,%x,%x,%x,%x \r\n",test_buf[i*7+0],test_buf[i*7+1],test_buf[i*7+2],test_buf[i*7+3],test_buf[i*7+4],test_buf[i*7+5],test_buf[i*7+6]);
	ke_timer_set(APP_PERIOD_TIMER, TASK_APP, 100);
	//spi_setBtVersion();
	SPI_InitMcu();
	//spi_bt2mcu = 1;
	//spi_slave_init();
	//SPI_Transfer_IRQ_On();
	
	//TH_AppTimerRTC(NULL);
	//Watch_RtcUpdateEvt(rtc_notify);
	//RTC_GetTime(&rtc);
	//UART_PRINTF("CurTime:%x,%x,%x\r\n",rtc.hour,rtc.min,rtc.sec);
	
	return KE_MSG_CONSUMED;
}
static int app_send_timer_handler(ke_msg_id_t const msgid,
                                    void *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id)
{
	//UART_PRINTF("%s\r\n", __func__);
	
	BLE_SendTimerEvent();
	return KE_MSG_CONSUMED;
}

static int app_adv_to_timer_handler(ke_msg_id_t const msgid,
                                    void *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id)
{
	//UART_PRINTF("%s\r\n", __func__);
	
	appm_stop_advertising();
	
	return KE_MSG_CONSUMED;
}

/*static int app_spi_cmd_transfered_handler(ke_msg_id_t const msgid,
                                    void *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id)*/



static int app_spi_resend_handler(ke_msg_id_t const msgid,
                                    void *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id)
{

	SPI_TransferTask();

	UART_PRINTF("timeout\r\n");
	return KE_MSG_CONSUMED;								
}

static int app_hr_40ms_handler(ke_msg_id_t const msgid,
                                    void *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id)
{

	if(hrMeasureing){
		ke_timer_set(APP_HR_TIMER, TASK_APP, 4);
		heart_rate_meas_timeout_handler();
		//ke_timer_set(APP_HR_TIMER, TASK_APP, 4);
	}
	return KE_MSG_CONSUMED;								
}

static int app_1s_timer_handler(ke_msg_id_t const msgid,
                                    void *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id)
{
	if(inHrMode)
	{
		if(hrMeasureing == HR_MEASURE_IDLE)
		{
			HR_StartMeasurement(HR_MEASURE_MANUAL);
			ke_timer_set(APP_1S_TIMER,TASK_APP,100);
		}
		else{
			feed3dSensor = 1;
			hrMeasureTime++;
			if(hrMeasureTime >=50)
			{
				hrMeasureTime = 0;
				inHrMode = 0;
				HR_StopMeasurement(HR_MEASURE_MANUAL);
			}
			else{
				ke_timer_set(APP_1S_TIMER,TASK_APP,100);
			}
		}
	}
	/*else{
		if(hrMeasureing != HR_MEASURE_IDLE)
		{
			HR_StopMeasurement(HR_MEASURE_MANUAL);
		}
	}*/
	return KE_MSG_CONSUMED;								
}



static int app_connected_timer_handler(ke_msg_id_t const msgid,
                                    void *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id)
{
	Watch_RtcUpdateEvt();
	if(getCurBleState() == BT_CONNECTED)
	{
		ke_timer_set(APP_CONNECTED_TIMER,TASK_APP,100);
	}
	return KE_MSG_CONSUMED;								
}


static int app_ancs_to_handler(ke_msg_id_t const msgid,
                                    void *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id)
{
	UART_PRINTF("ancs timeout\r\n");
	treatedNtfCnt++;
	if(treatedNtfCnt < ntfBufIndex && getCurBleState() == BT_CONNECTED)
	{
		app_ancsc_get_Socialmsg(m_notification_buf[treatedNtfCnt]);
		ke_timer_set(APP_ANCS_TO_TIMER,TASK_APP,100);
		ancs_busy = 1;
	}
	else
	{
		treatedNtfCnt = 0;
		ntfBufIndex = 0;
		ancs_busy = 0;
	}
	return KE_MSG_CONSUMED;								
}

static int app_poweron_sync_handler(ke_msg_id_t const msgid,
                                    void *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id)
{
	if(gpio_get_input(SPI_IRQ_PIN) && (gpio_get_input(GPIO_SPI_NSS_PIN)))
	{
		UART_PRINTF("poweron sync ok\r\n");
		SPI_Transfer_init();
		SPI_InitMcu();
		appm_start_advertising();
	}
	else{
		UART_PRINTF("poweron sync error\r\n");
		ke_timer_set(APP_POWERON_SYNC_TIMER,TASK_APP,30);
	}
	return KE_MSG_CONSUMED;	
}

static int app_hr_test_to_handler(ke_msg_id_t const msgid,
                                    void *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id)
{
	extern bool hrs3300_power_up_flg;
	if(hrs3300_power_up_flg)
	{
		hrs3300_power_up_flg = 0;
		Hrs3300_chip_disable();
	#ifdef I2C_HARDWARE
		i2c_deinit();
	#else
		i2cs_deinit();
	#endif
	}
	return KE_MSG_CONSUMED;										
}


/**
 ****************************************************************************************
 * @brief Handles reception of all messages sent from the lower layers to the application
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int appm_msg_handler(ke_msg_id_t const msgid,
                            void *param,
                            ke_task_id_t const dest_id,
                            ke_task_id_t const src_id)
{
    // Retrieve identifier of the task from received message
    ke_task_id_t src_task_id = MSG_T(msgid);
    // Message policy
    uint8_t msg_pol          = KE_MSG_CONSUMED;


    switch (src_task_id)
    {
        case (TASK_ID_GAPC):
        {
            if ((msgid >= GAPC_BOND_CMD) &&
                (msgid <= GAPC_SECURITY_IND))
            {
                // Call the Security Module
                msg_pol = appm_get_handler(&app_sec_table_handler, msgid, param, src_id);
            }
        } break;

        case (TASK_ID_GATTC):
        {
            // Service Changed - Drop
        } break;

	case (TASK_ID_ANCSC):
	{
		// Call the Health Thermometer Module
		msg_pol = appm_get_handler(&app_ancsc_table_handler, msgid, param, src_id);
	}
	break;

	case (TASK_ID_FFF0S):
	{
		// Call the Health Thermometer Module
		msg_pol = appm_get_handler(&app_fff0_table_handler, msgid, param, src_id);
	}
	break;
	
#if 0 //bros
	case (TASK_ID_DISS):
	{
		// Call the Device Information Module
		msg_pol = appm_get_handler(&app_dis_table_handler, msgid, param, src_id);
	}
	break;

        case (TASK_ID_BASS):
        {
            // Call the Battery Module
            msg_pol = appm_get_handler(&app_batt_table_handler, msgid, param, src_id);
        } break;
		
        case (TASK_ID_OADS):
        {
            // Call the Health Thermometer Module
            msg_pol = appm_get_handler(&app_oads_table_handler, msgid, param, src_id);
        } break;
#endif
        default:
        {
        } break;
    }

    return (msg_pol);
}


/*******************************************************************************
 * Function: gapc_update_conn_param_req_ind_handler
 * Description: Update request command processing from slaver connection parameters
 * Input: msgid   -Id of the message received.
 *		  param   -Pointer to the parameters of the message.
 *		  dest_id -ID of the receiving task instance
 *		  src_id  -ID of the sending task instance.
 * Return: If the message was consumed or not.
 * Others: void
*******************************************************************************/
static int gapc_update_conn_param_req_ind_handler (ke_msg_id_t const msgid, 
									const struct gapc_param_update_req_ind  *param,
                 					ke_task_id_t const dest_id, 
                 					ke_task_id_t const src_id)
{

	UART_PRINTF("slave send param_update_req\r\n");
	struct gapc_conn_param  up_param;
	
	up_param.intv_min   = BLE_UAPDATA_MIN_INTVALUE;
	up_param.intv_max   = BLE_UAPDATA_MAX_INTVALUE; 
	up_param.latency    = BLE_UAPDATA_LATENCY;  
	up_param.time_out   = BLE_UAPDATA_TIMEOUT; 
	
	appm_update_param(&up_param);
	
	return KE_MSG_CONSUMED;
}

 
/*******************************************************************************
 * Function: gapc_le_pkt_size_ind_handler
 * Description: GAPC_LE_PKT_SIZE_IND
 * Input: msgid   -Id of the message received.
 *		  param   -Pointer to the parameters of the message.
 *		  dest_id -ID of the receiving task instance
 *		  src_id  -ID of the sending task instance.
 * Return: If the message was consumed or not.
 * Others: void
*******************************************************************************/
static int gapc_le_pkt_size_ind_handler (ke_msg_id_t const msgid, 
									const struct gapc_le_pkt_size_ind  *param,
                 					ke_task_id_t const dest_id, 
                 					ke_task_id_t const src_id)
{
   	UART_PRINTF("%s \r\n", __func__);
	UART_PRINTF("1max_rx_octets = %d\r\n",param->max_rx_octets);
	UART_PRINTF("1max_rx_time = %d\r\n",param->max_rx_time);
	UART_PRINTF("1max_tx_octets = %d\r\n",param->max_tx_octets);
	UART_PRINTF("1max_tx_time = %d\r\n",param->max_tx_time);
	
	return KE_MSG_CONSUMED;
}

/**
 ****************************************************************************************
 * @brief  GAPC_PARAM_UPDATED_IND
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapc_param_updated_ind_handler (ke_msg_id_t const msgid, 
										const struct gapc_param_updated_ind  *param,
                 						ke_task_id_t const dest_id, 
                 						ke_task_id_t const src_id)
{
    UART_PRINTF("%s \r\n", __func__);
	UART_PRINTF("con_interval = %d\r\n",param->con_interval);
	UART_PRINTF("con_latency = %d\r\n",param->con_latency);
	UART_PRINTF("sup_to = %d\r\n",param->sup_to);
	
	if(param->con_interval>= ConnParamSlow.intv_min)
	{
		NowConnIntvSlow = 1;
	}
	else
	{
		NowConnIntvSlow = 0;
	}
	UART_PRINTF("NowConnIntvSlow %x\r\n",NowConnIntvSlow);
	UART_PRINTF("IsFastConnect %x\r\n",IsFastConnect);

	return KE_MSG_CONSUMED;
}


/**
 ****************************************************************************************
 * @brief  APP_SEND_SECURITY_REQ
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
 extern unsigned char ancs_flag;
static int gapc_send_security_req_handler(ke_msg_id_t const msgid, 
										void const *param,
        								ke_task_id_t const dest_id, 
        								ke_task_id_t const src_id)
{
	if(ancs_flag == 1)
	{
		ancs_flag = 0;
    	appm_send_seurity_req();
	}
	
    return KE_MSG_CONSUMED;
}


/**
 ****************************************************************************************
 * @brief  GATTC_MTU_CHANGED_IND
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gattc_mtu_changed_ind_handler(ke_msg_id_t const msgid,
                                     struct gattc_mtu_changed_ind const *ind,
                                     ke_task_id_t const dest_id,
                                     ke_task_id_t const src_id)
{
	UART_PRINTF("%s \r\n",__func__);
	UART_PRINTF("ind->mtu = %d,seq = %d\r\n",ind->mtu,ind->seq_num);
	
 	return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief   GAPC_PARAM_UPDATE_REQ_IND
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapc_param_update_req_ind_handler(ke_msg_id_t const msgid,
                                struct gapc_param_update_req_ind const *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id)
{
	UART_PRINTF("%s \r\n", __func__);
	// Prepare the GAPC_PARAM_UPDATE_CFM message
    struct gapc_param_update_cfm *cfm = KE_MSG_ALLOC(GAPC_PARAM_UPDATE_CFM,
                                             src_id, dest_id,
                                             gapc_param_update_cfm);
	 
	cfm->ce_len_max = 0xffff;
	cfm->ce_len_min = 0xffff;
	cfm->accept = true; 

	// Send message
    ke_msg_send(cfm);
	 
	return (KE_MSG_CONSUMED);
}

static int app_ancs_req_handler(ke_msg_id_t const msgid, void const *param,
        ke_task_id_t const dest_id, ke_task_id_t const src_id)
{

    //ke_timer_set(APP_LOOP_EVENT_IND,TASK_APP, 3);
    app_ancsc_enable_prf(app_env.conhdl);
    return KE_MSG_CONSUMED;
}



/*
 * GLOBAL VARIABLES DEFINITION
 ****************************************************************************************
 */

/* Default State handlers definition. */
const struct ke_msg_handler appm_default_state[] =
{
    // Note: first message is latest message checked by kernel so default is put on top.
    {KE_MSG_DEFAULT_HANDLER,    	(ke_msg_func_t)appm_msg_handler},
    {GAPM_DEVICE_READY_IND,     	(ke_msg_func_t)gapm_device_ready_ind_handler},
    {GAPM_CMP_EVT,             		(ke_msg_func_t)gapm_cmp_evt_handler},
    {GAPC_GET_DEV_INFO_REQ_IND, 	(ke_msg_func_t)gapc_get_dev_info_req_ind_handler},
    {GAPC_SET_DEV_INFO_REQ_IND, 	(ke_msg_func_t)gapc_set_dev_info_req_ind_handler},
    {GAPC_CONNECTION_REQ_IND,   	(ke_msg_func_t)gapc_connection_req_ind_handler},
    {GAPC_CMP_EVT,             		(ke_msg_func_t)gapc_cmp_evt_handler},
    {GAPC_DISCONNECT_IND,       	(ke_msg_func_t)gapc_disconnect_ind_handler},
    {GAPM_PROFILE_ADDED_IND,    	(ke_msg_func_t)gapm_profile_added_ind_handler},
    {GAPC_LE_PKT_SIZE_IND,			(ke_msg_func_t)gapc_le_pkt_size_ind_handler},
    {GAPC_PARAM_UPDATED_IND,		(ke_msg_func_t)gapc_param_updated_ind_handler},
    {APP_SEND_SECURITY_REQ,     	(ke_msg_func_t)gapc_send_security_req_handler},
    {GATTC_MTU_CHANGED_IND,			(ke_msg_func_t)gattc_mtu_changed_ind_handler},	
    {GAPC_PARAM_UPDATE_REQ_IND, 	(ke_msg_func_t)gapc_param_update_req_ind_handler},
    {APP_PARAM_UPDATE_REQ_IND, 		(ke_msg_func_t)gapc_update_conn_param_req_ind_handler},
    {APP_ANCS_REQ_IND,				(ke_msg_func_t)app_ancs_req_handler},
	#if RC_CALIBRATE
	{APP_RC_CALIBRATE_TIMER,		(ke_msg_func_t)app_calibrate_timer_handler},
	#endif
	{APP_PERIOD_TIMER,				(ke_msg_func_t)app_period_timer_handler},
	{APP_BLE_SEND_TIMER,			(ke_msg_func_t)app_send_timer_handler},
	{APP_ADV_TO_TIMER,				(ke_msg_func_t)app_adv_to_timer_handler},
	//{APP_SPI_CMD_RCVED,				(ke_msg_func_t)app_spi_cmd_rcved_handler},
//	{APP_SPI_PACKET_RCVED,			(ke_msg_func_t)app_spi_cmd_transfered_handler},
//	{APP_SPI_MCU_IRQ,				(ke_msg_func_t)app_spi_mcu_irq_handler},
	{APP_SPI_TIMER,					(ke_msg_func_t)app_spi_resend_handler},
	{APP_HR_TIMER,					(ke_msg_func_t)app_hr_40ms_handler},
	{APP_1S_TIMER,					(ke_msg_func_t)app_1s_timer_handler},
	{APP_CONNECTED_TIMER,			(ke_msg_func_t)app_connected_timer_handler},
	{APP_ANCS_TO_TIMER,				(ke_msg_func_t)app_ancs_to_handler},
	{APP_POWERON_SYNC_TIMER,		(ke_msg_func_t)app_poweron_sync_handler},
	{APP_HR_TEST_TO_TIMER,			(ke_msg_func_t)app_hr_test_to_handler},
};

/* Specifies the message handlers that are common to all states. */
const struct ke_state_handler appm_default_handler = KE_STATE_HANDLER(appm_default_state);

/* Defines the place holder for the states of all the task instances. */
ke_state_t appm_state[APP_IDX_MAX];

#endif //(BLE_APP_PRESENT)

/// @} APPTASK
