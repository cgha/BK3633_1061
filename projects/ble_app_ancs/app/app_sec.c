/**
 ****************************************************************************************
 *
 * @file app_sec.c
 *
 * @brief Application Security Entry Point
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup APP
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"


#include <string.h>
#include "co_math.h"
#include "gapc_task.h"      // GAP Controller Task API Definition
#include "gap.h"            // GAP Definition
#include "gapc.h"           // GAPC Definition
#include "prf_types.h"
#include "llc_llcp.h"

#include "app.h"            // Application API Definition
#include "app_sec.h"        // Application Security API Definition
#include "app_task.h"       // Application Manager API Definitionde 
#include "ke_timer.h"
#if (NVDS_SUPPORT)
#include "nvds.h"           // NVDS API Definitions
#else
#include "flash.h"
#endif
#include "co_utils.h"
#include "uart.h"
#include "app_bond_db.h"
/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */


uint8_t app_sec_buff[sizeof(APP_SEC_DATA_T)];

/// Application Security Environment Structure
struct app_sec_env_tag app_sec_env;



bool app_get_bonding_flag(void);

bool app_sec_get_enc_flag(void);


/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

void app_sec_init()
{
	/*uint8_t length = NVDS_LEN_PERIPH_BONDED;
	if (nvds_get(NVDS_TAG_PERIPH_BONDED, &length, (uint8_t *)&app_sec_env.bonded) != NVDS_OK)
	{
		// If read value is invalid, set status to not bonded
		if ((app_sec_env.bonded != true) && (app_sec_env.bonded != false))
		{
			app_sec_env.bonded = false;
		}
	}*/
	app_sec_env.peer_pairing = false;
	app_sec_env.peer_encrypt = false;	
	app_sec_env.bond_lost = false;
	app_sec_env.pairing_fail = false;
}

bool app_sec_get_bond_status(void)
{
    return app_sec_env.bonded;
}


bool app_get_bonding_flag(void)
{
	UART_PRINTF("peer pairing = %d\r\n",app_sec_env.peer_pairing);
	return app_sec_env.peer_pairing;
}


bool app_get_encrypt_flag(void)
{
	UART_PRINTF("peer encrypt = %d\r\n",app_sec_env.peer_encrypt);
	return app_sec_env.peer_encrypt;
}

#if 0
void app_sec_remove_bond(void)
{
	// Check if we are well bonded
	if (app_sec_env.bonded == true)
	{
		// Update the environment variable
		app_sec_env.bonded = false;

#if (NVDS_SUPPORT)
		if (nvds_put(NVDS_TAG_PERIPH_BONDED, NVDS_LEN_PERIPH_BONDED,
		             (uint8_t *)&app_sec_env.bonded) != NVDS_OK)
		{
			ASSERT_ERR(0);
		}
#else
		/*
		flash_read(FLASH_SPACE_TYPE_NVR,0x8000,sizeof(APP_SEC_DATA_T),  (uint8_t *)app_sec_para);
		app_sec_para->bond = 0x0;
		flash_erase_sector(FLASH_SPACE_TYPE_NVR, 0x8000);
		flash_write(FLASH_SPACE_TYPE_NVR, 0x8000, sizeof(APP_SEC_DATA_T),  (uint8_t *)app_sec_para);
		*/
		APP_SEC_DATA_P app_sec_para = (APP_SEC_DATA_P)app_sec_buff;
		flash_erase_sector(FLASH_SPACE_TYPE_NVR, 0x8000);
//		app_sec_para->bond = 0x0;
		memset(&app_sec_para->ltk, 0x0, sizeof( struct gapc_ltk));
		flash_write(FLASH_SPACE_TYPE_NVR, 0x8000, sizeof(APP_SEC_DATA_T), app_sec_buff);
#endif
	}
	app_sec_env.bonded = false;
	app_sec_env.peer_pairing = false;
	app_sec_env.peer_encrypt = false;
}
#endif

void app_sec_send_security_req(uint8_t conidx)
{
	if(!app_get_encrypt_flag() && !app_get_bonding_flag())
	{
		UART_PRINTF("app_sec_send_security_req\r\n");
		
		// Send security request
	    struct gapc_security_cmd *cmd = KE_MSG_ALLOC(GAPC_SECURITY_CMD,
	                                                 KE_BUILD_ID(TASK_GAPC, conidx), TASK_APP,
	                                                 gapc_security_cmd);

	    cmd->operation = GAPC_SECURITY_REQ;
	    cmd->auth      = GAP_AUTH_REQ_NO_MITM_BOND;
	    // Send the message
	    ke_msg_send(cmd);
	}
	else
	{
		ke_timer_clear(APP_SEND_SECURITY_REQ,TASK_APP); 
	}
}

/*
 * MESSAGE HANDLERS
 ****************************************************************************************
 */

static int gapc_bond_req_ind_handler(ke_msg_id_t const msgid,
                                     struct gapc_bond_req_ind const *param,
                                     ke_task_id_t const dest_id,
                                     ke_task_id_t const src_id)
{
	UART_PRINTF("%s \r\n",__func__);
    // Prepare the GAPC_BOND_CFM message
    struct gapc_bond_cfm *cfm = KE_MSG_ALLOC(GAPC_BOND_CFM,
                                             src_id, TASK_APP,
                                             gapc_bond_cfm);

    switch (param->request)
    {
        case (GAPC_PAIRING_REQ):
        {
	    	UART_PRINTF("gapc_pairing req\r\n");
	    	app_sec_env.peer_pairing = true;
            cfm->request = GAPC_PAIRING_RSP;

            cfm->accept  = false;

            // Check if we are already bonded (Only one bonded connection is supported)
            if (!app_sec_env.bonded)
            {
                cfm->accept  = true;

                // Pairing Features
				cfm->data.pairing_feat.auth      = GAP_AUTH_REQ_NO_MITM_BOND;
                cfm->data.pairing_feat.iocap     = GAP_IO_CAP_NO_INPUT_NO_OUTPUT;
                cfm->data.pairing_feat.key_size  = 16;
                cfm->data.pairing_feat.oob       = GAP_OOB_AUTH_DATA_NOT_PRESENT;
                cfm->data.pairing_feat.sec_req   = GAP_NO_SEC;
                cfm->data.pairing_feat.ikey_dist = GAP_KDIST_ENCKEY| GAP_KDIST_IDKEY;
                cfm->data.pairing_feat.rkey_dist = GAP_KDIST_ENCKEY| GAP_KDIST_IDKEY;   
            }
        } break;

	case (GAPC_LTK_EXCH):
	{
		UART_PRINTF("gapc_ltk req\r\n");
		// Counter
		uint8_t counter;
		cfm->accept  = true;
		cfm->request = GAPC_LTK_EXCH;

		// Generate all the values
		cfm->data.ltk.ediv = (uint16_t)co_rand_word();

		for (counter = 0; counter < RAND_NB_LEN; counter++)
		{
			cfm->data.ltk.ltk.key[counter]    = (uint8_t)co_rand_word();
			cfm->data.ltk.randnb.nb[counter] = (uint8_t)co_rand_word();
		}

		for (counter = RAND_NB_LEN; counter < KEY_LEN; counter++)
		{
			cfm->data.ltk.ltk.key[counter]    = (uint8_t)co_rand_word();
		}
#if (NVDS_SUPPORT)
		// Store the generated value in NVDS
		if (nvds_put(NVDS_TAG_LTK, NVDS_LEN_LTK,
		             (uint8_t *)&cfm->data.ltk) != NVDS_OK)
		{
			ASSERT_ERR(0);
		}
#else

		/*APP_SEC_DATA_P app_sec_para = (APP_SEC_DATA_P)app_sec_buff;
		flash_erase_sector(FLASH_SPACE_TYPE_NVR, 0x8000);
		memset(&app_sec_para->ltk, 0x0, sizeof( struct gapc_ltk));
		memcpy(&app_sec_para->ltk,(uint8_t *)&cfm->data.ltk,  sizeof( struct gapc_ltk));
		flash_write(FLASH_SPACE_TYPE_NVR, 0x8000, sizeof(APP_SEC_DATA_T),  app_sec_buff);*/
		APP_SEC_DATA_P app_sec_para = (APP_SEC_DATA_P)app_sec_buff;
		
		memset(&app_sec_para->ltk, 0x0, sizeof( struct gapc_ltk));
		memcpy(&app_sec_para->ltk,(uint8_t *)&cfm->data.ltk,  sizeof( struct gapc_ltk));
#endif
	}
	break;


	case (GAPC_IRK_EXCH):
	{
		UART_PRINTF("gapc_irk exch\r\n");

            cfm->accept  = true;
            cfm->request = GAPC_IRK_EXCH;

            // Load IRK
            memcpy(cfm->data.irk.irk.key, app_env.loc_irk, KEY_LEN);
            // load device address
            cfm->data.irk.addr.addr_type = ADDR_PUBLIC;
            memcpy(cfm->data.irk.addr.addr.addr,(uint8_t *)&co_default_bdaddr,BD_ADDR_LEN);
        } break;

        case (GAPC_TK_EXCH):
        {
            // Generate a PIN Code- (Between 100000 and 999999)
            uint32_t pin_code =123456;// (100000 + (co_rand_word()%900000));

            cfm->accept  = true;
            cfm->request = GAPC_TK_EXCH;

            // Set the TK value
            memset(cfm->data.tk.key, 0, KEY_LEN);

            cfm->data.tk.key[0] = (uint8_t)((pin_code & 0x000000FF) >>  0);
            cfm->data.tk.key[1] = (uint8_t)((pin_code & 0x0000FF00) >>  8);
            cfm->data.tk.key[2] = (uint8_t)((pin_code & 0x00FF0000) >> 16);
            cfm->data.tk.key[3] = (uint8_t)((pin_code & 0xFF000000) >> 24);
        } break;

        default:
        {
            ASSERT_ERR(0);
        } break;
    }

    // Send the message
    ke_msg_send(cfm);

    return (KE_MSG_CONSUMED);
}


static int gapc_bond_ind_handler(ke_msg_id_t const msgid,
                                 struct gapc_bond_ind const *param,
                                 ke_task_id_t const dest_id,
                                 ke_task_id_t const src_id)
{
	UART_PRINTF("%s param->info = 0x%x\r\n",__func__,param->info);
	switch (param->info)
	{
	case (GAPC_PAIRING_SUCCEED):
	{
		// Update the bonding status in the environment
		app_sec_env.bond_lost = false;
		app_sec_env.pairing_fail = false;
		app_sec_env.bonded = true;
		UART_PRINTF("gapc pairing success\r\n");

#if (NVDS_SUPPORT)
		// Update the bonding status in the environment
		if (nvds_put(NVDS_TAG_PERIPH_BONDED, NVDS_LEN_PERIPH_BONDED,
		             (uint8_t *)&app_sec_env.bonded) != NVDS_OK)
		{
			// An error has occurred during access to the NVDS
			ASSERT_ERR(0);
		}

		// Set the BD Address of the peer device in NVDS
		if (nvds_put(NVDS_TAG_PEER_BD_ADDRESS, NVDS_LEN_PEER_BD_ADDRESS,
		             (uint8_t *)gapc_get_bdaddr(0, SMPC_INFO_PEER)) != NVDS_OK)
		{
			// An error has occurred during access to the NVDS
			ASSERT_ERR(0);
		}
#else

		/*APP_SEC_DATA_P app_sec_para = (APP_SEC_DATA_P)app_sec_buff;
		flash_erase_sector(FLASH_SPACE_TYPE_NVR, 0x8000);
		app_sec_para->bond = 0x1; 
		memset(&app_sec_para->bdaddr,0x0, sizeof(struct gap_bdaddr));
		memcpy(&app_sec_para->bdaddr, gapc_get_bdaddr(0, SMPC_INFO_PEER), sizeof(struct gap_bdaddr));
		flash_write(FLASH_SPACE_TYPE_NVR, 0x8000, sizeof(APP_SEC_DATA_T), app_sec_buff);*/		
		
		struct bond_db_data bd;
		APP_SEC_DATA_P app_sec_para = (APP_SEC_DATA_P)app_sec_buff;
		
		memset(&bd, 0, sizeof(bd));
		
		app_sec_para->auth = param->data.auth.info; 
		
		memcpy(&app_sec_para->bdaddr, gapc_get_bdaddr(0, SMPC_INFO_PEER), sizeof(struct gap_bdaddr));

		if (app_sec_para->auth & GAP_AUTH_BOND)
		{
			// store bond data
			bd.valid = 0xAA;

			memcpy(&bd.ltk.ltk, &app_sec_para->ltk.ltk, sizeof(struct gap_sec_key));  /// Long Term Key
			bd.ltk.ediv = app_sec_para->ltk.ediv;                                     /// Encryption Diversifier
			memcpy(&bd.ltk.randnb, &app_sec_para->ltk.randnb, RAND_NB_LEN);          /// Random Number
			bd.ltk.key_size = app_sec_para->ltk.key_size;                             /// Encryption key size (7 to 16)

			// Check bits for resolvable address type
			if ((app_sec_para->bdaddr.addr_type == ADDR_RAND) &&
				(app_sec_para->bdaddr.addr.addr[0] & RND_RSLV_ADDR))
			{
				memcpy(&bd.irk, &app_sec_para->irk, sizeof(struct gapc_irk));     /// IRK
				bd.flags |= BOND_DB_ENTRY_IRK_PRESENT;
			}

			// Remote address
			bd.bdaddr.addr_type = app_sec_para->bdaddr.addr_type;
			memcpy(&bd.bdaddr.addr, &app_sec_para->bdaddr.addr.addr, sizeof(struct bd_addr));

			bd.auth = app_sec_para->auth;                                         // authentication level

			bond_db_store(&bd);
		}
		
#endif

	}
	break;

        case (GAPC_REPEATED_ATTEMPT):
        {
			app_sec_env.peer_pairing = false;
	    	app_sec_env.peer_encrypt = false;
			app_sec_env.pairing_fail = true;
            appm_disconnect();
        } break;

	case (GAPC_IRK_EXCH):
	{
		UART_PRINTF("gapc pairing GAPC_IRK_EXCH\r\n");
#if (NVDS_SUPPORT)
		// Store peer identity in NVDS
		if (nvds_put(NVDS_TAG_PEER_IRK, NVDS_LEN_PEER_IRK, (uint8_t *)&param->data.irk) != NVDS_OK)
		{
			ASSERT_ERR(0);
		}
#else

		/*flash_read(FLASH_SPACE_TYPE_NVR,0x8000, sizeof(APP_SEC_DATA_T), app_sec_buff);
		APP_SEC_DATA_P app_sec_para = (APP_SEC_DATA_P)app_sec_buff;
		flash_erase_sector(FLASH_SPACE_TYPE_NVR, 0x8000);
		memset(&app_sec_para->irk,0x0, sizeof(struct gapc_irk));
		memcpy(&app_sec_para->irk,  (uint8_t *)&param->data.irk, sizeof(struct gapc_irk));
		flash_write(FLASH_SPACE_TYPE_NVR, 0x8000, sizeof(APP_SEC_DATA_T), app_sec_buff);*/
		APP_SEC_DATA_P app_sec_para = (APP_SEC_DATA_P)app_sec_buff;
		memset(&app_sec_para->irk,0x0, sizeof(struct gapc_irk));
		memcpy(&app_sec_para->irk,  (uint8_t *)&param->data.irk, sizeof(struct gapc_irk));
#endif
		memcpy(&app_env.peer_irk,&param->data.irk.irk.key[0],sizeof(struct gapc_irk));
		UART_PRINTF("irk.key = ");
		for(int i = 0; i<sizeof(struct gap_sec_key); i++)
		{
			UART_PRINTF("0x%x ",param->data.irk.irk.key[i]);
		}
		UART_PRINTF("\r\n");

		UART_PRINTF("addr.type = %x\r\n",param->data.irk.addr.addr_type);
		UART_PRINTF("addr.addr = ");
		for(int i = 0; i<sizeof(struct bd_addr); i++)
		{
			UART_PRINTF("0x%x ",param->data.irk.addr.addr.addr[i]);
		}
		UART_PRINTF("\r\n");
	}
	break;

	case (GAPC_PAIRING_FAILED):
	{
		APP_SEC_DATA_P app_sec_para = (APP_SEC_DATA_P)app_sec_buff;
		UART_PRINTF("gapc pairing failed\r\n");
		app_sec_env.peer_pairing = false;
		app_sec_env.peer_encrypt = false;
		app_sec_env.pairing_fail = true;
		app_sec_para->auth = 0;
		//appm_disconnect();
	}
	break;

        default:
        {
            ASSERT_ERR(0);
        } break;
    }

    return (KE_MSG_CONSUMED);
}

static int gapc_encrypt_req_ind_handler(ke_msg_id_t const msgid,
                                        struct gapc_encrypt_req_ind const *param,
                                        ke_task_id_t const dest_id,
                                        ke_task_id_t const src_id)
{
	if(ke_timer_active(APP_SEND_SECURITY_REQ, TASK_APP))
	{
		ke_timer_clear(APP_SEND_SECURITY_REQ, TASK_APP);
	}
	
	UART_PRINTF("%s \r\n",__func__);
	app_sec_env.peer_encrypt = true;
    // LTK value
    struct gapc_ltk ltk;
    // Length
    uint8_t length = NVDS_LEN_LTK;

    // Prepare the GAPC_ENCRYPT_CFM message
    struct gapc_encrypt_cfm *cfm = KE_MSG_ALLOC(GAPC_ENCRYPT_CFM,
                                                src_id, TASK_APP,
                                                gapc_encrypt_cfm);

    cfm->found    = false;

	//if (app_sec_env.bonded)
	if(1)
	{
#if (NVDS_SUPPORT)

		// LTK value
		struct gapc_ltk ltk;
		// Length
		uint8_t length = NVDS_LEN_LTK;

		// Retrieve the required informations from NVDS
		if (nvds_get(NVDS_TAG_LTK, &length, (uint8_t *)&ltk) == NVDS_OK)
		{
			// Check if the provided EDIV and Rand Nb values match with the stored values
			if ((param->ediv == ltk.ediv) &&
			        !memcmp(&param->rand_nb.nb[0], &ltk.randnb.nb[0], sizeof(struct rand_nb)))
			{
				cfm->found    = true;
				cfm->key_size = 16;
				memcpy(&cfm->ltk, &ltk.ltk, sizeof(struct gap_sec_key));
			}
            else
            {
                UART_PRINTF("we are bonded with another device, disconnect the link\r\n");
                app_sec_env.bond_lost = true;
                appm_disconnect();
            }
			/*
			 * else we are bonded with another device, disconnect the link
			 */
		}
		else
		{
			UART_PRINTF("read ltk fail\r\n");
            app_sec_env.bond_lost = true;
            appm_disconnect();
			ASSERT_ERR(0);
		}
#else
		/*
		flash_read(FLASH_SPACE_TYPE_NVR,0x8000,sizeof(APP_SEC_DATA_T),  (uint8_t *)app_sec_para);
		// Check if the provided EDIV and Rand Nb values match with the stored values
		if ((param->ediv == app_sec_para->ltk.ediv) &&
		        !memcmp(&param->rand_nb.nb[0], (void*)app_sec_para->ltk.randnb.nb[0], sizeof(struct rand_nb)))
		{
			cfm->found    = true;
			cfm->key_size = 16;
			memcpy(&cfm->ltk, &app_sec_para->ltk.ltk, sizeof(struct gap_sec_key));
		}
		else
		{
			UART_PRINTF("we are bonded with another device, disconnect the link\r\n");
		}
		*/
		/*flash_read(FLASH_SPACE_TYPE_NVR,0x8000, sizeof(APP_SEC_DATA_T), app_sec_buff);
		APP_SEC_DATA_P app_sec_para = (APP_SEC_DATA_P)app_sec_buff;

		// Check if the provided EDIV and Rand Nb values match with the stored values
		if ((param->ediv == app_sec_para->ltk.ediv) &&
		        !memcmp(&param->rand_nb.nb[0], (void*)app_sec_para->ltk.randnb.nb, sizeof(struct rand_nb)))   
		{
			cfm->found    = true;
			cfm->key_size = 16;
			memcpy(&cfm->ltk, &app_sec_para->ltk.ltk, sizeof(struct gap_sec_key));
		}
		else
		{
			UART_PRINTF("we are bonded with another device, disconnect the link\r\n");
            app_sec_env.bond_lost = true;
            appm_disconnect();
		} */
		
		APP_SEC_DATA_P app_sec_para = (APP_SEC_DATA_P)app_sec_buff;
		const struct bond_db_data *pbd;
		
		// Retrieve bond data and put it inside the app_sec_env
		pbd = bond_db_lookup_by_ediv(&param->rand_nb, param->ediv);
		if (pbd)
		{
			UART_PRINTF("found ediv\r\n");
			//app_sec_env[connection_idx].auth = pbd->auth;                                               /// Authentication
			app_sec_para->auth = pbd->auth;
			memcpy(&app_sec_para->ltk.ltk, &pbd->ltk.ltk, sizeof(struct gap_sec_key));        /// Long Term Key
			app_sec_para->ltk.ediv = pbd->ltk.ediv;                                           /// Encryption Diversifier
			memcpy(app_sec_para->ltk.randnb.nb, &pbd->ltk.randnb, RAND_NB_LEN);                /// Random Number
			app_sec_para->ltk.key_size = pbd->ltk.key_size;                                   /// Encryption key size (7 to 16)
			memcpy(&app_sec_para->irk, &pbd->irk, sizeof(struct gapc_irk));               /// IRK
			
			memcpy(&cfm->ltk, &app_sec_para->ltk.ltk, sizeof(struct gap_sec_key));
			
			cfm->found    = true;
			cfm->key_size = 16;
		}
		else
		{
			UART_PRINTF("we are bonded with another device, disconnect the link\r\n");
            app_sec_env.bond_lost = true;
            appm_disconnect();
		} 
#endif
	}
	else
	{
		UART_PRINTF("bond lost,disconnect link,\r\n");
		app_sec_env.bond_lost = true;
		appm_disconnect();
	}
    /*
     * else the peer device is not known, an error should trigger a new pairing procedure.
     */

    // Send the message
    ke_msg_send(cfm);

    return (KE_MSG_CONSUMED);
}


static int gapc_encrypt_ind_handler(ke_msg_id_t const msgid,
                                    struct gapc_encrypt_ind const *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id)
{
    // encryption/ re-encryption succeeded
	UART_PRINTF("%s \r\n",__func__);

    return (KE_MSG_CONSUMED);
}

static int app_sec_msg_dflt_handler(ke_msg_id_t const msgid,
                                    void *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id)
{
    // Drop the message

    return (KE_MSG_CONSUMED);
}

/*
 * LOCAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/// Default State handlers definition
const struct ke_msg_handler app_sec_msg_handler_list[] =
{
    // Note: first message is latest message checked by kernel so default is put on top.
    {KE_MSG_DEFAULT_HANDLER,  (ke_msg_func_t)app_sec_msg_dflt_handler},

    {GAPC_BOND_REQ_IND,       (ke_msg_func_t)gapc_bond_req_ind_handler},
    {GAPC_BOND_IND,           (ke_msg_func_t)gapc_bond_ind_handler},

    {GAPC_ENCRYPT_REQ_IND,    (ke_msg_func_t)gapc_encrypt_req_ind_handler},
    {GAPC_ENCRYPT_IND,        (ke_msg_func_t)gapc_encrypt_ind_handler},
};

const struct ke_state_handler app_sec_table_handler =
    {&app_sec_msg_handler_list[0], (sizeof(app_sec_msg_handler_list)/sizeof(struct ke_msg_handler))};


/// @} APP
