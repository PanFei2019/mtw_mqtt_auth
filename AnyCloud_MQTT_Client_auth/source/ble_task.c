/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the WiFi Onboarding Using BLE
*              project for ModusToolbox.
*
* Related Document: See Readme.md
*
*******************************************************************************
* Copyright 2021, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

#include <app_utils.h>
#include "cybsp.h"
#include "cy_retarget_io.h"
#include <FreeRTOS.h>
#include <task.h>
#include "stdlib.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "cycfg_bt_settings.h"
#include "cycfg_gap.h"
#include "cybsp_bt_config.h"
#include "wiced_bt_stack.h"
#include "cycfg_gatt_db.h"
//#include "wifi_task.h"
#include "wiced_memory.h"

#if 1//dennis
#include "cy_em_eeprom.h"
#include "cybt_platform_config.h"
#endif

/******************************************************************************
 *                                Constants
 ******************************************************************************/

/* Priority for the button interrupt */
#define GPIO_INTERRUPT_PRIORITY (7)

/* LE Key Size */
#define MAX_KEY_SIZE (16)

/******************************************************************************
 *                                 TYPEDEFS
 ******************************************************************************/
typedef void (*pfn_free_buffer_t)(uint8_t *);

/******************************************************************************
 *                             Global Variables
 ******************************************************************************/

#if 1//dennis
wiced_bt_device_link_keys_t linkkey_info;
uint8_t link_encrypted = false;
uint8_t start_notification = false;
uint8_t start_indication = false;
uint8_t auth_data[15];
uint8_t auth_data_len = 0;
uint8_t auth_req_start = 0;
uint8_t auth_res_start = 0;
uint8_t auth_cpt_start = 0;
uint8_t auth_cpt_ack_start = 0;
#if 0
//eeprom
/* Logical Start of Emulated EEPROM in bytes. */
#define LOGICAL_EEPROM_START    (0u)

/* Set the macro FLASH_REGION_TO_USE to either USER_FLASH or
 * EMULATED_EEPROM_FLASH to specify the region of the flash used for
 * emulated EEPROM.
 */
#define USER_FLASH              (0u)
#define EMULATED_EEPROM_FLASH   (1u)
#define FLASH_REGION_TO_USE     EMULATED_EEPROM_FLASH


#if (FLASH_REGION_TO_USE)
CY_SECTION(".cy_em_eeprom")
#endif /* #if(FLASH_REGION_TO_USE) */
CY_ALIGN(CY_EM_EEPROM_FLASH_SIZEOF_ROW)


/* EEPROM Configuration details. */
#define SIMPLE_MODE             (0u) 
#define EEPROM_SIZE             (512u)
#define BLOCKING_WRITE          (1u)
#define REDUNDANT_COPY          (1u)
#define WEAR_LEVELLING_FACTOR   (2u)

const uint8_t EepromStorage[CY_EM_EEPROM_GET_PHYSICAL_SIZE(EEPROM_SIZE, 
SIMPLE_MODE, WEAR_LEVELLING_FACTOR, REDUNDANT_COPY)] = {0u};

/* EEPROM configuration and context structure. */
cy_stc_eeprom_config_t Em_EEPROM_config =
{
    .eepromSize = EEPROM_SIZE,
    .blockingWrite = BLOCKING_WRITE,
    .redundantCopy = REDUNDANT_COPY,
    .wearLevelingFactor = WEAR_LEVELLING_FACTOR,
    .userFlashStartAddr = (uint32_t)EepromStorage,
};

cy_stc_eeprom_context_t Em_EEPROM_context;
#endif

#if 1//dennis  define heap
typedef struct
{
    uint8_t         *p_rx_mem;
    uint32_t        rx_mem_size;
    uint8_t         *p_tx_cmd_mem;
    uint32_t        tx_cmd_mem_size;
    wiced_bt_heap_t *p_tx_data_heap;
    uint32_t        tx_data_heap_size;
} cybt_task_mem_cb_t;

extern cybt_task_mem_cb_t  task_mem_cb;
#endif

#endif


/* Task Handles for WiFi Task */
TaskHandle_t wifi_task_handle;

/* Maintains the connection id of the current connection */
uint16_t conn_id = 0;

/* This variable is set to true when button callback is received and
 * data is present in EMEEPROM. It is set to false after the WiFi Task
 * processes Disconnection notification. It is used to check button
 * interrupt while the device is trying to connect to WiFi
 */
volatile bool button_pressed = false;

/* This enables RTOS aware debugging. */
volatile int uxTopUsedPriority;

/******************************************************************************
 *                              Function Prototypes
 ******************************************************************************/
static wiced_result_t app_management_callback(wiced_bt_management_evt_t event,
                      wiced_bt_management_evt_data_t *p_event_data);
static wiced_bt_gatt_status_t app_gatts_callback(wiced_bt_gatt_evt_t event,
                              wiced_bt_gatt_event_data_t *p_data);
static void application_init(void);
uint8_t *app_alloc_buffer(uint16_t len);
void app_free_buffer(uint8_t *p_data);

#if 1//dennis
void read_host_info(void);
static void send_notification(void);
static void send_indication(void);
extern void publish_ping(void);
extern void publish_auth_req(void);
extern void publish_auth_cpt(void);
#endif

#if 1//feichao
wiced_bt_gatt_status_t app_gatt_disconnect_callback(wiced_bt_gatt_connection_status_t *p_conn_status);
#endif

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This is the main function for CM4 CPU
*    1. Initializes the BSP
*    2. Configures the button for interrupt
*    3. Initializes retarget IO for UART debug printing
*    4. Initializes platform configuration
*    5. Initializes BT stack and heap
*    6. Creates WiFi connect and disconnect tasks
*    7. Starts the RTOS scheduler
*
* Return:
*  int
*
*******************************************************************************/
void ble_task(void *args)
{
	#if 1//dennis
	static uint8_t connect_time_counter = 0;
	#endif
	
	uint32_t ulNotifiedValue;

    /* Return status for EEPROM. */
    cy_en_em_eeprom_status_t eepromReturnValue;

    printf("***************************\n"
           "BLE task start\n"
           "***************************\n\n");

	#if 0
    /* Initialize the EMEEPROM. */
    eepromReturnValue = Cy_Em_EEPROM_Init(&Em_EEPROM_config, &Em_EEPROM_context);
    if (CY_EM_EEPROM_SUCCESS != eepromReturnValue)
    {
        printf("Error initializing EMEEPROM\n");
        CY_ASSERT(0);
    }
	#endif

    /* Configure platform specific settings for Bluetooth */
    cybt_platform_config_init(&cybsp_bt_platform_cfg);

    /* Initialize the Bluetooth stack with a callback function and stack
     * configuration structure */
    if (WICED_SUCCESS != wiced_bt_stack_init(app_management_callback,
                                             &wiced_bt_cfg_settings))
    {
        printf("Error initializing BT stack\n");
        CY_ASSERT(0);
    }

	while(true)
    {
        /* Wait for a notification */
        xTaskNotifyWait( 0x00,      /* Don't clear any notification bits on entry. */
                         UINT32_MAX, /* Reset the notification value to 0 on exit. */
                         &ulNotifiedValue, /* Notified value pass out in
                                              ulNotifiedValue. */
                         #if 0
                         portMAX_DELAY );  /* Block indefinitely. */
						 #else
						 500);
						 #endif	
		
		#if 0
		if(link_encrypted)	
		{
			if(connect_time_counter++>=20)
			{
				link_encrypted = false;
				__NVIC_SystemReset();	
			}
			if(connect_time_counter==10)
			{
				printf("auto-reset \n");
			}
		}
		#else
		if(link_encrypted)		
		{
			#if 0
			if(start_notification)
			{
				send_notification();
			}
			#else
			if(start_indication)
			{
				send_indication();
			}
			#endif
			
			#if 1//IS_AUTHORIZER
			if(auth_res_start)
			{
				if(connect_time_counter++>=2)
				{
					connect_time_counter = 0;
					publish_auth_res();
				}
			}
			if(auth_cpt_ack_start)
			{
				if(connect_time_counter++>=2)
				{
					connect_time_counter = 0;
					publish_auth_cpt_ack();
				}
			}
			//#else
			if(auth_req_start)
			{
				if(connect_time_counter++>=10)
				{
					connect_time_counter = 0;
					publish_auth_req();
				}
			}
			
			if(auth_cpt_start)
			{
				if(connect_time_counter++>=2)
				{
					connect_time_counter = 0;
					publish_auth_cpt();
				}
			}
			#endif
		}
		#endif	
						 
	 }
}

/*******************************************************************************
* Function Name: application_init
********************************************************************************
* Summary:
* This function is called from the BTM enabled event
*    1. Initializes and registers the GATT DB
*    2. Sets pairable mode to true
*    3. Sets ADV data and starts advertising
*
*******************************************************************************/
void application_init(void)
{
    wiced_result_t result = WICED_ERROR;
    wiced_bt_gatt_status_t gatt_status = WICED_BT_GATT_INVALID_CONNECTION_ID;

    /* Return status for EEPROM. */
    cy_en_em_eeprom_status_t eepromReturnValue;


    /* Register with stack to receive GATT callback */
    gatt_status = wiced_bt_gatt_register(app_gatts_callback);
    printf("\nGATT status:\t");
    printf(get_bt_gatt_status_name(gatt_status));
    printf("\n");

    /*  Inform the stack to use our GATT database */
    gatt_status = wiced_bt_gatt_db_init(gatt_database, gatt_database_len, NULL);

    /* Allow peer to pair */
    wiced_bt_set_pairable_mode(WICED_TRUE, false);

    result = wiced_bt_ble_set_raw_advertisement_data(CY_BT_ADV_PACKET_DATA_SIZE,
                                                     cy_bt_adv_packet_data);
    if (WICED_SUCCESS != result)
    {
        printf("Set ADV data failed\n");
    }

	#if 0//dennis
    /* Determine if the EEPROM has a previously stored WiFi SSID value.
     * If it does, use the stored credentials to connect to WiFi.
     * Otherwise, start BLE advertisements so that the user can use BLE
     * to enter the WiFi credentials. */
    eepromReturnValue = Cy_Em_EEPROM_Read(LOGICAL_EEPROM_START,
                                      (void *)&wifi_details.wifi_ssid[0],
                                      sizeof(wifi_details), &Em_EEPROM_context);

    if ((CY_EM_EEPROM_SUCCESS == eepromReturnValue) &&
        (0 != wifi_details.ssid_len))
    {
        printf("Data present in EMEEPROM\n");
        /* Unblock WiFi task with notification value indicating the WiFi
         * credentials should be taken from EMEEPROM
         */
        xTaskNotify(wifi_task_handle, NOTIF_EMEEPROM, eSetValueWithOverwrite);
    }
    else /* WiFi credentials not found in EMEEPROM */
    {
        printf("Data not present in EMEEPROM\n");
        result = wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH,
                                               BLE_ADDR_PUBLIC, NULL);
        if(WICED_SUCCESS != result)
        {
            printf("Start ADV failed");
        }
    }
	#else
    //read_host_info();
	if(1)//(linkkey_info.bd_addr[0]==0)
	{
		//wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH,
        //                                    BLE_ADDR_PUBLIC, NULL);
	}
	else
	{
		printf("reconnecting\r\n");
		if(wiced_bt_start_advertisements(BTM_BLE_ADVERT_DIRECTED_LOW,
				BLE_ADDR_PUBLIC, linkkey_info.bd_addr))
		{
			printf("Failed to start high duty cycle directed advertising!!!\n");
		}
	}
	#endif
}

/*******************************************************************************
* Function Name: app_management_callback
********************************************************************************
* Summary:
* This function handles the BT stack events.
*
* Parameters:
*  wiced_bt_management_evt_t: event code
*  p_event_data: Pointer to the event data
*
* Return:
*  wiced_result_t: Result
*
*******************************************************************************/
wiced_result_t app_management_callback(wiced_bt_management_evt_t event,
               wiced_bt_management_evt_data_t *p_event_data)
{
	#if 1//dennis
	cy_en_em_eeprom_status_t eepromReturnValue;
	#endif
	
    wiced_result_t result = WICED_BT_SUCCESS;
    wiced_bt_device_address_t bda = {0};
    wiced_bt_dev_ble_io_caps_req_t *pairing_io_caps = &(p_event_data->
                                           pairing_io_capabilities_ble_request);

    printf("Bluetooth Management Event: \t");
    printf(get_bt_event_name(event));
    printf("\n");

    switch (event)
    {
    case BTM_ENABLED_EVT:
        /* Initialize the application */
		wiced_bt_set_local_bdaddr((uint8_t *)cy_bt_device_address, BLE_ADDR_PUBLIC);
        //wiced_bt_set_local_bdaddr((uint8_t *)cy_bt_device_address, BLE_ADDR_RANDOM);
        /* Bluetooth is enabled */
        wiced_bt_dev_read_local_addr(bda);
        printf("Local Bluetooth Address: ");
        print_bd_address(bda);

        application_init();
        break;

    case BTM_DISABLED_EVT:
        break;

    case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT:
        pairing_io_caps->local_io_cap = BTM_IO_CAPABILITIES_NONE;

        pairing_io_caps->oob_data = BTM_OOB_NONE;

		#if 0//dennis
        pairing_io_caps->auth_req = BTM_LE_AUTH_REQ_SC;
		#else
		pairing_io_caps->auth_req = BTM_LE_AUTH_REQ_SC_BOND;
		#endif

        pairing_io_caps->max_key_size = MAX_KEY_SIZE;

        pairing_io_caps->init_keys = BTM_LE_KEY_PENC | BTM_LE_KEY_PID |
                                    BTM_LE_KEY_PCSRK | BTM_LE_KEY_LENC;

        pairing_io_caps->resp_keys = BTM_LE_KEY_PENC | BTM_LE_KEY_PID |
                                    BTM_LE_KEY_PCSRK | BTM_LE_KEY_LENC;
        break;

    case BTM_PAIRING_COMPLETE_EVT:
        if (WICED_SUCCESS == p_event_data->
                            pairing_complete.pairing_complete_info.ble.status)
        {
            printf("Pairing Complete: SUCCESS\n");
        }
        else /* Pairing Failed */
        {
            printf("Pairing Complete: FAILED\n");
        }
        break;

    case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT:
        /* Paired Device Link Keys update */
		#if 0 //dennis
    	for(uint8_t i=0;i<sizeof(wiced_bt_device_link_keys_t);i++)
    	{
    		printf("%02x ", *(((uint8_t*)&p_event_data->paired_device_link_keys_update)+i));
    	}
    	printf("\r\n");
		
		printf("linkkey: ");
		for(uint8_t i=0;i<16;i++)
		{			
			printf("%02x ", *(((uint8_t*)&p_event_data->paired_device_link_keys_update.key_data.le_keys.pltk[0])+i));
		}
		printf("\r\n");
        #endif

		#if 0//dennis
    	eepromReturnValue = Cy_Em_EEPROM_Erase(&Em_EEPROM_context);
    	if(CY_EM_EEPROM_SUCCESS != eepromReturnValue)
		{
			printf("Failed to erase EMEEPROM \n");
		}

    	eepromReturnValue = Cy_Em_EEPROM_Write(LOGICAL_EEPROM_START,
    											(void *)((uint8_t *)&p_event_data->paired_device_link_keys_update.bd_addr[0]),
    											sizeof(wiced_bt_device_link_keys_t),
    											&Em_EEPROM_context);

    	if(CY_EM_EEPROM_SUCCESS != eepromReturnValue)
    	{
    		printf("Failed to write to EMEEPROM \n");
    	}
		#endif
        result = WICED_SUCCESS;
        break;

    case BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
        /* Paired Device Link Keys Request */
		#if 0//dennis
        if(linkkey_info.bd_addr[0]|linkkey_info.bd_addr[1]|linkkey_info.bd_addr[2]|linkkey_info.bd_addr[3]|linkkey_info.bd_addr[4]|linkkey_info.bd_addr[5])
    	{
    		printf("linkkey request success\r\n");
    		memcpy(&p_event_data->paired_device_link_keys_request, &linkkey_info, sizeof(wiced_bt_device_link_keys_t));
			#if 0
			 /* Add this peer device in the security database */
			result = wiced_bt_dev_add_device_to_address_resolution_db(&linkkey_info);
			if (result != WICED_BT_SUCCESS)
			{
				printf( "Err: wiced_bt_dev_add_device_to_address_resolution_db failed: %d\n", result);
			}
			else
			{
				printf( "add device to address resolution database success\r\n");
			}
			#endif
    		result = WICED_SUCCESS;
    	}
    	else
    	{
    		result = WICED_BT_ERROR;
    	}
		#else
        result = WICED_BT_ERROR;
		#endif
        break;

    case BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT:
        /* Local identity Keys Update */
        result = WICED_SUCCESS;
        break;

    case BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT:
        /* Local identity Keys Request */
        result = WICED_BT_ERROR;
        break;

    case BTM_ENCRYPTION_STATUS_EVT:
        if (WICED_SUCCESS == p_event_data->encryption_status.result)
        {
            printf("Encryption Status Event: SUCCESS\n");
			#if 1//dennis
			link_encrypted = true;
			#endif
        }
        else /* Encryption Failed */
        {
            printf("Encryption Status Event: FAILED\n");
        }
        break;

    case BTM_SECURITY_REQUEST_EVT:
        wiced_bt_ble_security_grant(p_event_data->security_request.bd_addr,
                                    WICED_BT_SUCCESS);
        break;

    case BTM_BLE_ADVERT_STATE_CHANGED_EVT:
        printf("\n");
        printf("Advertisement state changed to ");
        printf(get_bt_advert_mode_name(p_event_data->ble_advert_state_changed));
        printf("\n");
        break;

    default:
        break;
    }

    return result;
}

/*******************************************************************************
* Function Name: app_get_attribute
********************************************************************************
* Summary:
* This function searches through the GATT DB to point to the attribute
* corresponding to the given handle
*
* Parameters:
*  uint16_t handle: Handle to search for in the GATT DB
*
* Return:
*  gatt_db_lookup_table_t *: Pointer to the correct attribute in the GATT DB
*
*******************************************************************************/
gatt_db_lookup_table_t *app_get_attribute(uint16_t handle)
{
    /* Search for the given handle in the GATT DB and return the pointer to the
    correct attribute */
    uint8_t array_index = 0;

    for (array_index = 0; array_index < app_gatt_db_ext_attr_tbl_size; array_index++)
    {
        if (app_gatt_db_ext_attr_tbl[array_index].handle == handle)
        {
            return (&app_gatt_db_ext_attr_tbl[array_index]);
        }
    }
    return NULL;
}

/*******************************************************************************
* Function Name: app_gatts_req_read_handler
********************************************************************************
* Summary:
* This function handles the GATT read request events from the stack
*
* Parameters:
*  uint16_t conn_id: Connection ID
*  wiced_bt_gatt_read_t * p_read_data: Read data structure
*
* Return:
*  wiced_bt_gatt_status_t: GATT result
*
*******************************************************************************/
wiced_bt_gatt_status_t app_gatts_req_read_handler(uint16_t conn_id,
                       wiced_bt_gatt_opcode_t opcode,
                       wiced_bt_gatt_read_t *p_read_data, uint16_t len_requested)
{

    gatt_db_lookup_table_t *puAttribute;
    int attr_len_to_copy;

    /* Get the right address for the handle in Gatt DB */
    if (NULL == (puAttribute = app_get_attribute(p_read_data->handle)))
    {
        printf("Read handle attribute not found. Handle:0x%X\n",
                p_read_data->handle);
        wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, p_read_data->handle,
                                            WICED_BT_GATT_INVALID_HANDLE);
        return WICED_BT_GATT_INVALID_HANDLE;
    }

    attr_len_to_copy = puAttribute->cur_len;

    //printf("GATT Read handler: handle:0x%X, len:%d\n",
    //       p_read_data->handle, attr_len_to_copy);

    /* If the incoming offset is greater than the current length in the GATT DB
    then the data cannot be read back*/
    if (p_read_data->offset >= puAttribute->cur_len)
    {
        wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, p_read_data->handle,
                                            WICED_BT_GATT_INVALID_OFFSET);
        return (WICED_BT_GATT_INVALID_OFFSET);
    }

    int to_send = MIN(len_requested, attr_len_to_copy - p_read_data->offset);

    uint8_t *from = ((uint8_t *)puAttribute->p_data) + p_read_data->offset;

    wiced_bt_gatt_server_send_read_handle_rsp(conn_id, opcode, to_send, from, NULL);

    return WICED_BT_GATT_SUCCESS;
}

/*******************************************************************************
* Function Name: app_gatt_read_by_type_handler
********************************************************************************
* Summary:
* This function handles the GATT read by type request events from the stack
*
* Parameters:
*  uint16_t conn_id: Connection ID
*  wiced_bt_gatt_opcode_t opcode: GATT opcode
*  wiced_bt_gatt_read_t * p_read_data: Read data structure
*  uint16_t len_requested: Length requested
*
* Return:
*  wiced_bt_gatt_status_t: GATT result
*
*******************************************************************************/
wiced_bt_gatt_status_t app_gatt_read_by_type_handler(uint16_t conn_id,
                        wiced_bt_gatt_opcode_t opcode,
                        wiced_bt_gatt_read_by_type_t *p_read_data,
                        uint16_t len_requested)
{
    gatt_db_lookup_table_t *puAttribute;
    uint16_t    attr_handle = p_read_data->s_handle;
	#if 0//dennis
    uint8_t     *p_rsp = wiced_bt_get_buffer(len_requested);
	#else
	uint8_t     *p_rsp = wiced_bt_get_buffer_from_heap(task_mem_cb.p_tx_data_heap, (len_requested));
	#endif
    uint8_t    pair_len = 0;
    int used = 0;

    if (p_rsp == NULL)
    {
        wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, attr_handle,
                                            WICED_BT_GATT_INSUF_RESOURCE);
        return WICED_BT_GATT_INSUF_RESOURCE;
    }

    /* Read by type returns all attributes of the specified type,
     * between the start and end handles */
    while (WICED_TRUE)
    {
        attr_handle = wiced_bt_gatt_find_handle_by_type(attr_handle,
                p_read_data->e_handle, &p_read_data->uuid);

        if (attr_handle == 0)
            break;

        if ((puAttribute = app_get_attribute(attr_handle)) == NULL)
        {
            wiced_bt_gatt_server_send_error_rsp(conn_id, opcode,
                    p_read_data->s_handle, WICED_BT_GATT_ERR_UNLIKELY);
            wiced_bt_free_buffer(p_rsp);
            return WICED_BT_GATT_ERR_UNLIKELY;
        }

        {
            int filled = wiced_bt_gatt_put_read_by_type_rsp_in_stream(
                    p_rsp + used, len_requested - used, &pair_len, attr_handle,
                     puAttribute->cur_len, puAttribute->p_data);
            if (filled == 0)
            {
                break;
            }
            used += filled;
        }

        /* Increment starting handle for next search to one past current */
        attr_handle++;
    }

    if (used == 0)
    {
        wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, p_read_data->s_handle,
                                            WICED_BT_GATT_INVALID_HANDLE);
        wiced_bt_free_buffer(p_rsp);
        return WICED_BT_GATT_INVALID_HANDLE;
    }

    /* Send the response */
    wiced_bt_gatt_server_send_read_by_type_rsp(conn_id, opcode, pair_len,
            used, p_rsp, (wiced_bt_gatt_app_context_t)wiced_bt_free_buffer);

    return WICED_BT_GATT_SUCCESS;
}

/*******************************************************************************
* Function Name: app_gatts_req_write_handler
********************************************************************************
* Summary:
* This function handles the GATT write request events from the stack
*
* Parameters:
*  uint16_t conn_id: Connection ID
*  wiced_bt_gatt_write_t * p_data: Write data structure
*
* Return:
*  wiced_bt_gatt_status_t: GATT result
*
*******************************************************************************/
wiced_bt_gatt_status_t app_gatts_req_write_handler(uint16_t conn_id,
                       wiced_bt_gatt_opcode_t opcode,
                       wiced_bt_gatt_write_req_t *p_data)
{
    wiced_bt_gatt_status_t result = WICED_BT_GATT_SUCCESS;
    uint8_t *p_attr = p_data->p_val;
    gatt_db_lookup_table_t *puAttribute;

    //printf("GATT write handler: handle:0x%X len:%d, opcode:0x%X\n",
    //       p_data->handle, p_data->val_len, opcode);

    /* Get the right address for the handle in Gatt DB */
    if (NULL == (puAttribute = app_get_attribute(p_data->handle)))
    {
        printf("\nWrite Handle attr not found. Handle:0x%X\n", p_data->handle);
        return WICED_BT_GATT_INVALID_HANDLE;
    }
	
	#if 0//dennis
    switch (p_data->handle)
    {
    /* Write request for the WiFi SSID characteristic. Copy the incoming data
    to the WIFI SSID variable */
    case HDLC_CUSTOM_SERVICE_WIFI_SSID_VALUE:
        memset(app_custom_service_wifi_ssid, 0,
               strlen((char *)app_custom_service_wifi_ssid));
        memcpy(app_custom_service_wifi_ssid, p_attr, p_data->val_len);
        puAttribute->cur_len = p_data->val_len;
        printf("Wi-Fi SSID: %s\n", app_custom_service_wifi_ssid);
        break;

    /* Write request for the WiFi password characteristic. Accept the password.
    Copy the incoming data to the WIFI Password variable */
    case HDLC_CUSTOM_SERVICE_WIFI_PASSWORD_VALUE:
        memset(app_custom_service_wifi_password, 0,
                strlen((char *)app_custom_service_wifi_password));
        memcpy(app_custom_service_wifi_password, p_attr, p_data->val_len);
        puAttribute->cur_len = p_data->val_len;
        printf("Wi-Fi Password: %s\n", app_custom_service_wifi_password);
        break;

    /* Write request for the connection characteristic. Copy the incoming data
    to the WIFI connection variable. Based on the value either start connect
    or disconnect procedure */
    case HDLC_CUSTOM_SERVICE_WIFI_CONNECTION_VALUE:
        app_custom_service_wifi_connection[0] = p_attr[0];
        puAttribute->cur_len = p_data->val_len;

        /* Connection message */
        if (app_custom_service_wifi_connection[0])
        {
            /* Unblock WiFiConnect task with notification value indicating the
             * WiFi credentials should be taken from GATT DB
             */
            xTaskNotify(wifi_task_handle, NOTIF_GATT_DB, eSetValueWithOverwrite);
        }
        else /* Disconnection message */
        {
            xTaskNotify(wifi_task_handle, NOTIF_DISCONNECT_GATT_DB,
                        eSetValueWithOverwrite);
        }
        break;

    /* Notification for connection characteristic. If enabled, notification can
     * be sent to the client if the connection was successful or not
     */
    case HDLD_CUSTOM_SERVICE_WIFI_CONNECTION_CLIENT_CHAR_CONFIG:
        app_custom_service_wifi_connection_client_char_config[0] = p_attr[0];
        app_custom_service_wifi_connection_client_char_config[1] = p_attr[1];
        break;

    default:
        printf("Write GATT Handle not found\n");
        result = WICED_BT_GATT_INVALID_HANDLE;
        break;
    }
	#endif
	
	switch (p_data->handle)
    {
		case HDLD_NOTIFY_SERVICE_CLIENT_CHAR_CONFIG:
		app_notify_service_client_char_config[0] = p_attr[0];
		app_notify_service_client_char_config[1] = p_attr[1];
		
		app_notify_service_chara[1] = 1;
		app_notify_service_chara[2] = 1;
		//start_notification = true;
		start_indication = true;
		break;
		
		case HDLC_TEST_SERVICE_TEST_CHARA_VALUE:
		//start_notification = false;
		start_indication = false;
		for(uint8_t i=3;i<p_data->val_len;i++)
		{
			
			auth_data[i-3] = p_attr[i];
			//printf("%02x ",p_attr[i]);
		}
		//printf("\n");
		auth_data_len = p_data->val_len-3;
				
		switch(p_attr[2])
		{
			case 1:
			auth_req_start = 1;
			publish_auth_req();
			break;
			
			case 2:
			auth_cpt_start = 1;
			publish_auth_cpt();
			break;
			
			default:
			break;
		}
		//start_notification = true;
		//start_indication = true;
		break;
		
		default:
        printf("Write GATT Handle not found\n");
        result = WICED_BT_GATT_INVALID_HANDLE;
        break;
	}
	
    return result;
}

/*******************************************************************************
* Function Name: app_gatt_connect_callback
********************************************************************************
* Summary:
* This function handles the GATT connect request events from the stack
*
* Parameters:
*  wiced_bt_gatt_connection_status_t *p_conn_status: Connection or disconnection
*
* Return:
*  wiced_bt_gatt_status_t: GATT result
*
*******************************************************************************/
wiced_bt_gatt_status_t app_gatt_connect_callback(
                       wiced_bt_gatt_connection_status_t *p_conn_status)
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_ERROR;
    wiced_result_t result;

    /* Check whether it is a connect event or disconnect event. If the device
    has been disconnected then restart advertisement */
    if (NULL != p_conn_status)
    {
        if (p_conn_status->connected)
        {
            /* Device got connected */
            printf("\nConnected: Peer BD Address: ");
            print_bd_address(p_conn_status->bd_addr);
            printf("\n");
            conn_id = p_conn_status->conn_id;

			#if 0 //dennis		 	
			read_host_info();
            if(linkkey_info.bd_addr[0]|linkkey_info.bd_addr[1]|linkkey_info.bd_addr[2]|linkkey_info.bd_addr[3]|linkkey_info.bd_addr[4]|linkkey_info.bd_addr[5])
            {
            	printf("bonded\r\n");
            }
            else
            {
            	printf("bonding\r\n");
                result = wiced_bt_dev_sec_bond(p_conn_status->bd_addr,p_conn_status->addr_type,2,0,NULL);
            }
			#endif
			
        }
        else /* Device got disconnected */
        {
			start_indication = false;
            printf("\nDisconnected: Peer BD Address: ");
            print_bd_address(p_conn_status->bd_addr);
            printf("\n");

            printf("Reason for disconnection: \t");
            printf(get_bt_gatt_disconn_reason_name(p_conn_status->reason));
            printf("\n");

            conn_id = 0;

            result = wiced_bt_ble_set_raw_advertisement_data(CY_BT_ADV_PACKET_DATA_SIZE,
                                                             cy_bt_adv_packet_data);
            if (WICED_SUCCESS != result)
            {
                printf("Set ADV data failed\n");
            }

            result = wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH,
                                                   0, NULL);
            if(WICED_SUCCESS != result)
            {
                printf("Start ADV failed");
            }
        }
        status = WICED_BT_GATT_SUCCESS;
    }

    return status;
}

/*******************************************************************************
* Function Name: app_gatts_req_cb
********************************************************************************
* Summary:
* This function redirects the GATT attribute requests to the appropriate
* functions
*
* Parameters:
*  wiced_bt_gatt_attribute_request_t *p_data: GATT request data structure
*
* Return:
*  wiced_bt_gatt_status_t: GATT result
*
*******************************************************************************/
wiced_bt_gatt_status_t app_gatts_req_cb(wiced_bt_gatt_attribute_request_t *p_data)
{
    wiced_bt_gatt_status_t result = WICED_BT_GATT_INVALID_PDU;
    wiced_bt_gatt_write_req_t *p_write_request = &p_data->data.write_req;

	//printf("%s %d:opcode=%d\n",__FUNCTION__, __LINE__,p_data->opcode);

    switch (p_data->opcode)
    {
    case GATT_REQ_READ:
    case GATT_REQ_READ_BLOB:
        result = app_gatts_req_read_handler(p_data->conn_id, p_data->opcode,
                                &p_data->data.read_req, p_data->len_requested);
        break;

    case GATT_REQ_READ_BY_TYPE:
        result = app_gatt_read_by_type_handler(p_data->conn_id, p_data->opcode,
                            &p_data->data.read_by_type, p_data->len_requested);
        break;

    case GATT_REQ_WRITE:
    case GATT_CMD_WRITE:
    case GATT_CMD_SIGNED_WRITE:
        result = app_gatts_req_write_handler(p_data->conn_id, p_data->opcode,
                                             &(p_data->data.write_req));
        if ((p_data->opcode == GATT_REQ_WRITE) && (result == WICED_BT_GATT_SUCCESS))
        {
            wiced_bt_gatt_server_send_write_rsp(p_data->conn_id, p_data->opcode,
                                                p_write_request->handle);
        }
        else
        {
            wiced_bt_gatt_server_send_error_rsp(p_data->conn_id, p_data->opcode,
                    p_write_request->handle, result);
        }
        break;

    case GATT_REQ_MTU:
        printf("Exchanged MTU from client: %d\n", p_data->data.remote_mtu);
        wiced_bt_gatt_server_send_mtu_rsp(p_data->conn_id, p_data->data.remote_mtu,
                          wiced_bt_cfg_settings.p_ble_cfg->ble_max_rx_pdu_size);
        result = WICED_BT_GATT_SUCCESS;
        break;
		
	case GATT_HANDLE_VALUE_CONF:
		start_indication = false;
		//printf("send indication success\n");
		break;


    default:
        break;
    }

    return result;
}

/*******************************************************************************
* Function Name: app_gatts_callback
********************************************************************************
* Summary:
* This function redirects the GATT requests to the appropriate functions
*
* Parameters:
*  wiced_bt_gatt_attribute_request_t *p_data: GATT request data structure
*
* Return:
*  wiced_bt_gatt_status_t: GATT result
*
*******************************************************************************/
wiced_bt_gatt_status_t app_gatts_callback(wiced_bt_gatt_evt_t event,
                       wiced_bt_gatt_event_data_t *p_data)
{
    wiced_bt_gatt_status_t result = WICED_BT_GATT_INVALID_PDU;

	//printf("app_gatts_callback:event=%d\n",event);

    switch (event)
    {
    case GATT_CONNECTION_STATUS_EVT:
        result = app_gatt_connect_callback(&p_data->connection_status);
        break;

    case GATT_ATTRIBUTE_REQUEST_EVT:
        result = app_gatts_req_cb(&p_data->attribute_request);
        break;

    case GATT_GET_RESPONSE_BUFFER_EVT:
        p_data->buffer_request.buffer.p_app_rsp_buffer = app_alloc_buffer(
                                          p_data->buffer_request.len_requested);
        p_data->buffer_request.buffer.p_app_ctxt = (void *)app_free_buffer;
        result = WICED_BT_GATT_SUCCESS;
        break;

    case GATT_APP_BUFFER_TRANSMITTED_EVT:
    {
        pfn_free_buffer_t pfn_free = (pfn_free_buffer_t)p_data->
                                      buffer_xmitted.p_app_ctxt;

        /* If the buffer is dynamic, the context will point to a function to
         * free it.
         */
        if (pfn_free)
            pfn_free(p_data->buffer_xmitted.p_app_data);

        result = WICED_BT_GATT_SUCCESS;
    }
    break;

	#if 1 //feichao
    case GATT_DISCONNECTION_STATUS_EVT:
        result = app_gatt_disconnect_callback(&p_data->connection_status);
        break;
	#endif

    default:
        break;
    }
    return result;
}


/*******************************************************************************
 * Function Name: app_alloc_buffer
 *******************************************************************************
 * Summary:
 *  This function allocates memory to the buffer.
 *
 *
 * Parameters:
 *  uint16_t len: Length of the memory required
 *
 * Return:
 *  uint8_t *: Pointer to the memory allocated
 *
 *******************************************************************************/
uint8_t *app_alloc_buffer(uint16_t len)
{
    uint8_t *p = (uint8_t *)malloc(len);
    return p;
}

/*******************************************************************************
 * Function Name: app_free_buffer
 *******************************************************************************
 * Summary:
 *  This function frees up the buffer memory.
 *
 *
 * Parameters:
 *  uint8_t *p_data: Pointer to the buffer to be free
 *
 *******************************************************************************/
void app_free_buffer(uint8_t *p_data)
{
    free(p_data);
}

#if 0//dennis
void read_host_info(void)
{
	wiced_result_t result;
	cy_en_em_eeprom_status_t eepromReturnValue;

	eepromReturnValue = Cy_Em_EEPROM_Read(LOGICAL_EEPROM_START, (void *)((uint8_t *)&linkkey_info.bd_addr[0]),
	                                              sizeof(wiced_bt_device_link_keys_t), &Em_EEPROM_context);
	if(CY_EM_EEPROM_SUCCESS == eepromReturnValue)
	{
#if 1
		for(uint8_t i=0;i<sizeof(wiced_bt_device_link_keys_t);i++)
		{
			printf("%02x ", *(((uint8_t*)&linkkey_info.bd_addr[0])+i));
		}
		printf("\r\n");

		
		printf("linkkey: ");
		for(uint8_t i=0;i<16;i++)
		{			
			printf("%02x ", *(((uint8_t*)&linkkey_info.key_data.le_keys.pltk[0])+i));
		}
		printf("\r\n");
#endif
	}

	#if 1
	if(linkkey_info.bd_addr[0]|linkkey_info.bd_addr[1]|linkkey_info.bd_addr[2]|linkkey_info.bd_addr[3]|linkkey_info.bd_addr[4]|linkkey_info.bd_addr[5])
	{
		 /* Add this peer device in the security database */
		result = wiced_bt_dev_add_device_to_address_resolution_db(&linkkey_info);
		if (result != WICED_BT_SUCCESS)
		{
			printf( "Err: wiced_bt_dev_add_device_to_address_resolution_db failed: %d\n", result);
		}
		else
		{
			printf( "add device to address resolution database success\r\n");
		}
	}
	#endif
}
#endif 

#if 1 //feichao
wiced_bt_gatt_status_t app_gatt_disconnect_callback(wiced_bt_gatt_connection_status_t *p_conn_status)
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_ERROR;
    wiced_result_t result;

    if (NULL != p_conn_status)
    {
        if (!p_conn_status->connected)
        {
            printf("\n feichao app_gatt_disconnect_callback: Peer BD Address: ");
            print_bd_address(p_conn_status->bd_addr);
            printf("\n");

            printf("feichao app_gatt_disconnect_callback Reason for disconnection: \t");
            printf(get_bt_gatt_disconn_reason_name(p_conn_status->reason));
            printf("\n");

            conn_id = 0;

            result = wiced_bt_ble_set_raw_advertisement_data(CY_BT_ADV_PACKET_DATA_SIZE, cy_bt_adv_packet_data);
            if(WICED_SUCCESS != result)
            {
                printf("Set ADV data failed\n");
            }

            wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL);
        }
        status = WICED_BT_GATT_SUCCESS;
    }

    return status;
}

static void send_notification(void)
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_SUCCESS;
	
	if(app_notify_service_client_char_config[0]|app_notify_service_client_char_config[1]<<8)
	{
		printf("Send notification\n");
		
		app_notify_service_chara[0] = 0x55;
												 
		wiced_bt_gatt_server_send_notification(conn_id, HDLC_NOTIFY_SERVICE_VALUE, 
		app_notify_service_chara[1]+2, app_notify_service_chara, NULL);

		if (WICED_BT_GATT_SUCCESS != status)
		{
			printf("Send notification failed\n");
		}
	}
	else
	{
		printf("notification is not enabled\n");
	}
}

static void send_indication(void)
{
	wiced_bt_gatt_status_t status = WICED_BT_GATT_SUCCESS;
	if(app_notify_service_client_char_config[0]|app_notify_service_client_char_config[1]<<8)
	{
		//printf("Send indication\n");
		
		app_notify_service_chara[0] = 0x55;
		wiced_bt_gatt_server_send_indication(conn_id, HDLC_NOTIFY_SERVICE_VALUE, 
		app_notify_service_chara[1]+2, app_notify_service_chara, NULL);
		
		if (WICED_BT_GATT_SUCCESS != status)
		{
			printf("Send indication failed\n");
		}
	}
	else
	{
		printf("indication is not enabled\n");
	}
}

void start_ble_adv(void)
{
	wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH,
                                            BLE_ADDR_PUBLIC, NULL);
}


#endif



/* [] END OF FILE */

