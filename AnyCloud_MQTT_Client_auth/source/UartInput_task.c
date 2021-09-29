/*******************************************************************************
 * File Name:   UartInput_task.c
 *
 * Description: This file contains the task definition for initializing the
 * Wi-Fi device, connecting to the AP, disconnecting from AP, scanning and 
 * EEPROM related functionality.
 *
 * Related Document: See Readme.md
 *
 *******************************************************************************
 * (c) 2020, Cypress Semiconductor Corporation. All rights reserved.
 *******************************************************************************
 * This software, including source code, documentation and related materials
 * ("Software"), is owned by Cypress Semiconductor Corporation or one of its
 * subsidiaries ("Cypress") and is protected by and subject to worldwide patent
 * protection (United States and foreign), United States copyright laws and
 * international treaty provisions. Therefore, you may use this Software only
 * as provided in the license agreement accompanying the software package from
 * which you obtained this Software ("EULA").
 *
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software source
 * code solely for use in connection with Cypress's integrated circuit products.
 * Any reproduction, modification, translation, compilation, or representation
 * of this Software except as specified above is prohibited without the express
 * written permission of Cypress.
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
 * including Cypress's product in a High Risk Product, the manufacturer of such
 * system or application assumes all risk of such use and in doing so agrees to
 * indemnify Cypress against all liability.
 ******************************************************************************/

#include <FreeRTOS.h>
#include <task.h>
#include "cybsp.h"
#include "stdio.h"
#include "math.h"

#include "cyhal_uart.h"
#include "cy_retarget_io.h"
#include "cy_em_eeprom.h"

#define UART_INTERRUPT_PRIORITY             (7)


uint8_t decryptPassword[6] = {0x38,0x67,0x84,0x29,0x31,0x72};
uint8_t encryptPassword[6] = {0x75,0x34,0x26,0x81,0x95,0x18};	

uint8_t                client_device_name[15];

uint8_t wifi_ssid_auth[20] = {'H','U','A','W','E','I',' ','M','a','t','e',' ','2','0','\0'};;
uint8_t wifi_password_auth[20] = {'7','4','1','8','b','5','2','2','a','5','b','3','\0'};
uint8_t wifi_security_auth[20] = {0};
uint8_t wifi_info[100];

uint8_t wifi_ssid_auth_len = 0;
uint8_t wifi_password_auth_len = 0;
uint8_t wifi_security_auth_len = 0;

uint8_t auth_res_data[15];
uint8_t auth_res_data_len = 0;

uint8_t auth_res_completed = 0;
extern uint8_t AP_connected;

extern uint8_t rng_data[10];

uint16_t auth_number = 0;

extern uint8_t auth_res_start;

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

uint8_t wait_auth_res_data = 0;
extern void publish_auth_res(void);

void encrypt(uint8_t* source, uint8_t* pass, uint8_t len)
{
	for(uint8_t i=0;i<len;++i)
	{
		source[i] ^= pass[i];
	}
}

//extern cyhal_uart_t cy_retarget_io_uart_obj;

/*******************************************************************************
* Function Name: UartInput_task
********************************************************************************
* Summary:
* This function initializes the WCM module and connects/disconnects to the WIFI AP
*
*******************************************************************************/
void UartInput_task(void * arg)
{
	uint32_t ulNotifiedValue;
	uint8_t uart_rx_buf[20];
	uint8_t uart_rx_len = 0;
	uint8_t i = 0;
	uint8_t j = 0;
	uint32_t k = 0;
	uint16_t time_count = 0;
	uint8_t item_counter = 0;	
	
	cy_en_em_eeprom_status_t eepromReturnValue;
	
	printf("UartInput_task\r\n");
	
	eepromReturnValue = Cy_Em_EEPROM_Init(&Em_EEPROM_config, &Em_EEPROM_context);
	if (CY_EM_EEPROM_SUCCESS != eepromReturnValue)
    {
        printf("Error initializing EMEEPROM\n");
        CY_ASSERT(0);
    }


	eepromReturnValue = Cy_Em_EEPROM_Read(LOGICAL_EEPROM_START, (void *)wifi_info,
	                                              sizeof(wifi_info), &Em_EEPROM_context);
	if(CY_EM_EEPROM_SUCCESS == eepromReturnValue)
	{
		for(i=0;i<sizeof(wifi_info);i++)
		{
			printf("%02x ", wifi_info[i]);
		}
		printf("\r\n");
		i = 0;
		
		if(wifi_info[0]==0)
		{
			printf("there is no wifi info in eeprom, please input wifi ssid\n");
		}
		else
		{
			wifi_ssid_auth_len = wifi_info[0];
			memcpy(wifi_ssid_auth,&wifi_info[1],wifi_ssid_auth_len);
			
			wifi_password_auth_len = wifi_info[wifi_ssid_auth_len+1];
			memcpy(wifi_password_auth,&wifi_info[wifi_ssid_auth_len+2],wifi_password_auth_len);
			
			wifi_security_auth_len = wifi_info[wifi_password_auth_len+wifi_ssid_auth_len+2];
			memcpy(wifi_security_auth,&wifi_info[wifi_password_auth_len+wifi_ssid_auth_len+3],wifi_security_auth_len);
			
			printf("wifi ssid:");
			for(j=0;j<wifi_ssid_auth_len;j++)
			{
				printf("%c",wifi_ssid_auth[j]);
			}
			printf("\r\n");
			//printf("wifi_ssid_auth_len = %d\n",wifi_ssid_auth_len);
			
			printf("wifi password:");
			for(j=0;j<wifi_password_auth_len;j++)
			{
				printf("%c",wifi_password_auth[j]);
			}
			printf("\r\n");
			//printf("wifi_password_auth_len = %d\n",wifi_password_auth_len);
			
			printf("wifi security:");
			for(j=0;j<wifi_security_auth_len;j++)
			{
				printf("%c",wifi_security_auth[j]);
			}
			printf("\r\n");
			//printf("wifi_security_auth_len = %d\n",wifi_security_auth_len);
		}
	}
	else
	{
		printf("read eeprom fail\n");
	}
	
    while(true)
    {
		#if 1
		if(1)//(wifi_info[0])
		{
			/* Wait for a notification */
			xTaskNotifyWait( 0x00,      /* Don't clear any notification bits on entry. */
							 UINT32_MAX, /* Reset the notification value to 0 on exit. */
							 &ulNotifiedValue, /* Notified value pass out in
												  ulNotifiedValue. */
							 1 );  /* Block indefinitely. */
							 
			if(k++>=3000)
			{
				k = 0;
				if(wait_auth_res_data)
				{
					if(auth_res_completed==0)
					{
						//printf("please input auth number\n");
					}
				}
				else
				{
					switch(item_counter)
					{
						case 0: 
						if(AP_connected==0)
						{
							printf("please input wifi ssid if want change wifi ssid\n");
						}
						break;
						
						case 1:
						printf("please input wifi password\n");
						break;
						
						case 2:
						printf("please input wifi security,please input 0 if open, please input 1 if WPA2\n");
						break;
					}
				}
			}
		}
		#endif
		
		//uart_rx_len = 0;
		//cyhal_uart_read(&cy_retarget_io_uart_obj,uart_rx_buf,&uart_rx_len);
		#if 1
		if (CY_RSLT_SUCCESS == cyhal_uart_getc(&cy_retarget_io_uart_obj,
                                               &uart_rx_buf[i], 5))
        {
			if(i==0)
			{
				time_count=0;
			}
			#if 0
            if (CY_RSLT_SUCCESS == cyhal_uart_putc(&cy_retarget_io_uart_obj,
                                                   uart_rx_buf[i]))
            {
				printf("\r\n");
            }
			#endif
			i++;
        }
        else
        {
			if(i)
			{
				time_count++;
				if(time_count>50)
				{
					#if 1//IS_AUTHORIZER
					if(wait_auth_res_data)
					{
						if(auth_res_completed==0)
						{
							auth_res_completed = 1;
							memset(auth_res_data,0,sizeof(auth_res_data));
							memcpy(auth_res_data,uart_rx_buf,i);
							auth_res_data_len = i;
							auth_number = 0;

							printf("auth_res_data_len=%d\n",auth_res_data_len);
							for(i=0;i<auth_res_data_len;i++)
							{
								auth_res_data[i] -= 0x30;
								if((auth_res_data_len-i)!=1)
								{
									auth_number += auth_res_data[i]*pow(10,(auth_res_data_len-1));
								}
								else
								{
									auth_number += auth_res_data[i];
								}
							}
							printf("auth_number=%d\n",auth_number);
							
							encrypt(rng_data,decryptPassword,5);
							
							auth_res_data[0] = auth_number;
							auth_res_data[1] = (auth_number>>8);
							
							auth_res_data[0] ^= rng_data[0];
							auth_res_data[1] ^= rng_data[1];							
							memcpy(&auth_res_data[2],&rng_data[2],3);							
							encrypt(auth_res_data,encryptPassword,5);							
							auth_res_data_len = 5;
							auth_res_start = 1;
							publish_auth_res();
							i = 0;							
						}
					}
					else
					#endif
					{
						if(item_counter==2)
						{
							memset(wifi_security_auth,0,sizeof(wifi_security_auth));
							memcpy(wifi_security_auth,uart_rx_buf,i);
							wifi_security_auth[i] = '\0';
							for(j=0;j<i;j++)
							{
								cyhal_uart_putc(&cy_retarget_io_uart_obj,
														   wifi_security_auth[j]);
							}
							printf("\r\n");
							printf("wirte wifi security success\n");
							wifi_security_auth_len = i;
							printf("wifi_security_auth_len = %d\n",wifi_security_auth_len);
							printf("please reset system\n");
							item_counter++;
							i = 0;
							
							memset(wifi_info,0,sizeof(wifi_info));
							wifi_info[0] = wifi_ssid_auth_len;
							memcpy(&wifi_info[1],wifi_ssid_auth,wifi_ssid_auth_len);
							wifi_info[wifi_ssid_auth_len+1] = wifi_password_auth_len;
							memcpy(&wifi_info[wifi_ssid_auth_len+2],wifi_password_auth,wifi_password_auth_len);
							wifi_info[wifi_password_auth_len+wifi_ssid_auth_len+2] = wifi_security_auth_len;
							memcpy(&wifi_info[wifi_password_auth_len+wifi_ssid_auth_len+3],wifi_security_auth,wifi_security_auth_len);
							
							#if 1//dennis
							eepromReturnValue = Cy_Em_EEPROM_Erase(&Em_EEPROM_context);
							if(CY_EM_EEPROM_SUCCESS != eepromReturnValue)
							{
								printf("Failed to erase EMEEPROM \n");
							}

							eepromReturnValue = Cy_Em_EEPROM_Write(LOGICAL_EEPROM_START,
																	(void *)wifi_info,
																	sizeof(wifi_info),
																	&Em_EEPROM_context);

							if(CY_EM_EEPROM_SUCCESS != eepromReturnValue)
							{
								printf("Failed to write to EMEEPROM \n");
							}
							#endif
						}
						
						if(item_counter==1)
						{
							memset(wifi_password_auth,0,sizeof(wifi_password_auth));
							memcpy(wifi_password_auth,uart_rx_buf,i);
							wifi_password_auth[i] = '\0';
							for(j=0;j<i;j++)
							{
								cyhal_uart_putc(&cy_retarget_io_uart_obj,
														   wifi_password_auth[j]);
							}
							printf("\r\n");
							printf("wirte wifi password success\n");
							wifi_password_auth_len = i;
							printf("wifi_password_auth_len = %d\n",wifi_password_auth_len);
							printf("\r\n");
							printf("\r\n");
							printf("\r\n");
							printf("please input wifi security\n");
							item_counter++;
							i = 0;
						}
						
						if(item_counter==0)
						{															
							memset(wifi_ssid_auth,0,sizeof(wifi_ssid_auth));
							memcpy(wifi_ssid_auth,uart_rx_buf,i);
							wifi_ssid_auth[i] = '\0';
							for(j=0;j<i;j++)
							{
								cyhal_uart_putc(&cy_retarget_io_uart_obj,
														   wifi_ssid_auth[j]);
							}
							printf("\r\n");
							printf("wirte wifi ssid success\n");
							wifi_ssid_auth_len = i;
							printf("wifi_ssid_auth_len = %d\n",wifi_ssid_auth_len);
							printf("\r\n");
							printf("\r\n");
							printf("\r\n");
							printf("please input wifi password\n");
							item_counter++;						
							i = 0;
						}
					}					
				}
			}
			//printf("gutc failed\r\n");
        }
		#endif       
	}
}

/* [] END OF FILE */
