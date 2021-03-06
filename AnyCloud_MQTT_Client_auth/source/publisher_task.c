/******************************************************************************
* File Name:   publisher_task.c
*
* Description: This file contains the task that initializes the user button
*              GPIO, configures the ISR, and publishes MQTT messages on the 
*              topic 'MQTT_TOPIC' to control a device that is actuated by the
*              subscriber task. The file also contains the ISR that notifies
*              the publisher task about the new device state to be published.
*
* Related Document: See README.md
*
*******************************************************************************
* (c) 2020-2021, Cypress Semiconductor Corporation. All rights reserved.
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
*******************************************************************************/

#include "cyhal.h"
#include "cybsp.h"
#include "FreeRTOS.h"

/* Task header files */
#include "publisher_task.h"
#include "mqtt_task.h"
#include "subscriber_task.h"

/* Configuration file for MQTT client */
#include "mqtt_client_config.h"

/* Middleware libraries */
#include "cy_retarget_io.h"
#include "iot_mqtt.h"

#if 1//dennis
TimerHandle_t  PublishTimer = NULL;
char *MQTT_AUTH_REQ_DATA = NULL;
char *MQTT_AUTH_RES_DATA = NULL;
char *MQTT_AUTH_CPT_DATA = NULL;

extern uint8_t auth_data[15];
extern uint8_t auth_data_len;

extern uint8_t auth_res_data[15];
extern uint8_t auth_res_data_len;

extern void start_ble_adv(void);
#endif

/******************************************************************************
* Macros
******************************************************************************/
/* Interrupt priority for User Button Input. */
#define USER_BTN_INTR_PRIORITY          (5)

/* The maximum number of times each PUBLISH in this example will be retried. */
#define PUBLISH_RETRY_LIMIT             (10)

/* A PUBLISH message is retried if no response is received within this 
 * time (in milliseconds).
 */
#define PUBLISH_RETRY_MS                (1000)

/******************************************************************************
* Global Variables
*******************************************************************************/
/* FreeRTOS task handle for this task. */
TaskHandle_t publisher_task_handle;

/* Structure to store publish message information. */
IotMqttPublishInfo_t publishInfo =
{
    .qos = (IotMqttQos_t) MQTT_MESSAGES_QOS,
	#if 0//dennis
    .pTopicName = MQTT_TOPIC,
    .topicNameLength = (sizeof(MQTT_TOPIC) - 1),
    #else
	#if IS_AUTHORIZER
	.pTopicName = MQTT_TOPIC_AUTHORIZER,
	.topicNameLength = (sizeof(MQTT_TOPIC_AUTHORIZER) - 1),
	#else
	.pTopicName = MQTT_TOPIC_AUTHORIZEE,
	.topicNameLength = (sizeof(MQTT_TOPIC_AUTHORIZEE) - 1),
	#endif
	#endif  
    .retryMs = PUBLISH_RETRY_MS,
    .retryLimit = PUBLISH_RETRY_LIMIT
};

/******************************************************************************
* Function Prototypes
*******************************************************************************/
void isr_button_press(void *callback_arg, cyhal_gpio_event_t event);

#if 1//dennis
static void PublishTimerCallback(TimerHandle_t xTimer)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	uint32_t new_device_state;
	static uint8_t ping_flag = 0;
	static uint8_t time_counter = 0;
	
	(void) xTimer;
	/* Toggle the device state. */
    if (current_device_state == DEVICE_ON_STATE)
    {
        new_device_state = DEVICE_OFF_STATE;
    }
    else
    {
        new_device_state = DEVICE_ON_STATE;
    }

	#if 1//dennis
	current_device_state = new_device_state;
	cyhal_gpio_write(CYBSP_USER_LED2, new_device_state);
	#endif

	
	#if 0
	if(ping_flag==0)
	{
		ping_flag = 1;
		/* Notify the publisher task about the new state to be published. */
		xTaskNotifyFromISR(publisher_task_handle, PING_CMD, eSetValueWithoutOverwrite,
		                   &xHigherPriorityTaskWoken);

		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
	#endif

	if(time_counter++>=5)
	{
		time_counter = 0;
		/* Notify the publisher task about the new state to be published. */
		xTaskNotifyFromISR(publisher_task_handle, PING_CMD, eSetValueWithoutOverwrite,
		                   &xHigherPriorityTaskWoken);

		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		//xTimerStop(PublishTimer,0);
		//cyhal_gpio_write(CYBSP_USER_LED2, DEVICE_OFF_STATE);
	}
}               
#endif

/******************************************************************************
 * Function Name: publisher_task
 ******************************************************************************
 * Summary:
 *  Task that handles initialization of the user button GPIO, configuration of
 *  ISR, and publishing of MQTT messages to control the device that is actuated
 *  by the subscriber task.
 *
 * Parameters:
 *  void *pvParameters : Task parameter defined during task creation (unused)
 *
 * Return:
 *  void
 *
 ******************************************************************************/
void publisher_task(void *pvParameters)
{
	uint8_t i = 0;
    /* Status variable */
    int result;

    /* Variable to receive new device state from the user button ISR. */
    uint32_t publish_device_state;

    /* Status of MQTT publish operation that will be communicated to MQTT 
     * client task using a message queue in case of failure during publish.
     */
    mqtt_result_t mqtt_publish_status = MQTT_PUBLISH_FAILURE;

    /* To avoid compiler warnings */
    (void)pvParameters;

    /* Initialize the user button GPIO and register interrupt on falling edge. */
    cyhal_gpio_init(CYBSP_USER_BTN, CYHAL_GPIO_DIR_INPUT,
                    CYHAL_GPIO_DRIVE_PULLUP, CYBSP_BTN_OFF);
    cyhal_gpio_register_callback(CYBSP_USER_BTN, isr_button_press, NULL);
    cyhal_gpio_enable_event(CYBSP_USER_BTN, CYHAL_GPIO_IRQ_FALL,
                            USER_BTN_INTR_PRIORITY, true);
    
    //printf("Press the user button (SW2) to publish \"%s\"/\"%s\" on the topic '%s'...\n\n", 
    //       MQTT_DEVICE_ON_MESSAGE, MQTT_DEVICE_OFF_MESSAGE, publishInfo.pTopicName);
		   
	start_ble_adv();

	#if !IS_AUTHORIZER//dennis
    if(PublishTimer ==NULL)  PublishTimer = xTimerCreate("PublishTimer",pdMS_TO_TICKS(1000),pdTRUE,( void * )0,PublishTimerCallback);
	xTimerStop(PublishTimer,0);
	//xTimerStart(PublishTimer, 0);
	#endif	

	MQTT_AUTH_REQ_DATA = malloc(9+15);
	*MQTT_AUTH_REQ_DATA = 'A';
	*(MQTT_AUTH_REQ_DATA+1) = 'U';
	*(MQTT_AUTH_REQ_DATA+2) = 'T';
	*(MQTT_AUTH_REQ_DATA+3) = 'H';
	*(MQTT_AUTH_REQ_DATA+4) = ' ';
	*(MQTT_AUTH_REQ_DATA+5) = 'R';
	*(MQTT_AUTH_REQ_DATA+6) = 'E';
	*(MQTT_AUTH_REQ_DATA+7) = 'Q';
	*(MQTT_AUTH_REQ_DATA+8) = '1';
	*(MQTT_AUTH_REQ_DATA+9) = '2';
	*(MQTT_AUTH_REQ_DATA+10) = '3';
	*(MQTT_AUTH_REQ_DATA+11) = '\0';

	MQTT_AUTH_RES_DATA = malloc(9+15);
	*MQTT_AUTH_RES_DATA = 'A';
	*(MQTT_AUTH_RES_DATA+1) = 'U';
	*(MQTT_AUTH_RES_DATA+2) = 'T';
	*(MQTT_AUTH_RES_DATA+3) = 'H';
	*(MQTT_AUTH_RES_DATA+4) = ' ';
	*(MQTT_AUTH_RES_DATA+5) = 'R';
	*(MQTT_AUTH_RES_DATA+6) = 'E';
	*(MQTT_AUTH_RES_DATA+7) = 'S';
	*(MQTT_AUTH_RES_DATA+8) = '4';
	*(MQTT_AUTH_RES_DATA+9) = '5';
	*(MQTT_AUTH_RES_DATA+10) = '6';
	*(MQTT_AUTH_RES_DATA+11) = '\0';
	
	MQTT_AUTH_CPT_DATA = malloc(9+15);
	*MQTT_AUTH_CPT_DATA = 'A';
	*(MQTT_AUTH_CPT_DATA+1) = 'U';
	*(MQTT_AUTH_CPT_DATA+2) = 'T';
	*(MQTT_AUTH_CPT_DATA+3) = 'H';
	*(MQTT_AUTH_CPT_DATA+4) = ' ';
	*(MQTT_AUTH_CPT_DATA+5) = 'C';
	*(MQTT_AUTH_CPT_DATA+6) = 'P';
	*(MQTT_AUTH_CPT_DATA+7) = 'T';
	*(MQTT_AUTH_CPT_DATA+8) = '4';
	*(MQTT_AUTH_CPT_DATA+9) = '5';
	*(MQTT_AUTH_CPT_DATA+10) = '6';
	*(MQTT_AUTH_CPT_DATA+11) = '\0';

	
    while (true)
    {
        /* Wait for notification from the User Button ISR. */
        xTaskNotifyWait(0, 0, &publish_device_state, portMAX_DELAY);

		#if 0//dennis
        /* Assign the publish message payload according to received device state. */
        publishInfo.pPayload = MQTT_DEVICE_OFF_MESSAGE;
        
        if (publish_device_state == DEVICE_ON_STATE)
        {
            publishInfo.pPayload = MQTT_DEVICE_ON_MESSAGE;
        }
		publishInfo.payloadLength = strlen(publishInfo.pPayload);
		#else		
		switch(publish_device_state)
		{
			case PING_CMD:
			publishInfo.pPayload = MQTT_PING_MESSAGE;
			publishInfo.payloadLength = strlen(publishInfo.pPayload);
			break;
			case PING_ACK_CMD:
			publishInfo.pPayload = MQTT_PING_ACK_MESSAGE;
			publishInfo.payloadLength = strlen(publishInfo.pPayload);
			break;
			case AUTH_REQ_CMD:
			for(i=0;i<auth_data_len;i++)
			{
				*(MQTT_AUTH_REQ_DATA+8+i) = auth_data[i];
			}
			*(MQTT_AUTH_REQ_DATA+8+i) = '\0';
			
			publishInfo.pPayload = MQTT_AUTH_REQ_DATA;
			publishInfo.payloadLength = 8+auth_data_len;
			break;
			case AUTH_RES_CMD:
			for(i=0;i<auth_res_data_len;i++)
			{
				*(MQTT_AUTH_RES_DATA+8+i) = auth_res_data[i];
			}
			*(MQTT_AUTH_RES_DATA+8+i) = '\0';
			publishInfo.pPayload = MQTT_AUTH_RES_DATA;
			publishInfo.payloadLength = 8+auth_res_data_len;
			break;
			case AUTH_CPT_CMD:
			for(i=0;i<auth_data_len;i++)
			{
				*(MQTT_AUTH_CPT_DATA+8+i) = auth_data[i];
			}
			*(MQTT_AUTH_CPT_DATA+8+i) = '\0';
			publishInfo.pPayload = MQTT_AUTH_CPT_DATA;
			publishInfo.payloadLength = 8+auth_data_len;
			break;
			case AUTH_CPT_ACK_CMD:
			publishInfo.pPayload = MQTT_AUTH_CPT_ACK_MESSAGE;
			publishInfo.payloadLength = strlen(publishInfo.pPayload);
			break;
			default: break;
		}
		
		#endif
        
		#if 0//IS_AUTHORIZER
        printf("Publishing '%s' on the topic '%s'\n\n",
               (char *)publishInfo.pPayload,
               publishInfo.pTopicName);
		#endif

        /* Publish the MQTT message with the configured settings. */
        result = IotMqtt_PublishSync(mqttConnection,
                                     &publishInfo,
                                     0,
                                     MQTT_TIMEOUT_MS);

        if (result != IOT_MQTT_SUCCESS)
        {
            /* Inform the MQTT client task about the publish failure and suspend
             * the task for it to be killed by the MQTT client task later.
             */
            printf("MQTT Publish failed with error '%s'.\n\n",
                   IotMqtt_strerror((IotMqttError_t) result));
            xQueueOverwrite(mqtt_status_q, &mqtt_publish_status);
            vTaskSuspend( NULL );
        }
    }
	free(MQTT_AUTH_REQ_DATA);
	free(MQTT_AUTH_RES_DATA);
	free(MQTT_AUTH_CPT_DATA);
}

/******************************************************************************
 * Function Name: isr_button_press
 ******************************************************************************
 * Summary:
 *  GPIO interrupt service routine. This function detects button
 *  presses and sends task notifications to the publisher task about the new 
 *  device state that needs to be published.
 *
 * Parameters:
 *  void *callback_arg : pointer to variable passed to the ISR (unused)
 *  cyhal_gpio_event_t event : GPIO event type (unused)
 *
 * Return:
 *  void
 *
 ******************************************************************************/
void isr_button_press(void *callback_arg, cyhal_gpio_event_t event)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint32_t new_device_state;

    /* To avoid compiler warnings */
    (void) callback_arg;
    (void) event;

    /* Toggle the device state. */
    if (current_device_state == DEVICE_ON_STATE)
    {
        new_device_state = DEVICE_OFF_STATE;
    }
    else
    {
        new_device_state = DEVICE_ON_STATE;
    }

	#if 1//dennis
	current_device_state = new_device_state;
	#endif

    /* Notify the publisher task about the new state to be published. */
    xTaskNotifyFromISR(publisher_task_handle, new_device_state, eSetValueWithoutOverwrite,
                       &xHigherPriorityTaskWoken);

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/******************************************************************************
 * Function Name: publisher_cleanup
 ******************************************************************************
 * Summary:
 *  Cleanup function for the publisher task that disables the user button  
 *  interrupt. This operation needs to be necessarily performed before deleting  
 *  the publisher task.
 *
 * Parameters:
 *  void
 *
 * Return:
 *  void
 *
 ******************************************************************************/
void publisher_cleanup(void)
{
    /* Deregister the ISR and disable the interrupt on the user button. */
    cyhal_gpio_register_callback(CYBSP_USER_BTN, NULL, NULL);
    cyhal_gpio_enable_event(CYBSP_USER_BTN, CYHAL_GPIO_IRQ_FALL,
                            USER_BTN_INTR_PRIORITY, false);
}

/* [] END OF FILE */
