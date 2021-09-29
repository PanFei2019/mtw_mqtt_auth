/******************************************************************************
* File Name:   subscriber_task.c
*
* Description: This file contains the task that initializes the user LED GPIO,
*              subscribes to the topic 'MQTT_TOPIC', and actuates the user LED
*              based on the notifications received from the MQTT subscriber
*              callback.
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
#include "string.h"
#include "FreeRTOS.h"

/* Task header files */
#include "subscriber_task.h"
#include "mqtt_task.h"

/* Configuration file for MQTT client */
#include "mqtt_client_config.h"

/* Middleware libraries */
#include "cy_retarget_io.h"
#include "iot_mqtt.h"

/******************************************************************************
* Macros
******************************************************************************/
/* The number of MQTT topics to be subscribed to. */
#define SUBSCRIPTION_COUNT          (1)

/******************************************************************************
* Function Prototypes
*******************************************************************************/
static void mqtt_subscription_callback(void *pCallbackContext,
                                       IotMqttCallbackParam_t *pPublishInfo);

#if 1//dennis
void publish_ping_ack(void);
void publish_auth_req(void);
void publish_auth_res(void);
void publish_auth_cpt(void);
void publish_auth_cpt_ack(void);
extern TaskHandle_t publisher_task_handle;
extern TimerHandle_t  PublishTimer;
uint8_t ping_ack_flag = 0;
uint8_t auth_req_flag = 0;
uint8_t auth_res_flag = 0;
uint8_t auth_cpt_flag = 0;
uint8_t auth_cpt_ack_flag = 0;
extern uint8_t wait_auth_res_data;
extern uint8_t start_indication;
extern uint8_t auth_req_start;
extern uint8_t auth_res_start;
extern uint8_t auth_cpt_start;
extern uint8_t auth_cpt_ack_start;
extern uint8_t app_notify_service_chara[];
uint8_t rng_data[10];
#endif

/******************************************************************************
* Global Variables
*******************************************************************************/
/* Task handle for this task. */
TaskHandle_t subscriber_task_handle;

/* Variable to denote the current state of the user LED that is also used by 
 * the publisher task.
 */
uint32_t current_device_state = DEVICE_OFF_STATE;

/* Configure the subscription information structure. */
IotMqttSubscription_t subscribeInfo =
{
    .qos = (IotMqttQos_t) MQTT_MESSAGES_QOS,
	#if 0//dennis
    .pTopicFilter = MQTT_TOPIC,
    .topicFilterLength = (sizeof(MQTT_TOPIC) - 1),
    #else
	#if 1//IS_AUTHORIZER
	.pTopicFilter = MQTT_TOPIC_AUTHORIZEE,
    .topicFilterLength = (sizeof(MQTT_TOPIC_AUTHORIZEE) - 1),
	#else
	.pTopicFilter = MQTT_TOPIC_AUTHORIZER,
    .topicFilterLength = (sizeof(MQTT_TOPIC_AUTHORIZER) - 1),
    #endif
	#endif
    /* Configure the callback function to handle incoming MQTT messages */
    .callback.function = mqtt_subscription_callback
};

/******************************************************************************
 * Function Name: subscriber_task
 ******************************************************************************
 * Summary:
 *  Task that sets up the user LED GPIO, subscribes to topic - 'MQTT_TOPIC',
 *  and controls the user LED based on the received task notification.
 *
 * Parameters:
 *  void *pvParameters : Task parameter defined during task creation (unused)
 *
 * Return:
 *  void
 *
 ******************************************************************************/
void subscriber_task(void *pvParameters)
{
    /* Status variable */
    int result = EXIT_SUCCESS;

    /* Variable to denote received LED state. */
    uint32_t received_led_state;

    /* Status of MQTT subscribe operation that will be communicated to MQTT 
     * client task using a message queue in case of failure in subscription.
     */
    mqtt_result_t mqtt_subscribe_status = MQTT_SUBSCRIBE_FAILURE;

    /* To avoid compiler warnings */
    (void)pvParameters;

    /* Initialize the User LED. */
    cyhal_gpio_init(CYBSP_USER_LED, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_PULLUP,
                    CYBSP_LED_STATE_OFF);

    /* Subscribe with the configured parameters. */
    result = IotMqtt_SubscribeSync(mqttConnection,
                                   &subscribeInfo,
                                   SUBSCRIPTION_COUNT,
                                   0,
                                   MQTT_TIMEOUT_MS);
    if (result != EXIT_SUCCESS)
    {
        /* Notify the MQTT client task about the subscription failure and  
         * suspend the task for it to be killed by the MQTT client task later.
         */
        printf("MQTT Subscribe failed with error '%s'.\n\n",
               IotMqtt_strerror((IotMqttError_t) result));
        xQueueOverwrite(mqtt_status_q, &mqtt_subscribe_status);
        vTaskSuspend( NULL );
    }

    printf("MQTT client subscribed to the topic '%.*s' successfully.\n\n", 
           subscribeInfo.topicFilterLength, subscribeInfo.pTopicFilter);

    while (true)
    {
        /* Block till a notification is received from the subscriber callback. */
        xTaskNotifyWait(0, 0, &received_led_state, portMAX_DELAY);
        /* Update the LED state as per received notification. */
		#if 0//dennis
        cyhal_gpio_write(CYBSP_USER_LED, received_led_state);
        /* Update the current device state extern variable. */
        current_device_state = received_led_state;
		#endif
    }
}

/******************************************************************************
 * Function Name: mqtt_subscription_callback
 ******************************************************************************
 * Summary:
 *  Callback to handle incoming MQTT messages. This callback prints the 
 *  contents of an incoming message and notifies the subscriber task with the  
 *  LED state as per the received message.
 *
 * Parameters:
 *  void *pCallbackContext : Parameter defined during MQTT Subscribe operation
 *                           using the IotMqttCallbackInfo_t.pCallbackContext
 *                           member (unused)
 *  IotMqttCallbackParam_t * pPublishInfo : Information about the incoming 
 *                                          MQTT PUBLISH message passed by
 *                                          the MQTT library.
 *
 * Return:
 *  void
 *
 ******************************************************************************/
static void mqtt_subscription_callback(void *pCallbackContext,
                                       IotMqttCallbackParam_t *pPublishInfo)
{

	#if 1//dennis
	static uint32_t m_led_status = 0;
	uint8_t i = 0;
	#endif
	
    /* Received MQTT message */
    const char *pPayload = pPublishInfo->u.message.info.pPayload;
    /* LED state that should be sent to LED task depending on received message. */
    uint32_t subscribe_led_state = DEVICE_OFF_STATE;

    /* To avoid compiler warnings */
    (void) pCallbackContext;

	#if 0
    /* Print information about the incoming PUBLISH message. */
    printf("Incoming MQTT message received:\n"
           "Subscription topic filter: %.*s\n"
           "Published topic name: %.*s\n"
           "Published QoS: %d\n"
           "Published payload: %.*s\n\n",
           pPublishInfo->u.message.topicFilterLength,
           pPublishInfo->u.message.pTopicFilter,
           pPublishInfo->u.message.info.topicNameLength,
           pPublishInfo->u.message.info.pTopicName,
           pPublishInfo->u.message.info.qos,
           pPublishInfo->u.message.info.payloadLength,
           pPayload);
	#endif

	#if 0//dennis
    /* Assign the LED state depending on the received MQTT message. */
    if ((strlen(MQTT_DEVICE_ON_MESSAGE) == pPublishInfo->u.message.info.payloadLength) &&
        (strncmp(MQTT_DEVICE_ON_MESSAGE, pPayload, pPublishInfo->u.message.info.payloadLength) == 0))
    {
        subscribe_led_state = DEVICE_ON_STATE;
    }
    else if ((strlen(MQTT_DEVICE_OFF_MESSAGE) == pPublishInfo->u.message.info.payloadLength) &&
             (strncmp(MQTT_DEVICE_OFF_MESSAGE, pPayload, pPublishInfo->u.message.info.payloadLength) == 0))
    {
        subscribe_led_state = DEVICE_OFF_STATE;
    }
    else
    {
        printf("Received MQTT message not in valid format!\n");
        return;
    }
	 /* Notify the subscriber task about the received LED control message. */
    xTaskNotify(subscriber_task_handle, subscribe_led_state, eSetValueWithoutOverwrite);
	publish_ping_ack();
	#else

	#if 1//IS_AUTHORIZER
	if ((strlen(MQTT_PING_MESSAGE) == pPublishInfo->u.message.info.payloadLength) &&
        (strncmp(MQTT_PING_MESSAGE, pPayload, pPublishInfo->u.message.info.payloadLength) == 0))
    {
    	printf("MQTT_PING_MESSAGE\r\n");
    	if(m_led_status)
    	{
    		subscribe_led_state = m_led_status;
			m_led_status = 0;
    	}
		else
		{
			subscribe_led_state = m_led_status;
			m_led_status = 1;
		}
		publish_ping_ack();
		 /* Notify the subscriber task about the received LED control message. */
	    //xTaskNotify(subscriber_task_handle, subscribe_led_state, eSetValueWithoutOverwrite);
		
    }
	else if (strncmp(MQTT_AUTH_REQ_MESSAGE, pPayload, 8) == 0)
    {
    	if(auth_req_flag==0)
    	{
    		auth_req_flag = 1;
			printf("\n\n\n\n");
	    	printf("MQTT_AUTH_REQ_MESSAGE:");
			for(i=10;i<pPublishInfo->u.message.info.payloadLength-1;i++)
			{
				rng_data[i-10] = *(pPayload+i);
				printf("%02x ",*(pPayload+i));
			}
			printf("\r\n");
			printf("device number:%d\n",(*(pPayload+pPublishInfo->u.message.info.payloadLength-1))^0x72);
			printf("remaining number = %d\n",(*(pPayload+8))|(uint16_t)(*(pPayload+9))<<8);	
			printf("please input auth number\n");
			printf("\n\n");
    	}
		wait_auth_res_data = 1;
		//publish_auth_res();
    }	
	else if (strncmp(MQTT_AUTH_CPT_MESSAGE, pPayload, 8) == 0)
    {
    	if(auth_cpt_flag==0)
    	{
    		auth_cpt_flag = 1;
			printf("\n\n*****************************************************************************\n");
    		printf("MQTT_AUTH_CPT_MESSAGE:");  
			for(i=10;i<pPublishInfo->u.message.info.payloadLength;i++)
			{
				printf("%02x ",*(pPayload+i));
			}
			printf("\r\n");
			printf("device number:%d\n",*(pPayload+pPublishInfo->u.message.info.payloadLength-1));
			printf("remaining number = %d\n",(*(pPayload+8))|(uint16_t)(*(pPayload+9))<<8);	
			printf("*****************************************************************************\n\n\n\n\n");
			auth_res_start = 0;
			auth_cpt_ack_start = 1;
			publish_auth_cpt_ack();
    	}
		cyhal_gpio_write(CYBSP_USER_LED, DEVICE_ON_STATE);
    }	

	//#else
	if ((strlen(MQTT_PING_ACK_MESSAGE) == pPublishInfo->u.message.info.payloadLength) &&
        (strncmp(MQTT_PING_ACK_MESSAGE, pPayload, pPublishInfo->u.message.info.payloadLength) == 0))
    {
    	xTimerStop(PublishTimer,0);
		cyhal_gpio_write(CYBSP_USER_LED2, DEVICE_ON_STATE);
    	printf("MQTT_PING_ACK_MESSAGE\r\n");
    	if(m_led_status)
    	{
    		subscribe_led_state = m_led_status;
			m_led_status = 0;
    	}
		else
		{
			subscribe_led_state = m_led_status;
			m_led_status = 1;
		}
		 /* Notify the subscriber task about the received LED control message. */
	    xTaskNotify(subscriber_task_handle, subscribe_led_state, eSetValueWithoutOverwrite);
		publish_auth_req();
    }
	else if (strncmp(MQTT_AUTH_RES_MESSAGE, pPayload, 8) == 0)
    {
    	if(auth_res_flag==0)
    	{
    		auth_res_flag = 1;
			//printf("MQTT_AUTH_RES_MESSAGE:");
			for(i=8;i<pPublishInfo->u.message.info.payloadLength;i++)
			{
				app_notify_service_chara[i-6] = *(pPayload+i);
				//printf("%02x ",*(pPayload+i));
			}
			//printf("\r\n");
			app_notify_service_chara[1] = pPublishInfo->u.message.info.payloadLength - 8;
			start_indication = true;
			auth_req_start = 0;
    	}
	}
	else if (strncmp(MQTT_AUTH_CPT_ACK_MESSAGE, pPayload, 8) == 0)
	{
		if(auth_cpt_ack_flag==0)
		{
			auth_cpt_ack_flag = 1;
			//printf("MQTT_AUTH_CPT_ACK_MESSAGE\r\n");
			auth_cpt_start = 0;
			xTimerStop(PublishTimer,0);
			cyhal_gpio_write(CYBSP_USER_LED, DEVICE_ON_STATE);
			cyhal_gpio_write(CYBSP_USER_LED2, DEVICE_ON_STATE);
		}		
	}
		
	#endif
	#endif

   
}

/******************************************************************************
 * Function Name: mqtt_unsubscribe
 ******************************************************************************
 * Summary:
 *  Function that unsubscribes from the topic specified by the macro 
 *  'MQTT_TOPIC'. This operation is called during cleanup by the MQTT client 
 *  task.
 *
 * Parameters:
 *  void 
 *
 * Return:
 *  void 
 *
 ******************************************************************************/
void mqtt_unsubscribe(void)
{
    IotMqttError_t result = IotMqtt_UnsubscribeSync(mqttConnection,
                                                    &subscribeInfo,
                                                    SUBSCRIPTION_COUNT,
                                                    0,
                                                    MQTT_TIMEOUT_MS);
    if (result != IOT_MQTT_SUCCESS)
    {
        printf("MQTT Unsubscribe operation failed with error '%s'!\n",
               IotMqtt_strerror(result));
    }
}

#if 1//dennis
void publish_ping(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    /* Notify the publisher task about the new state to be published. */
    xTaskNotifyFromISR(publisher_task_handle, PING_CMD, eSetValueWithoutOverwrite,
                       &xHigherPriorityTaskWoken);

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}



void publish_ping_ack(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	
#if 0
    uint32_t new_device_state;
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
#endif
    /* Notify the publisher task about the new state to be published. */
    xTaskNotifyFromISR(publisher_task_handle, PING_ACK_CMD, eSetValueWithoutOverwrite,
                       &xHigherPriorityTaskWoken);

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void publish_auth_req(void)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	 /* Notify the publisher task about the new state to be published. */
    xTaskNotifyFromISR(publisher_task_handle, AUTH_REQ_CMD, eSetValueWithoutOverwrite,
                       &xHigherPriorityTaskWoken);

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void publish_auth_res(void)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	 /* Notify the publisher task about the new state to be published. */
    xTaskNotifyFromISR(publisher_task_handle, AUTH_RES_CMD, eSetValueWithoutOverwrite,
                       &xHigherPriorityTaskWoken);

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void publish_auth_cpt(void)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	 /* Notify the publisher task about the new state to be published. */
    xTaskNotifyFromISR(publisher_task_handle, AUTH_CPT_CMD, eSetValueWithoutOverwrite,
                       &xHigherPriorityTaskWoken);

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}


void publish_auth_cpt_ack(void)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	 /* Notify the publisher task about the new state to be published. */
    xTaskNotifyFromISR(publisher_task_handle, AUTH_CPT_ACK_CMD, eSetValueWithoutOverwrite,
                       &xHigherPriorityTaskWoken);

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}



#endif

/* [] END OF FILE */
