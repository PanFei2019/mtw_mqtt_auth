/*
 * Copyright 2016-2021, Cypress Semiconductor Corporation (an Infineon company) or
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
 */

#include "wiced_bt_dev.h"
#include "wiced_hal_gpio.h"
#include "wiced_bt_trace.h"
#include "wiced_platform.h"

/* The pin P31 is also mapped to PUART_TX. Initializing the LED on P31 conflicts with PUART
 * operation. Hence, the second LED on this platform(P31) is not being initialized here
 * if the SDK user wants to reconfigure the platform to use the LED on P31 , the values for
 * second entry should be WICED_P31, GPIO_OUTPUT_ENABLE | GPIO_PULL_UP , GPIO_PIN_OUTPUT_LOW */
wiced_led_config_t platform_led[] =
{
   {
            .led_gpio = WICED_P26,
            .led_config = ( GPIO_OUTPUT_ENABLE | GPIO_PULL_UP ),
            .led_default_state = GPIO_PIN_OUTPUT_HIGH,
    }
};

/* Initialize LEDs and turn off */
void platform_led_init( void )
{
    uint32_t i = 0;

    /* Initialize LEDs and turn off by default */
    for(i=0; i < (sizeof(platform_led)/sizeof(wiced_led_config_t)); i++)
    {
        wiced_hal_gpio_configure_pin( platform_led[i].led_gpio, platform_led[i].led_config, platform_led[i].led_default_state);
    }
}
