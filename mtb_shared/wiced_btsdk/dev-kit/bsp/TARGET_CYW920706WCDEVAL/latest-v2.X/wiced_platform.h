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

/** @file
*
* Defines peripherals available for use on CYW920706WCDEVAL, and legacy BCM920706_P49 and BCM920706_B49 boards.
* Use LEGACY_BOARD=1 in the make target for the BCM920706_P49 and BCM920706_B49 boards
* Use B49=1 in the make target for BCM920706_B49 board.
*
*/

#pragma once

/** \addtogroup Platfrom config - Peripherals pin configuration
*   \ingroup HardwareDrivers
*/
/*! @{ */

/******************************************************
 *                   Enumerations
 ******************************************************/

#include "wiced_bt_types.h"
#include "wiced_hal_gpio.h"

extern void platform_led_init( void );

typedef struct
{
    wiced_bt_gpio_numbers_t led_gpio;
    uint32_t led_config;
    uint32_t led_default_state;
}wiced_led_config_t;

typedef enum
{
    WICED_PLATFORM_LED_1,
    WICED_PLATFORM_LED_MAX
}wiced_platform_led_t;

#define HCI_UART_DEFAULT_BAUD   3000000   /* default baud rate is 3M, that is max supported baud rate on Mac OS */

#ifdef LEGACY_BOARD
//P49_20706
#define WICED_GPIO_BUTTON                                   WICED_P30      /* pin for button interrupts */
#define WICED_GPIO_PIN_BUTTON                               WICED_GPIO_BUTTON

/* x can be GPIO_EN_INT_RISING_EDGE or GPIO_EN_INT_FALLING_EDGE or GPIO_EN_INT_BOTH_EDGE */
#define WICED_GPIO_BUTTON_SETTINGS(x)                       ( GPIO_INPUT_ENABLE | GPIO_PULL_DOWN | x )
/* if edge isn't specified (legacy/shared code), use GPIO_EN_INT_FALLING_EDGE */
#define WICED_GPIO_BUTTON_SETTINGS_DEFAULT                  ( GPIO_INPUT_ENABLE | GPIO_PULL_DOWN | GPIO_EN_INT_FALLING_EDGE )
#define WICED_GPIO_BUTTON_DEFAULT_STATE                     GPIO_PIN_OUTPUT_LOW

//P49_20706 (default)
#define WICED_BUTTON_PRESSED_VALUE                 1
#else
//CYW920706WCDEVAL
#define WICED_GPIO_BUTTON                                   WICED_P30      /* pin for button interrupts */
#define WICED_GPIO_PIN_BUTTON                               WICED_GPIO_BUTTON

/* x can be GPIO_EN_INT_RISING_EDGE or GPIO_EN_INT_FALLING_EDGE or GPIO_EN_INT_BOTH_EDGE */
#define WICED_GPIO_BUTTON_SETTINGS(x)                       ( GPIO_INPUT_ENABLE | GPIO_PULL_UP | x )
/* if edge isn't specified (legacy/shared code), use GPIO_EN_INT_RISING_EDGE */
#define WICED_GPIO_BUTTON_SETTINGS_DEFAULT                  ( GPIO_INPUT_ENABLE | GPIO_PULL_UP | GPIO_EN_INT_RISING_EDGE )
#define WICED_GPIO_BUTTON_DEFAULT_STATE                     GPIO_PIN_OUTPUT_HIGH

//P49_20706 (default)
#define WICED_BUTTON_PRESSED_VALUE                 0
#endif

#define WICED_PUART_TXD                           WICED_P31      /* pin for PUART TXD         */
#define WICED_PUART_RXD                           WICED_P33      /* pin for PUART RXD         */

/* @} */

/** \addtogroup Platfrom config - Default flash(i.e. flash exists on WICED eval boards) configuration.
*   \ingroup HardwareDrivers
*/
/*! @{ */
/******************************************************
 *                   Enumerations
 ******************************************************/
/**
 *  Recommend to use 4K sector flash.
 */
#if defined(USE_256K_SECTOR_SIZE)
  #define FLASH_SECTOR_SIZE         (256*1024)
  #define FLASH_SIZE                (256*FLASH_SECTOR_SIZE)
#else
  #define FLASH_SECTOR_SIZE         (4*1024)
  #define FLASH_SIZE                0x80000 // 512  kbyte/4M Bit Sflash for new tag boards
#endif

/** Number of sectors reserved from the end of the flash for the application
 *  specific purpose(for ex: to log the crash dump). By default no reservation
 Note:- 16K of flash is used for internal firmware operation.
 (Remaining of the flash - reservation) can be divided equally and used for active
 and upgradable firmware. So, application should care the OTA firmware(application+patch)
 size while reserving using below.
 */
#define APPLICATION_SPECIFIC_FLASH_RESERVATION  0

/* @} */
