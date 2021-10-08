/*******************************************************************************
* File Name: cycfg_pins.c
*
* Description:
* Pin configuration
* This file was automatically generated and should not be modified.
* Tools Package 2.2.0.2801
* COMPONENT_20721B2 
* personalities 1.0.0.31
* udd 3.0.0.775
*
********************************************************************************
* Copyright 2021 Cypress Semiconductor Corporation
* SPDX-License-Identifier: Apache-2.0
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
********************************************************************************/

#include "cycfg_pins.h"

#define BUTTON_USER_config \
{\
    .gpio = (wiced_bt_gpio_numbers_t*)&platform_gpio_pins[PLATFORM_GPIO_0].gpio_pin, \
    .config = GPIO_INPUT_ENABLE | GPIO_PULL_UP, \
    .default_state = GPIO_PIN_OUTPUT_HIGH, \
    .button_pressed_value = GPIO_PIN_OUTPUT_LOW, \
}
#define BUTTON_VOL_UP_config \
{\
    .gpio = (wiced_bt_gpio_numbers_t*)&platform_gpio_pins[PLATFORM_GPIO_8].gpio_pin, \
    .config = GPIO_INPUT_ENABLE | GPIO_PULL_UP, \
    .default_state = GPIO_PIN_OUTPUT_HIGH, \
    .button_pressed_value = GPIO_PIN_OUTPUT_LOW, \
}
#define LED2_config \
{\
    .gpio = (wiced_bt_gpio_numbers_t*)&platform_gpio_pins[PLATFORM_GPIO_11].gpio_pin, \
    .config = GPIO_OUTPUT_ENABLE | GPIO_PULL_UP, \
    .default_state = GPIO_PIN_OUTPUT_HIGH, \
 }
#define LED1_config \
{\
    .gpio = (wiced_bt_gpio_numbers_t*)&platform_gpio_pins[PLATFORM_GPIO_12].gpio_pin, \
    .config = GPIO_OUTPUT_ENABLE | GPIO_PULL_UP, \
    .default_state = GPIO_PIN_OUTPUT_HIGH, \
 }
#define BUTTON_VOL_DN_config \
{\
    .gpio = (wiced_bt_gpio_numbers_t*)&platform_gpio_pins[PLATFORM_GPIO_1].gpio_pin, \
    .config = GPIO_INPUT_ENABLE | GPIO_PULL_UP, \
    .default_state = GPIO_PIN_OUTPUT_HIGH, \
    .button_pressed_value = GPIO_PIN_OUTPUT_LOW, \
}
#define BUTTON_CUSTOM_config \
{\
    .gpio = (wiced_bt_gpio_numbers_t*)&platform_gpio_pins[PLATFORM_GPIO_2].gpio_pin, \
    .config = GPIO_INPUT_ENABLE | GPIO_PULL_UP, \
    .default_state = GPIO_PIN_OUTPUT_HIGH, \
    .button_pressed_value = GPIO_PIN_OUTPUT_LOW, \
}

const wiced_platform_gpio_t platform_gpio_pins[] = 
{
	[PLATFORM_GPIO_0] = {WICED_P00, WICED_GPIO},
	[PLATFORM_GPIO_1] = {WICED_P04, WICED_GPIO},
	[PLATFORM_GPIO_2] = {WICED_P05, WICED_GPIO},
	[PLATFORM_GPIO_3] = {WICED_P06, audio_0_do_out_0_TRIGGER_IN},
	[PLATFORM_GPIO_4] = {WICED_P07, audio_0_clk_0_TRIGGER_IN},
	[PLATFORM_GPIO_5] = {WICED_P08, adc_0_channel_0_TRIGGER_IN},
	[PLATFORM_GPIO_6] = {WICED_P12, spi_1_miso_0_TRIGGER_IN},
	[PLATFORM_GPIO_7] = {WICED_P13, spi_1_clk_0_TRIGGER_IN},
	[PLATFORM_GPIO_8] = {WICED_P14, WICED_GPIO},
	[PLATFORM_GPIO_9] = {WICED_P15, audio_0_ws_sync_0_TRIGGER_IN},
	[PLATFORM_GPIO_10] = {WICED_P16, audio_0_di_in_0_TRIGGER_IN},
	[PLATFORM_GPIO_11] = {WICED_P26, WICED_GPIO},
	[PLATFORM_GPIO_12] = {WICED_P27, WICED_GPIO},
	[PLATFORM_GPIO_13] = {WICED_P30, spi_1_mosi_0_TRIGGER_IN},
	[PLATFORM_GPIO_14] = {WICED_P32, uart_1_txd_0_TRIGGER_IN},
	[PLATFORM_GPIO_15] = {WICED_P34, spi_1_cs_0_TRIGGER_IN},
	[PLATFORM_GPIO_16] = {WICED_P35, amplifiers_0_tx_pu_0_TRIGGER_IN},
	[PLATFORM_GPIO_17] = {WICED_P36, amplifiers_0_rx_pu_0_TRIGGER_IN},
	[PLATFORM_GPIO_18] = {WICED_P37, adc_0_channel_1_TRIGGER_IN},
};
const size_t platform_gpio_pin_count = (sizeof(platform_gpio_pins) / sizeof(wiced_platform_gpio_t));
const wiced_platform_led_config_t platform_led[] = 
{
	[WICED_PLATFORM_LED_2] = LED2_config,
	[WICED_PLATFORM_LED_1] = LED1_config,
};
const size_t led_count = (sizeof(platform_led) / sizeof(wiced_platform_led_config_t));
const wiced_platform_button_config_t platform_button[] = 
{
	[WICED_PLATFORM_BUTTON_4] = BUTTON_USER_config,
	[WICED_PLATFORM_BUTTON_3] = BUTTON_VOL_DN_config,
	[WICED_PLATFORM_BUTTON_1] = BUTTON_CUSTOM_config,
	[WICED_PLATFORM_BUTTON_2] = BUTTON_VOL_UP_config,
};
const size_t button_count = (sizeof(platform_button) / sizeof(wiced_platform_button_config_t));
const wiced_platform_gpio_config_t platform_gpio[] = 
{
};
const size_t gpio_count = (sizeof(platform_gpio) / sizeof(wiced_platform_gpio_config_t));

