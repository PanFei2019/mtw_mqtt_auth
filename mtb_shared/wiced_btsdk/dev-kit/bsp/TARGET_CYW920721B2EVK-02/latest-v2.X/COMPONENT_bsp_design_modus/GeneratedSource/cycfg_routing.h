/*******************************************************************************
* File Name: cycfg_routing.h
*
* Description:
* Establishes all necessary connections between hardware elements.
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

#if !defined(CYCFG_ROUTING_H)
#define CYCFG_ROUTING_H

#if defined(__cplusplus)
extern "C" {
#endif

#include "cycfg_notices.h"
static inline void init_cycfg_routing(void) {}
#define init_cycfg_connectivity() init_cycfg_routing()
#define ioss_0_pin_6_AUX UNKNOWN
#define ioss_0_pin_7_AUX UNKNOWN
#define ioss_0_pin_8_AUX UNKNOWN
#define ioss_0_pin_12_AUX UNKNOWN
#define ioss_0_pin_13_AUX UNKNOWN
#define ioss_0_pin_15_AUX UNKNOWN
#define ioss_0_pin_16_AUX UNKNOWN
#define ioss_0_pin_30_AUX UNKNOWN
#define ioss_0_pin_32_AUX UNKNOWN
#define ioss_0_pin_34_AUX UNKNOWN
#define ioss_0_pin_35_AUX UNKNOWN
#define ioss_0_pin_36_AUX UNKNOWN
#define ioss_0_pin_37_AUX UNKNOWN

#define adc_0_channel_0_TRIGGER_IN WICED_GPIO
#define adc_0_channel_1_TRIGGER_IN WICED_GPIO
#define ADC_THERMISTOR_aux_0_TRIGGER_OUT ADC_INPUT_P8
#define amplifiers_0_rx_pu_0_TRIGGER_IN WICED_RX_PU
#define amplifiers_0_tx_pu_0_TRIGGER_IN WICED_TX_PU
#define audio_0_clk_0_TRIGGER_IN WICED_PCM_CLK_I2S_CLK
#define audio_0_di_in_0_TRIGGER_IN WICED_PCM_IN_I2S_DI
#define audio_0_do_out_0_TRIGGER_IN WICED_PCM_OUT_I2S_DO
#define audio_0_ws_sync_0_TRIGGER_IN WICED_PCM_SYNC_I2S_WS
#define BATTERY_MON_aux_0_TRIGGER_OUT ADC_INPUT_P37
#define spi_1_clk_0_TRIGGER_IN WICED_SPI_2_CLK
#define spi_1_cs_0_TRIGGER_IN WICED_SPI_2_CS
#define spi_1_miso_0_TRIGGER_IN WICED_SPI_2_MISO
#define spi_1_mosi_0_TRIGGER_IN WICED_SPI_2_MOSI
#define uart_1_txd_0_TRIGGER_IN WICED_UART_2_TXD

#if defined(__cplusplus)
}
#endif


#endif /* CYCFG_ROUTING_H */
