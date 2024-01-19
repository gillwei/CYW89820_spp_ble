/*
 * Copyright 2020, Cypress Semiconductor Corporation or a subsidiary of
 * Cypress Semiconductor Corporation. All Rights Reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software"), is owned by Cypress Semiconductor Corporation
 * or one of its subsidiaries ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products. Any reproduction, modification, translation,
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


#ifndef _WICED_APP_H_
#define _WICED_APP_H_

#include "wiced_bt_trace.h"
#include "garmin_nvm_storage.h"

#define WICED_PIN_CODE_LEN                  4
extern  uint8_t pincode[WICED_PIN_CODE_LEN];

/* BR/EDR Profiles/Applications */
//#define WICED_APP_AUDIO_SRC_INCLUDED        WICED_TRUE
//#define WICED_APP_AUDIO_RC_TG_INCLUDED      WICED_TRUE
//#define WICED_APP_AUDIO_RC_CT_INCLUDED      WICED_TRUE

/* BLE Profiles/Applications */
#define WICED_APP_LE_INCLUDED               WICED_TRUE
#define WICED_APP_LE_SLAVE_CLIENT_INCLUDED  WICED_TRUE
//#define WICED_APP_ANCS_INCLUDED             WICED_TRUE
#define WICED_APP_TEST_INCLUDED             WICED_TRUE

//#define ADV_FOREVER
//#define HCI_TRACE_OVER_TRANSPORT            0   // If defined HCI traces are send over transport/WICED HCI interface //link modify, move to spp.c
//#define SAVE_KEY_ON_BUFFER

#define SUPPORT_SERVER_MODE                 1

/* Max transport size is limited by 264 - 12 internal packet = 252 byte
   Set size to 0xF0 for easy debugging */
#define MAX_TRANSPORT_DATA_SIZE             240
#define MAX_DEVICE_NAME_LEN                 32
#define MAX_REMOTE_NAME_RETRY_TIME          3
#define HCI_UART_APP_BAUDRATE               3000000
#define NAVI_UUID_U16                       0x7220
#define YCONNECT_UUID_U16                   0x8911

#define GARMIN_SW_MAJOR_VER                 1
#define GARMIN_SW_MINOR_VER                 8

#define CONFIG_TX_PWR                       1

#if     CONFIG_TX_PWR
    #define CONFIGURED_BTC_TX_POWER         4
    #define CONFIGURED_BLE_TX_POWER         0
#else
    #define POKE_POWER                      11
#endif

#define IAP2_TRACE_SEND                     1
#define BLE_READ_FROM_89820                 0
#define WICED_NVRAM_VSID_APP                ( WICED_NVRAM_VSID_START + 2 )


#define PROTOCOL_ID_NAVI_APP                0x01
#define PROTOCOL_ID_Y_APP                   0x02
#define IAP2_APP_HANDLE_NUMBER              2
#define NAVI_APP_HANDLE_BYTE                0
#define Y_APP_HANDLE_BYTE                   1

#define SPP_RFCOMM_SCN                      1
#define IAP2_RFCOMM_SCN                     2
#define SPP_RFCOMM_SCN_2                    3

typedef enum BT_spp_app_type
{
    BT_SPP_APP_NAVILITE = 0,
    BT_SPP_APP_MOTOCON,

    BT_SPP_APP_TYPE_CNT,
    BT_SPP_APP_TYPE_INVALID = BT_SPP_APP_TYPE_CNT
} BT_spp_app_type_e;
extern BT_paired_device_list_t*               paired_list;
#endif /* _WICED_APP_H_ */
