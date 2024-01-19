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
 * This file implement BTLE application controlled over UART.
 * The GATT database is defined in this file and is not changed by the MCU.
 */

#pragma once

#include <wiced_bt_cfg.h>
#include <wiced_bt_gatt.h>

/******************************************************
 *                     Constants
 ******************************************************/
#define LE_CONTROL_MAX_CONNECTIONS          20
#define LE_CONTROL_CONNECT_TIMEOUT          10

#define CYPRESS_COMPANY_ID 0x0131
#define LE_PROCESS_DATA_MAX_LEN             256
#define LE_PROCESS_DATA_HEADER_LEN          8
/******************************************************
 *                     Structures
 ******************************************************/
typedef struct
{
#define LE_CONTROL_STATE_IDLE                       0
#define LE_CONTROL_STATE_DISCOVER_PRIMARY_SERVICES  1
#define LE_CONTROL_STATE_DISCOVER_CHARACTERISTICS   2
#define LE_CONTROL_STATE_DISCOVER_DESCRIPTORS       3
#define LE_CONTROL_STATE_READ_VALUE                 4
#define LE_CONTROL_STATE_WRITE_VALUE                5
#define LE_CONTROL_STATE_WRITE_NO_RESPONSE_VALUE    6
#define LE_CONTROL_STATE_NOTIFY_VALUE               7
#define LE_CONTROL_STATE_INDICATE_VALUE             8
#define LE_CONTROL_STATE_WRITE_DESCRIPTOR_VALUE     9
#define LE_CONTROL_STATE_DISCONNECTING              10

    uint8_t           state;                // Application discovery state
    wiced_bool_t      indication_sent;      // WICED_TRUE if indication sent and not acked
    BD_ADDR           bd_addr;
    uint16_t          conn_id;              // Connection ID used for exchange with the stack
    uint16_t          peer_mtu;             // MTU received in the MTU request (or 23 if peer did not send MTU request)

    uint8_t           role;                 // HCI_ROLE_MASTER or HCI_ROLE_SLAVE
} hci_control_le_conn_state_t;

typedef struct
{
    hci_control_le_conn_state_t conn[LE_CONTROL_MAX_CONNECTIONS + 1];
} hci_control_le_cb_t;

typedef struct t_hci_control_le_pending_tx_buffer_t
{
    wiced_bool_t        tx_buf_saved;
    uint16_t            tx_buf_conn_id;
    uint16_t            tx_buf_type;
    uint16_t            tx_buf_len;
    uint16_t            tx_buf_handle;
    uint8_t             tx_buf_data[HCI_CONTROL_GATT_COMMAND_MAX_TX_BUFFER];
} hci_control_le_pending_tx_buffer_t;

/******************************************************
 *               Function Definitions
 ******************************************************/
void hci_control_le_init(void);
void hci_control_le_enable(const wiced_bt_cfg_settings_t *settings);
void hci_control_le_connect_timeout(uint32_t count);
wiced_bt_gatt_status_t hci_control_le_gatt_callback(wiced_bt_gatt_evt_t event,
        wiced_bt_gatt_event_data_t *p_data);
