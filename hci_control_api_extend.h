/*********************************************************************
* @file
* hci_control_api_extend.h
*
* @brief
* The WICED HCI Garmin customized commands and events, except for list
* hci_control_api.h
*
* Copyright 2020 by Garmin Ltd. or its subsidiaries.
*********************************************************************/

#ifndef HCI_CONTROL_API_EXTEND_H
#define HCI_CONTROL_API_EXTEND_H

#ifdef __cplusplus
extern "C" {
#endif

/* Garmin add Miscellaneous commands */
#define HCI_CONTROL_MISC_COMMAND_SW_VERSION                 ( ( HCI_CONTROL_GROUP_MISC << 8 ) | 0x03 )    /* Request Garmin SW version */
#define HCI_CONTROL_MISC_COMMAND_READ_PAIR_INFO             ( ( HCI_CONTROL_GROUP_MISC << 8 ) | 0x04 )    /* Request BT send paired device name and address */
#define HCI_CONTROL_MISC_COMMAND_READ_PAIR_DEV_LIST         ( ( HCI_CONTROL_GROUP_MISC << 8 ) | 0x05 )    /* Request BT send paired device list */
#define HCI_CONTROL_MISC_COMMAND_READ_PAIR_DEV_NUM          ( ( HCI_CONTROL_GROUP_MISC << 8 ) | 0x06 )    /* Request BT send paired device number */
#define HCI_CONTROL_MISC_COMMAND_UNPAIR_ALL_DEV             ( ( HCI_CONTROL_GROUP_MISC << 8 ) | 0x07 )    /* Unpair all paired devices */
#define HCI_CONTROL_MISC_COMMAND_USER_CONFIRM_RESULT        ( ( HCI_CONTROL_GROUP_MISC << 8 ) | 0x08 )    /* Send the Host MCU User Confirm Result to BT module */
#define HCI_CONTROL_MISC_COMMAND_MISC_LE_ADV                ( ( HCI_CONTROL_GROUP_MISC << 8 ) | 0x09 )    /* Send BLE initial non-connectable advertising */

/* Garmin add Miscellaneous events */
#define HCI_CONTROL_MISC_EVENT_SW_VERSION                   ( ( HCI_CONTROL_GROUP_MISC << 8 ) | 0x03 )      /* Garmin SW version */
#define HCI_CONTROL_MISC_EVENT_READ_PAIR_INFO               ( ( HCI_CONTROL_GROUP_MISC << 8 ) | 0x04 )      /* Update paired device name and address */
#define HCI_CONTROL_MISC_EVENT_READ_PAIR_DEV_LIST           ( ( HCI_CONTROL_GROUP_MISC << 8 ) | 0x05 )      /* Update paired device list */
#define HCI_CONTROL_MISC_EVENT_READ_PAIR_DEV_NUM            ( ( HCI_CONTROL_GROUP_MISC << 8 ) | 0x06 )      /* Update paired device number */
#define HCI_CONTROL_MISC_EVENT_UNPAIR_ALL_DEV               ( ( HCI_CONTROL_GROUP_MISC << 8 ) | 0x07 )
#define HCI_CONTROL_MISC_EVENT_USER_CONFIRM_RESULT          ( ( HCI_CONTROL_GROUP_MISC << 8 ) | 0x08 )      /* Send the User Confirm Passkey and Device name to BT module */
#define HCI_CONTROL_MISC_EVENT_ADDR_PAIR_FAIL               ( ( HCI_CONTROL_GROUP_MISC << 8 ) | 0x09 )      /* SPP or iAP2 connect check the authentication error */
#define HCI_CONTROL_MISC_EVENT_PAIRING_CLT                  ( ( HCI_CONTROL_GROUP_MISC << 8 ) | 0x0A )      /* Add transport type for pairing complete event */
#define HCI_CONTROL_MISC_EVENT_SECURITY_FAIL                ( ( HCI_CONTROL_GROUP_MISC << 8 ) | 0x0B )      /* Send Authentication security fail event to MCU */

/* Extended Events for IAP2 Y-connect APP */
#define HCI_CONTROL_IAP2_EVENT_CONNECTED_2                  ( ( HCI_CONTROL_GROUP_IAP2 << 8 ) | 0x10 )      /* IAP2 connection opened */
#define HCI_CONTROL_IAP2_EVENT_SERVICE_NOT_FOUND_2          ( ( HCI_CONTROL_GROUP_IAP2 << 8 ) | 0x11 )      /* SDP record with IAP2 service not found */
#define HCI_CONTROL_IAP2_EVENT_CONNECTION_FAILED_2          ( ( HCI_CONTROL_GROUP_IAP2 << 8 ) | 0x12 )      /* Connection attempt failed  */
#define HCI_CONTROL_IAP2_EVENT_DISCONNECTED_2               ( ( HCI_CONTROL_GROUP_IAP2 << 8 ) | 0x13 )      /* IAP2 connection closed */
#define HCI_CONTROL_IAP2_EVENT_TX_COMPLETE_2                ( ( HCI_CONTROL_GROUP_IAP2 << 8 ) | 0x14 )      /* Data packet has been queued for transmission */
#define HCI_CONTROL_IAP2_EVENT_RX_DATA_2                    ( ( HCI_CONTROL_GROUP_IAP2 << 8 ) | 0x15 )      /* IAP2 data received */

/* Events for the Y-connect SPP profile use */
#define HCI_CONTROL_SPP_EVENT_CONNECTED_2                    ( ( HCI_CONTROL_GROUP_SPP << 8 ) | 0x08 )    /* SPP connection opened */
#define HCI_CONTROL_SPP_EVENT_SERVICE_NOT_FOUND_2            ( ( HCI_CONTROL_GROUP_SPP << 8 ) | 0x09 )    /* SDP record with SPP service not found */
#define HCI_CONTROL_SPP_EVENT_CONNECTION_FAILED_2            ( ( HCI_CONTROL_GROUP_SPP << 8 ) | 0x0A )    /* Connection attempt failed  */
#define HCI_CONTROL_SPP_EVENT_DISCONNECTED_2                 ( ( HCI_CONTROL_GROUP_SPP << 8 ) | 0x0B )    /* SPP connection closed */
#define HCI_CONTROL_SPP_EVENT_TX_COMPLETE_2                  ( ( HCI_CONTROL_GROUP_SPP << 8 ) | 0x0C )    /* Data packet has been queued for transmission */
#define HCI_CONTROL_SPP_EVENT_RX_DATA_2                      ( ( HCI_CONTROL_GROUP_SPP << 8 ) | 0x0D )    /* SPP data received */
#define HCI_CONTROL_SPP_EVENT_COMMAND_STATUS_2               ( ( HCI_CONTROL_GROUP_SPP << 8 ) | 0x0E )    /* Command status event for the requested operation */
#ifdef __cplusplus
}
#endif

#endif /* HCI_CONTROL_API_EXTEND_H */
