/*********************************************************************
* @file  hci_control_api_ex.h
* @brief Extended HCI control api defined by Garmin.
*
* Copyright 2021 by Garmin Ltd. or its subsidiaries.
*********************************************************************/

#ifndef _HCI_CONTROL_API_EX_H_
#define _HCI_CONTROL_API_EX_H_

#ifdef __cplusplus
extern "C" {
#endif

typedef enum BLE_ADVERTISING_TYPE
{
    BLE_ADV_OFF,
    BLE_ADV_NON_CONNECTABLE,
    BLE_ADV_CONNECTABLE
}ble_advertising_type_t;

/*--------------------------------------------------------------------
                        GENERAL INCLUDES
--------------------------------------------------------------------*/
/*====================================================================
@note: Cypress defines the values used for Wiced HCI protocol in file
       hci_control_api.h. Based on that Garmin adds required values
       with following types in this extended file to use:
         - NEW: A new value defined by Garmin. For easier to identify
                add the suffix - GARMIN.
         - REDECLARED: A value is already defined by Cypress but is
                       not declared in hci_control_api.h
       Make sure that the values added here are not conflicted with
       hci_control_api.h.
====================================================================*/
#include "hci_control_api.h"

/*--------------------------------------------------------------------
                        HCI ERROR CODES
--------------------------------------------------------------------*/
/*====================================================================
@type: REDECLARED
====================================================================*/
#define HCI_SUCCESS                   ( 0x00 )
#define HCI_ERR_PEER_USER             ( 0x13 )
#define HCI_ERR_CONN_CAUSE_LOCAL_HOST ( 0x16 )

/*--------------------------------------------------------------------
                        PACKET TYPES
--------------------------------------------------------------------*/
/*====================================================================
@type: REDECLARED
====================================================================*/
#define HCI_STANDARD_COMMAND_PKT ( 0x01 )
#define HCI_STANDARD_EVENT_PKT   ( 0x04 )

/*--------------------------------------------------------------------
                        GROUP CODES
--------------------------------------------------------------------*/
/*====================================================================
@type: NEW
====================================================================*/
#define HCI_CONTROL_GROUP_STANDARD_GARMIN ( 0xFE )

/*--------------------------------------------------------------------
                        GENERAL COMMANDS
--------------------------------------------------------------------*/

/*--------------------------------------------------------------------
                        STANDARD HCI COMMANDS
--------------------------------------------------------------------*/
/*====================================================================
@type: REDECLARED
====================================================================*/
/*link note: would these command set be sent from MCU to 898920? should Garmin spp handle these?*/
#define HCI_STANDARD_COMMAND_RESET           ( 0x0C03 )
#define HCI_STANDARD_COMMAND_DUT_MODE        ( 0x1803 )
#define HCI_STANDARD_COMMAND_TX_CARRIER      ( 0xFC14 )
#define HCI_STANDARD_COMMAND_UPDATE_BAUDRATE ( 0xFC18 )
#define HCI_STANDARD_COMMAND_WRITE_RAM       ( 0xFC4C )
#define HCI_STANDARD_COMMAND_READ_RAM        ( 0xFC4D )
#define HCI_STANDARD_COMMAND_LAUNCH_RAM      ( 0xFC4E )
#define HCI_STANDARD_COMMAND_CHIP_ERASE      ( 0xFFCE )

/*--------------------------------------------------------------------
                        SPP COMMANDS
--------------------------------------------------------------------*/
/*====================================================================
@type: NEW
@data: Byte[0]: (6-bytes) Bluetooth device address
       Byte[6]: (1-byte) SPP application type specified in BT_spp_app_type_e
====================================================================*/
#define HCI_CONTROL_SPP_COMMAND_CONNECT_GARMIN ( ( HCI_CONTROL_GROUP_SPP << 8 ) | 0xFF )

/*====================================================================
@type: NEW
@data: Byte[0]: (2-bytes) Connection handle
       Byte[2]: (1-byte) SPP application type specified in BT_spp_app_type_e
====================================================================*/
#define HCI_CONTROL_SPP_COMMAND_DISCONNECT_GARMIN ( ( HCI_CONTROL_GROUP_SPP << 8 ) | 0xFE )

/*====================================================================
@type: NEW
@data: Byte[0]: (2-bytes) Connection handle
       Byte[2]: (1-byte) SPP application type specified in BT_spp_app_type_e
       Byte[3]: (n-bytes) Data to remote device, where n <= SPP_DATA_MAX_SIZE
====================================================================*/
#define HCI_CONTROL_SPP_COMMAND_DATA_GARMIN ( ( HCI_CONTROL_GROUP_SPP << 8 ) | 0xFD )

/*--------------------------------------------------------------------
                        IAP2 COMMANDS
--------------------------------------------------------------------*/
/*====================================================================
@type: NEW
@data: Byte[0]: (6-bytes) Bluetooth device address
====================================================================*/
#define HCI_CONTROL_IAP2_COMMAND_CONNECT_GARMIN ( ( HCI_CONTROL_GROUP_IAP2 << 8 ) | 0xFF )

/*====================================================================
@type: NEW
@data: Byte[0]: (2-bytes) Connection handle
====================================================================*/
#define HCI_CONTROL_IAP2_COMMAND_DISCONNECT_GARMIN ( ( HCI_CONTROL_GROUP_IAP2 << 8 ) | 0xFE )

/*====================================================================
@type: NEW
@data: Byte[0]: (2-bytes) Connection handle
       Byte[2]: (1-byte) SPP application type specified in BT_spp_app_type_e
       Byte[3]: (n-bytes) Data to remote device, where n <= SPP_DATA_MAX_SIZE
====================================================================*/
#define HCI_CONTROL_IAP2_COMMAND_DATA_GARMIN ( ( HCI_CONTROL_GROUP_IAP2 << 8 ) | 0xFD )

/*--------------------------------------------------------------------
                        LE COMMANDS
--------------------------------------------------------------------*/
/*====================================================================
@type: NEW
@data: Byte[0]: (1-byte) Advertising mode specified in BLE_advertising_mode_e
       Byte[1]: (1-byte) Data type specified in BLE_advertising_data_type_e
       Byte[2]: (1-byte) Data length
       Byte[3]: (n-bytes) Advertising data, where n <= BLE_ADVERTISING_DATA_MAX_SIZE
====================================================================*/
#define HCI_CONTROL_LE_COMMAND_ADVERTISE_GARMIN ( ( HCI_CONTROL_GROUP_LE << 8 ) | 0xFF )

/*--------------------------------------------------------------------
                        GENERAL EVENTS
--------------------------------------------------------------------*/
/*====================================================================
@type: NEW
@data: Byte[0]: (6-bytes) Bluetooth device address
====================================================================*/
#define HCI_CONTROL_EVENT_AUTH_LOST_GARMIN ( ( HCI_CONTROL_GROUP_DEVICE << 8 ) | 0xFF )

/*====================================================================
@type: NEW
@data: Byte[0]: (6-bytes) Bluetooth device address
       Byte[6]: (1-byte) Transport type
       Byte[7]: (1-byte) Connected or not
       Byte[8]: (1-byte) Reason
====================================================================*/
#define HCI_CONTROL_EVENT_CONNECTION_STATUS_GARMIN ( ( HCI_CONTROL_GROUP_DEVICE << 8 ) | 0xFE )

/*====================================================================
@type: NEW
@data: Byte[0]: (1-byte) Software major version
       Byte[1]: (1-byte) Software minor version
====================================================================*/
#define HCI_CONTROL_EVENT_DEVICE_STARTED_GARMIN ( ( HCI_CONTROL_GROUP_DEVICE << 8 ) | 0xFD )

/*====================================================================
@type: NEW
@data: Byte[0]: (6-bytes) Bluetooth device address
====================================================================*/
#define HCI_CONTROL_EVENT_PAIRED_DEVICE_DELETED_GARMIN ( ( HCI_CONTROL_GROUP_DEVICE << 8 ) | 0xFC )

/*====================================================================
@type: NEW
@data: Byte[0]: (1-byte) Number of total paired devices
       Byte[1]: (1-byte) Number of current device
       Byte[2]: (6-bytes) Bluetooth device address
       Byte[8]: (32-bytes) Bluetooth device name
       Byte[40]: (1-byte) Device type specified in BT_device_type_e
       Byte[41]: (1-byte) Whether or not the device has lost its authentication
       Byte[42]: (1-byte) Whether or not the device supports iAP connection
====================================================================*/
#define HCI_CONTROL_EVENT_PAIRED_DEVICE_LIST_GARMIN ( ( HCI_CONTROL_GROUP_DEVICE << 8 ) | 0xFB )

/*====================================================================
@type: NEW
@data: Byte[0]: (1-byte) Result
       Byte[1]: (1-byte) Transport type specified in BT_transport_type_e
       Byte[2]: (6-bytes) Bluetooth device address
====================================================================*/
#define HCI_CONTROL_EVENT_PAIRING_COMPLETE_GARMIN ( ( HCI_CONTROL_GROUP_DEVICE << 8 ) | 0xFA )

/*====================================================================
@type: NEW
@data: Byte[0]: (6-bytes) Bluetooth device address
       Byte[6]: (4-bytes) Numeric comparison code
       Byte[10]: (n-bytes) Bluetooth device name, where n <= BT_DEVICE_NAME_LEN
====================================================================*/
#define HCI_CONTROL_EVENT_USER_CONFIRMATION_GARMIN ( ( HCI_CONTROL_GROUP_DEVICE << 8 ) | 0xF9 )

/*--------------------------------------------------------------------
                        STANDARD HCI EVENTS
--------------------------------------------------------------------*/
/*====================================================================
@type: REDECLARED
====================================================================*/
#define HCI_STANDARD_EVENT_COMMAND_COMPLETE ( 0x0E )

/*--------------------------------------------------------------------
                        SPP EVENTS
--------------------------------------------------------------------*/
/*====================================================================
@type: NEW
@data: Byte[0]: (6-bytes) Bluetooth device address
       Byte[6]: (2-bytes) Connection handle
       Byte[8]: (1-byte) SPP application type specified in BT_spp_app_type_e
====================================================================*/
#define HCI_CONTROL_SPP_EVENT_CONNECTED_GARMIN ( ( HCI_CONTROL_GROUP_SPP << 8 ) | 0xFF )

/*====================================================================
@type: NEW
@data: Byte[0]: (1-byte) SPP application type specified in BT_spp_app_type_e
====================================================================*/
#define HCI_CONTROL_SPP_EVENT_CONNECTION_FAILED_GARMIN ( ( HCI_CONTROL_GROUP_SPP << 8 ) | 0xFE )

/*====================================================================
@type: NEW
@data: Byte[0]: (2-bytes) Connection handle
       Byte[2]: (1-byte) SPP application type specified in BT_spp_app_type_e
====================================================================*/
#define HCI_CONTROL_SPP_EVENT_DISCONNECTED_GARMIN ( ( HCI_CONTROL_GROUP_SPP << 8 ) | 0xFD )

/*====================================================================
@type: NEW
@data: Byte[0]: (2-bytes) Connection handle
       Byte[2]: (1-byte) SPP application type specified in BT_spp_app_type_e
       Byte[3]: (n-bytes) Data from remote device, where n <= SPP_DATA_MAX_SIZE
====================================================================*/
#define HCI_CONTROL_SPP_EVENT_RX_DATA_GARMIN ( ( HCI_CONTROL_GROUP_SPP << 8 ) | 0xFC )

/*--------------------------------------------------------------------
                        IAP2 EVENTS
--------------------------------------------------------------------*/
/*====================================================================
@type: NEW
@data: Byte[0]: (6-bytes) Bluetooth device address
       Byte[6]: (2-bytes) Connection handle
       Byte[8]: (1-byte) SPP application type specified in BT_spp_app_type_e
====================================================================*/
#define HCI_CONTROL_IAP2_EVENT_CONNECTED_GARMIN ( ( HCI_CONTROL_GROUP_IAP2 << 8 ) | 0xFF )

/*====================================================================
@type: NEW
@data: Byte[0]: (1-byte) SPP application type specified in BT_spp_app_type_e
====================================================================*/
#define HCI_CONTROL_IAP2_EVENT_CONNECTION_FAILED_GARMIN ( ( HCI_CONTROL_GROUP_IAP2 << 8 ) | 0xFE )

/*====================================================================
@type: NEW
@data: Byte[0]: (2-bytes) Connection handle
       Byte[2]: (1-byte) SPP application type specified in BT_spp_app_type_e
====================================================================*/
#define HCI_CONTROL_IAP2_EVENT_DISCONNECTED_GARMIN ( ( HCI_CONTROL_GROUP_IAP2 << 8 ) | 0xFD )

/*====================================================================
@type: NEW
@data: Byte[0]: (2-bytes) Connection handle
       Byte[2]: (1-byte) SPP application type specified in BT_spp_app_type_e
       Byte[3]: (n-bytes) Data from remote device, where n <= SPP_DATA_MAX_SIZE
====================================================================*/
#define HCI_CONTROL_IAP2_EVENT_RX_DATA_GARMIN ( ( HCI_CONTROL_GROUP_IAP2 << 8 ) | 0xFC )

#ifdef __cplusplus
}
#endif

#endif // _HCI_CONTROL_API_EX_H_
