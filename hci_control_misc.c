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
 * This file implements the Miscellaneous Commands controlled over UART. Please refer to
 * the WICED HCI Control Protocol Software User Manual (WICED-SWUM10x-R) for additional
 * details on the HCI UART control protocol
 */

#include "wiced_bt_cfg.h"
#include "hci_control_api.h"
#include "hci_control.h"
//#include "hci_control_audio.h"
#include "wiced_transport.h"

#define BCM920706 20706
#define BCM920707 20707

#include "wiced_bt_trace.h"
#include "wiced_app.h"
#include "wiced_hal_nvram.h"
#include "hci_control_misc.h"
//#include "spp.h"
#include "hci_control_api_extend.h"
#include "hci_control_api_ex.h"
#include "hci_control_device.h"

/******************************************************************************
 *                          Constants
 ******************************************************************************/
#define CCUID_LENGTH  14
/******************************************************************************
 *                          Function Declarations
 ******************************************************************************/
void hci_control_misc_handle_get_version(void);
static void hci_control_misc_sw_version(void);
static void hci_handle_all_unbond(void);
void hci_control_le_set_scan_response_data(uint8_t* device_name,uint32_t device_name_length);
extern void                          hci_control_le_set_con_advertisement_data(void);
extern void                          hci_control_le_set_non_con_advertisement_data(uint8_t* device_name,uint32_t device_name_length);
/******************************************************************************
 *                          Variable Definitions
 ******************************************************************************/
/******************************************************************************
 *                          Function Definitions
 ******************************************************************************/
void hci_control_misc_handle_command( uint16_t cmd_opcode, uint8_t* p_data, uint32_t data_len )
{
	//uint8_t pair_dev_num[1] = { 0 };
    wiced_result_t result;
    switch( cmd_opcode )
    {
    case HCI_CONTROL_MISC_COMMAND_PING:
        wiced_transport_send_data( HCI_CONTROL_MISC_EVENT_PING_REPLY, p_data, data_len );
        break;

    case HCI_CONTROL_MISC_COMMAND_GET_VERSION:
        hci_control_misc_handle_get_version();
        break;

    case HCI_CONTROL_MISC_COMMAND_SW_VERSION:
        hci_control_misc_sw_version();
        break;

    case HCI_CONTROL_MISC_COMMAND_READ_PAIR_INFO:
        if( 1 == data_len )
            {
            hci_control_misc_read_pair_info( p_data[0] );
            }
        break;

    /* Send pair dev number to MCU */
    case HCI_CONTROL_MISC_COMMAND_READ_PAIR_DEV_LIST:
        if( 1 == data_len )
        {
            hci_pair_dev_list_return( p_data[0]);
        }
        break;

    /* Clear all pair devices for factory reset function */
    case HCI_CONTROL_MISC_COMMAND_UNPAIR_ALL_DEV:
        hci_handle_all_unbond();
        break;

    /* Receive the user confirm request result and feed to BT stack */
    case HCI_CONTROL_MISC_COMMAND_USER_CONFIRM_RESULT:
        spp_user_confirm_result( p_data[0] );
        break;

    case HCI_CONTROL_MISC_COMMAND_MISC_LE_ADV:
        hci_control_le_handle_misc_advertise_cmd( p_data, data_len );
        break;

    default:
        WICED_BT_TRACE( "Unknown miscellaneous command, opcode:%04x\n\r", cmd_opcode );
        break;
    }
}

void hci_control_misc_handle_get_version( void )
{
    uint8_t   tx_buf[20];
    uint8_t   cmd = 0;

// If this is 20819 or 20820, we do detect the device from hardware
#define RADIO_ID    0x006007c0
#define RADIO_20820 0x80
#define CHIP_20820  20820
#define CHIP_20819  20819
#if (CHIP==CHIP_20819) || (CHIP==CHIP_20820)
    uint32_t chip = CHIP_20819;
    if (*(UINT32*) RADIO_ID & RADIO_20820)
    {
        chip = CHIP_20820;
    }
#else
    uint32_t  chip = CHIP;
#endif

    tx_buf[cmd++] = WICED_SDK_MAJOR_VER;
    tx_buf[cmd++] = WICED_SDK_MINOR_VER;
    tx_buf[cmd++] = WICED_SDK_REV_NUMBER;
    tx_buf[cmd++] = WICED_SDK_BUILD_NUMBER & 0xFF;
    tx_buf[cmd++] = (WICED_SDK_BUILD_NUMBER>>8) & 0xFF;
    tx_buf[cmd++] = chip & 0xFF;
    tx_buf[cmd++] = (chip>>8) & 0xFF;
    tx_buf[cmd++] = (chip>>24) & 0xFF;
    tx_buf[cmd++] = 0; // not used

    /* Send MCU app the supported features */
    tx_buf[cmd++] = HCI_CONTROL_GROUP_GATT;
    tx_buf[cmd++] = HCI_CONTROL_GROUP_AUDIO;
    tx_buf[cmd++] = HCI_CONTROL_GROUP_ANCS;
    tx_buf[cmd++] = HCI_CONTROL_GROUP_AVRC_CONTROLLER;
    tx_buf[cmd++] = HCI_CONTROL_GROUP_AMS;
    tx_buf[cmd++] = HCI_CONTROL_GROUP_AVRC_TARGET;
    tx_buf[cmd++] = HCI_CONTROL_GROUP_SPP;
    tx_buf[cmd++] = HCI_CONTROL_GROUP_IAP2;
#ifdef WICED_APP_HFP_AG_INCLUDED
    tx_buf[cmd++] = HCI_CONTROL_GROUP_AG;
#endif
#ifdef WICED_APP_HFP_HF_INCLUDED
    tx_buf[cmd++] = HCI_CONTROL_GROUP_HF;
#endif

    wiced_transport_send_data( HCI_CONTROL_MISC_EVENT_VERSION, tx_buf, cmd );

    //hci_control_audio_support_features_send();
}

static void hci_control_misc_sw_version(void)
{
    uint8_t   tx_buf[2];
    uint8_t   cmd = 0;

    tx_buf[cmd++] = GARMIN_SW_MAJOR_VER;
    tx_buf[cmd++] = GARMIN_SW_MINOR_VER;

    wiced_transport_send_data( HCI_CONTROL_MISC_EVENT_SW_VERSION, tx_buf, cmd );
}

/*link note: add api by our self, should we keep this function? or can be replace by origin function?*/
void hci_control_misc_read_pair_info(uint8_t pair_dev_index)
{
    /* Go through the saved NVRAM chunks */
    wiced_result_t              result;
    int                         read_bytes;
    wiced_bt_device_link_keys_t link_keys;
    wiced_bt_remote_name_t      name;
    uint8_t                     data[1 + BD_ADDR_LEN + MAX_DEVICE_NAME_LEN];
    uint16_t                    nvram_id_linkkey = WICED_NVRAM_VSID_APP + ( pair_dev_index * 2 );
    uint16_t                    nvram_id_devname = nvram_id_linkkey + 1;

    read_bytes = wiced_hal_read_nvram( nvram_id_linkkey, sizeof( wiced_bt_device_link_keys_t ), (uint8_t*)&link_keys, &result );

    // Save pair_dev_index
    data[0] = pair_dev_index;

    if( 0 == result )
    {
        WICED_BT_TRACE( "link_keys read bytes:%d\n\r", read_bytes );

        memcpy( &(data[1]), &(link_keys.bd_addr[0]), BD_ADDR_LEN );

        wiced_hal_read_nvram( nvram_id_devname, MAX_DEVICE_NAME_LEN, (uint8_t*)&name, &result );
        WICED_BT_TRACE( "read name result:%d name: %s\n\r", result, name );
        memcpy( &data[1 + BD_ADDR_LEN], &name, MAX_DEVICE_NAME_LEN );

        wiced_transport_send_data( HCI_CONTROL_MISC_EVENT_READ_PAIR_INFO, &(data[0]),  1 + BD_ADDR_LEN + MAX_DEVICE_NAME_LEN );
    }
}



/*
 * Input the pair dev index and return the pair device number and pair device information,
 * used on the case when MCU need to update the pair device list
 */
/*link note: add api by our self, should we keep this function? or can be replace by origin function?*/
void hci_pair_dev_list_return(uint8_t pair_dev_index)
{
    int                         read_bytes;
    wiced_bt_device_link_keys_t link_keys;
    wiced_bt_remote_name_t      name;
    uint8_t                     data[2 + BD_ADDR_LEN + MAX_DEVICE_NAME_LEN]; // Pair dev number + pair dev index + bd addr + device name
    uint16_t                    nvram_id_linkkey = WICED_NVRAM_VSID_APP + ( pair_dev_index * 2 );
    uint16_t                    nvram_id_devname = nvram_id_linkkey + 1;
    uint8_t                     pair_dev_num;

    WICED_BT_TRACE( "%s \n\r", __FUNCTION__);
    if(pair_dev_index >= MAX_PAIRED_DEVICE_NUM)
    {
        WICED_BT_TRACE( "illegal pair index:%d\n\r",pair_dev_index );
    }
    pair_dev_num = paired_list->paired_device_num;
    WICED_BT_TRACE( "%s: pair num:%d, pair index:%d\n\r", __FUNCTION__, pair_dev_num, pair_dev_index );

    data[0] = pair_dev_num;
    data[1] = pair_dev_index;

    memcpy( &(data[2]), &(paired_list->device[pair_dev_index].link_key.bd_addr[0]), BD_ADDR_LEN);
    memcpy( &data[2 + BD_ADDR_LEN], &(paired_list->device[pair_dev_index].device_name), MAX_DEVICE_NAME_LEN );
    wiced_transport_send_data( HCI_CONTROL_MISC_EVENT_READ_PAIR_DEV_LIST, &(data[0]),  2 + BD_ADDR_LEN + MAX_DEVICE_NAME_LEN );
}

/*
 * Read NVRAM ID to calculate the pair device number, use the result
 * and readbytes to judge the VSID is empty or not
 */
/*link note: add api by our self, should we keep this function? or can be replace by origin function?*/
uint8_t hci_read_pair_dev_num(void)
{
    wiced_result_t result;
    int        nvram_id = 0;
    uint8_t    pair_dev_num[1] = { 0 };

    nvram_id = hci_hal_alloc_nvram_id();
    WICED_BT_TRACE( "%s nvram_id:%d\n\r", __FUNCTION__, nvram_id );
    if( nvram_id != 0 )
    {
        return ( nvram_id - WICED_NVRAM_VSID_APP ) / 2;
    }
    else
    {
        return 0;
    }
}

/* Clear all pair devices for factory reset function */
/*link note: add api by our self, should we keep this function? or can be replace by origin function?*/
void hci_handle_all_unbond(void)
{
    wiced_result_t              result_key;
    wiced_result_t              result_name;
    uint8_t                     paired_dev_num;
    int                         nvram_id_linkkey = WICED_NVRAM_VSID_APP;
    int                         nvram_id_name = nvram_id_linkkey + 1;

    paired_dev_num = hci_read_pair_dev_num();
    WICED_BT_TRACE( "%s: paired num:%d\n\r", __FUNCTION__, paired_dev_num);

    // Unpair all paired devices
    for( uint8_t i = 0; i < paired_dev_num; i++ )
    {
        wiced_hal_delete_nvram( nvram_id_linkkey + 2 * i, &result_key );
        wiced_hal_delete_nvram( nvram_id_name + 2 * i, &result_name );
        WICED_BT_TRACE( "Delete result_key:%d result:name:%d\n\r", result_key, result_name );
    }
}

/*
 *  Handle initial advertising command
 */
void hci_control_le_handle_misc_advertise_cmd
    (
    uint8_t* p_data,
    uint32_t data_length
    )
{
    wiced_result_t          result = WICED_SUCCESS;
    ble_advertising_type_t  ble_advertising_type;

    ble_advertising_type = p_data[0];

    switch( ble_advertising_type )
    {
    case BLE_ADV_OFF:
        result = wiced_bt_start_advertisements( BTM_BLE_ADVERT_OFF, BLE_ADDR_RANDOM, NULL );
        break;

    case BLE_ADV_NON_CONNECTABLE:
        hci_control_le_set_non_con_advertisement_data( &p_data[1], data_length - 1 );
        result = wiced_bt_start_advertisements( BTM_BLE_ADVERT_NONCONN_HIGH, BLE_ADDR_RANDOM, NULL );
        break;

    case BLE_ADV_CONNECTABLE:
        hci_control_le_set_con_advertisement_data();
        hci_control_le_set_scan_response_data( &p_data[1], data_length - 1 );
        result = wiced_bt_start_advertisements( BTM_BLE_ADVERT_UNDIRECTED_HIGH, BLE_ADDR_RANDOM, NULL );
        break;

    default:
        WICED_BT_TRACE( "%s Parameter ERROR %d\r\n", __FUNCTION__, ble_advertising_type );
        break;
    }

    WICED_BT_TRACE( "%s ble_advertising_type:%d result:%d\n\r", __FUNCTION__, ble_advertising_type, result );
    hci_control_send_command_status_evt( HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_SUCCESS );
}

/*
 * When 89820 initialized, set BLE scan response data
 */
void hci_control_le_set_scan_response_data
    (
    uint8_t* device_name,
    uint32_t device_name_length
    )
{
    wiced_bt_ble_advert_elem_t adv_elem[2];
    uint8_t num_elem = 0;
    uint8_t flag = BTM_BLE_GENERAL_DISCOVERABLE_FLAG;
    wiced_result_t result;

    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_FLAG;
    adv_elem[num_elem].len          = sizeof(uint8_t);
    adv_elem[num_elem].p_data       = &flag;
    num_elem++;

    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_NAME_COMPLETE;
    adv_elem[num_elem].len          = device_name_length;
    adv_elem[num_elem].p_data       = (uint8_t *)device_name;
    num_elem++;

    result = wiced_bt_ble_set_raw_scan_response_data(num_elem, adv_elem);
    WICED_BT_TRACE( "%s result:%d\n\r", __FUNCTION__, result );
}
