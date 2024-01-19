/*
 * Copyright 2018, Cypress Semiconductor Corporation or a subsidiary of Cypress Semiconductor
 *  Corporation. All rights reserved. This software, including source code, documentation and  related
 * materials ("Software"), is owned by Cypress Semiconductor  Corporation or one of its
 *  subsidiaries ("Cypress") and is protected by and subject to worldwide patent protection
 * (United States and foreign), United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license agreement accompanying the
 * software package from which you obtained this Software ("EULA"). If no EULA applies, Cypress
 * hereby grants you a personal, nonexclusive, non-transferable license to  copy, modify, and
 * compile the Software source code solely for use in connection with Cypress's  integrated circuit
 * products. Any reproduction, modification, translation, compilation,  or representation of this
 * Software except as specified above is prohibited without the express written permission of
 * Cypress. Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO  WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING,  BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE. Cypress reserves the right to make changes to
 * the Software without notice. Cypress does not assume any liability arising out of the application
 * or use of the Software or any product or circuit  described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or failure of the
 * Cypress product may reasonably be expected to result  in significant property damage, injury
 * or death ("High Risk Product"). By including Cypress's product in a High Risk Product, the
 *  manufacturer of such system or application assumes  all risk of such use and in doing so agrees
 * to indemnify Cypress against all liability.
 */

/** @file
 *
 * This file provides HCI to iAP2 pipe
 *
 */
#if 1	//dbg
#include "bt_types.h"
#include "wiced_bt_dev.h"
#endif
#include "wiced.h"
#include "wiced_gki.h"
#include "hci_control_api.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_iap2.h"
#include "wiced_bt_spp.h"
#include "wiced_timer.h"
#include "wiced_transport.h"
#include "string.h"
#include "wiced_bt_mfi.h"
#include "sdpdefs.h"
#include "wiced_app.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_dev.h"
#include "iap2_int.h"
#include "hci_control_api_extend.h"
#include "hci_control_api_ex.h"

#define IAP2_RFCOMM_SCN   2
/* Max TX packet to be sent over iAP2 */
#define TRANS_MAX_BUFFERS                   2//10 ->2
#define TRANS_UART_BUFFER_SIZE              1024

uint8_t LocalName[]                     = { 'i', 'a', 'p', '2', '_', 's', 'p', 'p', 0 };
uint8_t ModelIdentifier[]               = { '1', 'A', '1', '0', '0', '0', '1', '2', 0 };
uint8_t Manufacturer[]                  = { 'G', 'a', 'r', 'm', 'i', 'n', 0 };
uint8_t SerialNumber[]                  = { '0', '0', '0', '0', '0', '0', '0', '2', 0 };
uint8_t FirmwareVersion[]               = { '1', '.', '0', 0 };
uint8_t HardwareVersion[]               = { '8', '9', '8', '2', '0', 0 };
uint8_t ExternalAccessoryProtocolName[] = { 'c', 'o', 'm', '.', 'g', 'a', 'r', 'm', 'i', 'n', '.', 'n', 'a', 'v', 'i', 'l', 'i', 't', 'e', '.', 'd', 'a', 't', 'a', 0 };
uint8_t ExternalAccessoryProtocolName_2[] = { 'c', 'o', 'm', '.', 'y', 'c', 'o', 'n', 'n', 'e', 'c', 't', '.', 'd', 'a', 't', 'a', 0 };

uint8_t MatchAction                     = IAP2_MATCH_ACTION_DEVICE_MAY_PROMPT_AND_FIND_APP_BUTTON;
uint8_t AppMatchTeamId[]                = { '4', 'Q', 'X', 'E', '3', 'C', 'L', '7', 'F', 'A', '.', 'c', 'o', 'm', '.', 'c', 'y', 'p', 'r', 'e', 's', 's', '.', 's', 'e', 'r', 'i', 'a', 'l', 't', 'u', 'n', 'n', 'e', 'l', '.', 'd', 'a', 't', 'a', 0 };
uint8_t CurrentLanguage[]               = { 'e', 'n', 0 };
uint8_t SupportedLanguage[]             = { 'e', 'n', 0, 0 };
wiced_transport_buffer_pool_t*  iap2_host_trans_pool;

static uint32_t iap2_send_data_count = 0;            /* Used for transfer to HCI UART */
static uint32_t iap2_send_data_count_proto_2 = 0;    /* Used for transfer to HCI UART */

void hci_iap2_handle_command(uint16_t cmd_opcode, uint8_t* p_data, uint32_t data_len);
static void         iap2_connection_up_callback(uint16_t handle, uint8_t* bda);
static void         iap2_connection_down_callback(uint16_t handle);
static void         iap2_connection_failed_callback(void);
static void         iap2_service_not_found_callback(void);
static void         iap2_tx_complete_callback(uint16_t handle, wiced_result_t result);
static wiced_bool_t iap2_rx_data_callback(uint16_t handle, uint8_t* p_data, uint32_t data_len);
static void         iap2_transport_flow_control_timeout(uint32_t param);
static void         iap2_ota_flow_control_timeout(uint32_t param);

extern void wiced_bt_iap2_rx_flow_enable(uint16_t handle, wiced_bool_t enable);

wiced_bt_iap2_reg_t iap2_reg =
{
    IAP2_RFCOMM_SCN,                    /* RFCOMM service channel number for iAP2 connection */
    SPP_MAX_PACKET_SIZE,                /* RFCOMM MTU for iAP2 connection, iAP2 adds some header on top of SPP */
    iap2_connection_up_callback,        /* iAP2 connection established */
    iap2_connection_failed_callback,    /* iAP2 connection establishment failed */
    iap2_service_not_found_callback,    /* iAP2 service not found */
    iap2_connection_down_callback,      /* iAP2 connection disconnected */
    iap2_rx_data_callback,              /* Data packet received */
};

wiced_timer_t   iap2_transport_flow_control_timer;
wiced_timer_t   iap2_ota_flow_control_timer;

uint16_t     iap2_ea_handle = 0;            // session handle of the current EA connection
uint32_t     iap2_total_bytes_received = 0; // number of bytes received during the active EA session
BD_ADDR      iap2_connected_bda;            // connected device
wiced_bool_t iap2_ota_flow_controlled;      // TRUE if we were not able to send data OTA

static uint16_t iap2_connection_handle[IAP2_APP_HANDLE_NUMBER];

/*
 * iAP2 initialization
 */
void hci_iap2_init()
{
	iap2_host_trans_pool = wiced_transport_create_buffer_pool(TRANS_UART_BUFFER_SIZE, TRANS_MAX_BUFFERS);

    // Initialize IAP2 library
    wiced_bt_iap2_startup(&iap2_reg);
}

/*
 * Handle commands for iAP2 unit received over the UART from MCU
 */
void hci_iap2_handle_command(uint16_t cmd_opcode, uint8_t* p_data, uint32_t data_len)
{
    uint8_t    *p_trans_buf;
    uint8_t    buffer[128];
    uint16_t   out_len = sizeof(buffer);
    uint16_t   ea_handle;

    uint16_t handle_garmin = 0;
    BD_ADDR addr;
    BT_spp_app_type_e app_type = BT_SPP_APP_TYPE_INVALID;
//    wiced_bt_buffer_statistics_t buffer_stats[4];

    switch(cmd_opcode)
    {
    case HCI_CONTROL_IAP2_COMMAND_CONNECT:
        wiced_bt_iap2_connect(p_data);
        break;

    case HCI_CONTROL_IAP2_COMMAND_DISCONNECT:
        wiced_bt_iap2_disconnect((p_data[0] << 8) | p_data[1]);
        break;

    case HCI_CONTROL_IAP2_COMMAND_DATA:
        // WICED_BT_TRACE("COMMAND_DATA handle:%d length:%d\n\r", handle, data_len - 2);

        if (data_len > IAP2_MAX_PACKET_SIZE)
        {
            WICED_BT_TRACE("too many bytes\n\r");
            return;
        }

//	    wiced_bt_get_buffer_usage (buffer_stats, sizeof(buffer_stats));

//	    WICED_BT_TRACE("0:%d/%d 1:%d/%d 2:%d/%d 3:%d/%d\n\r", buffer_stats[0].current_allocated_count, buffer_stats[0].max_allocated_count,
//	                   buffer_stats[1].current_allocated_count, buffer_stats[1].max_allocated_count,
//	                   buffer_stats[2].current_allocated_count, buffer_stats[2].max_allocated_count,
//	                   buffer_stats[3].current_allocated_count, buffer_stats[3].max_allocated_count);

        // first 2 bytes should be set by the application with the session id
        ea_handle = p_data[0] + (p_data[1] << 8);
        if (!wiced_bt_iap2_send_session_data(ea_handle, p_data + 2, data_len - 2))
            iap2_tx_complete_callback(ea_handle, 1);

        // check if there is place for more data, if true tell the MCU that the buffer is done
        // otherwise start flow control timer, to wait until buffer becomes available
        if (wiced_bt_iap2_can_send_more_data())
            iap2_tx_complete_callback(ea_handle, 0);
        else
        {
            iap2_ota_flow_controlled = WICED_TRUE;
            wiced_start_timer(&iap2_ota_flow_control_timer, 50);
        }
        break;

    case HCI_CONTROL_IAP2_COMMAND_GET_AUTH_CHIP_INFO:
        WICED_BT_TRACE("Received get auth chip info\n\r");
        if (wiced_bt_iap2_get_auth_chip_info(buffer, sizeof (buffer)) == WICED_BT_SUCCESS)
            wiced_transport_send_data(HCI_CONTROL_IAP2_EVENT_AUTH_CHIP_INFO, buffer, AUTH_CHIP_INFO_DATA_SIZE);
        break;

    case HCI_CONTROL_IAP2_COMMAND_GET_AUTH_CHIP_CERTIFICATE:

    	WICED_BT_TRACE("Received get auth chip certificate\n\r");
        if ((p_trans_buf = (uint8_t*) wiced_transport_allocate_buffer(iap2_host_trans_pool)) != NULL)
        {
            if (wiced_auth_chip_copy_device_certificate(p_trans_buf, &out_len) == WICED_BT_SUCCESS)
            {
                WICED_BT_TRACE("read success len:%d\n\r", out_len);
                wiced_transport_send_buffer(HCI_CONTROL_IAP2_EVENT_AUTH_CHIP_CERTIFICATE, p_trans_buf, out_len);
            }
            else
            {
                WICED_BT_TRACE("read failed%d\n\r");
                wiced_transport_free_buffer(p_trans_buf);
            }
        }
        break;

    case HCI_CONTROL_IAP2_COMMAND_GET_AUTH_CHIP_SIGNATURE:
        WICED_BT_TRACE("Received auth chip sign\n\r");
        if (wiced_auth_chip_platform_create_signature(p_data, data_len, buffer, &out_len) == WICED_BT_SUCCESS)
            wiced_transport_send_data(HCI_CONTROL_IAP2_EVENT_AUTH_CHIP_SIGNATURE, buffer, out_len);
        break;

/*Add by Garmin define command */
    case HCI_CONTROL_IAP2_COMMAND_CONNECT_GARMIN:
        /* p[0]~p[5] :bt address*/
        /* link note : why should we need reverse address*/
        memcpy(&(addr[0]), p_data, BD_ADDR_LEN);
        /* p[6] : BT_spp_app_type_e*/
        app_type = *(p_data + 6);
        WICED_BT_TRACE("Garmin IAP2 connect command, addr(rev):%B\r\n", &addr[0]);
        wiced_bt_iap2_connect(p_data);
        break;

    case HCI_CONTROL_IAP2_COMMAND_DISCONNECT_GARMIN:
        /* p[0]~p[1] :Connection handle*/
        memcpy(&(handle_garmin), p_data, 2);
        WICED_BT_TRACE("Garmin SPP disconnect command, handle:%d\r\n", handle_garmin);
        wiced_bt_iap2_disconnect(handle_garmin);
        break;

    case HCI_CONTROL_IAP2_COMMAND_DATA_GARMIN:
        /* p[0]~p[1] :Connection handle*/
        memcpy(&(handle_garmin), p_data, 2);
        /* p[2] : BT_spp_app_type_e*/
        app_type = *(p_data + 2);
        WICED_BT_TRACE("Garmin SPP send data command, handle:%d app type:%d,data len =%d\r\n", handle_garmin, app_type, data_len-3);

        if (data_len - 3 > IAP2_MAX_PACKET_SIZE)
        {
            WICED_BT_TRACE("too many bytes\n\r");
            return;
        }

//	    wiced_bt_get_buffer_usage (buffer_stats, sizeof(buffer_stats));

//	    WICED_BT_TRACE("0:%d/%d 1:%d/%d 2:%d/%d 3:%d/%d\n\r", buffer_stats[0].current_allocated_count, buffer_stats[0].max_allocated_count,
//	                   buffer_stats[1].current_allocated_count, buffer_stats[1].max_allocated_count,
//	                   buffer_stats[2].current_allocated_count, buffer_stats[2].max_allocated_count,
//	                   buffer_stats[3].current_allocated_count, buffer_stats[3].max_allocated_count);

        // first 2 bytes should be set by the application with the session id
        if (!wiced_bt_iap2_send_session_data(handle_garmin, p_data + 3 , data_len - 3))
            iap2_tx_complete_callback(handle_garmin, 1);

        // check if there is place for more data, if true tell the MCU that the buffer is done
        // otherwise start flow control timer, to wait until buffer becomes available
        if (wiced_bt_iap2_can_send_more_data())
            iap2_tx_complete_callback(handle_garmin, 0);
        else
        {
            iap2_ota_flow_controlled = WICED_TRUE;
            wiced_start_timer(&iap2_ota_flow_control_timer, 50);
        }

        break;

    default:
        WICED_BT_TRACE("hci_iap2_handle_command:unknown opcode %x\n\r", cmd_opcode);
        break;
    }
}

/*
 * Send iAP2 connection up event
 */
void iap2_connection_up_callback(uint16_t handle, uint8_t* bda)
{
    uint8_t   tx_buf[9];
    uint8_t  *p = tx_buf;
    wiced_result_t result;
    int       i;

    WICED_BT_TRACE("%s handle:%d address:%B", __FUNCTION__, handle, bda);

    wiced_init_timer(&iap2_transport_flow_control_timer, iap2_transport_flow_control_timeout, handle, WICED_MILLI_SECONDS_TIMER);
    wiced_init_timer(&iap2_ota_flow_control_timer, iap2_ota_flow_control_timeout, handle, WICED_MILLI_SECONDS_TIMER);

    for (i = 0; i < BD_ADDR_LEN; i++)
        *p++ = bda[BD_ADDR_LEN - 1 - i];;

    iap2_total_bytes_received  = 0;
    iap2_ota_flow_controlled   = WICED_FALSE;
    memcpy (iap2_connected_bda, bda, BD_ADDR_LEN);

    *p++ = (handle & 0xff);
    *p++ = ((handle >> 8) & 0xff);

    get_iap2_connection_handle( (uint16_t *)iap2_connection_handle );
    //Byte[0]: (6-bytes) Bluetooth device address
    memcpy(&tx_buf[0], bda, BD_ADDR_LEN);
    //Byte[6]: (2-bytes) Connection handle
    memcpy(&tx_buf[6], handle, sizeof(uint16_t));
    //Byte[8]: (1-byte) SPP application type specified in BT_spp_app_type_e
    if( handle == iap2_connection_handle[NAVI_APP_HANDLE_BYTE] )
    {
        WICED_BT_TRACE("%s proto_ID:1, NAVI APP, handle:%d address:%B ", __FUNCTION__, handle, bda);
        tx_buf[8] = BT_SPP_APP_NAVILITE;
    }
    else if( handle == iap2_connection_handle[Y_APP_HANDLE_BYTE] )
    {
        WICED_BT_TRACE("%s proto_ID:2 Yconnect APP, handle:%d address:%B ", __FUNCTION__, handle, bda);
        tx_buf[8] = BT_SPP_APP_MOTOCON;
    }
    else
    {
        WICED_BT_TRACE( "ERROR:iap2 connection handle unknown:%d %d %d\r\n", handle, iap2_connection_handle[NAVI_APP_HANDLE_BYTE], iap2_connection_handle[Y_APP_HANDLE_BYTE] );
        tx_buf[8] = BT_SPP_APP_TYPE_INVALID;
    }
    wiced_transport_send_data(HCI_CONTROL_IAP2_EVENT_CONNECTED_GARMIN, &tx_buf[0], 9);
}

/*
 * Connection failed indication from the library
 */
void iap2_connection_failed_callback(void)
{
    WICED_BT_TRACE("%s\n\r", __FUNCTION__);
    /*no handle reuturn, can not recognize whitch app*/
    wiced_transport_send_data(HCI_CONTROL_IAP2_EVENT_CONNECTION_FAILED_GARMIN, BT_SPP_APP_NAVILITE, 1);
    wiced_transport_send_data(HCI_CONTROL_IAP2_EVENT_CONNECTION_FAILED_GARMIN, BT_SPP_APP_MOTOCON, 1);
}

/*
 * Service not found failed indication from the library
 */
void iap2_service_not_found_callback(void)
{
    WICED_BT_TRACE("%s\n\r", __FUNCTION__);
    wiced_transport_send_data(HCI_CONTROL_IAP2_EVENT_SERVICE_NOT_FOUND, NULL, 0);
}

/*
 * Send a data packet to the server
 */
void iap2_tx_complete_callback(uint16_t handle, wiced_result_t result)
{
    uint8_t  tx_buf[6];
    uint8_t *p = tx_buf;
    wiced_result_t  trans_rc = WICED_SUCCESS;

    *p++ = (handle & 0xff);
    *p++ = ((handle >> 8) & 0xff);
    *p++ = result;

    if( handle == iap2_connection_handle[NAVI_APP_HANDLE_BYTE] )
    {
        trans_rc = wiced_transport_send_data(HCI_CONTROL_IAP2_EVENT_TX_COMPLETE, tx_buf, (int)(p - tx_buf));
        WICED_BT_TRACE("hci_iap2_send_tx_complete - proto ID:1 handle:%d result:%d \n\r", handle, result);
    }
    else if( handle == iap2_connection_handle[Y_APP_HANDLE_BYTE] )
    {
        trans_rc = wiced_transport_send_data(HCI_CONTROL_IAP2_EVENT_TX_COMPLETE_2, tx_buf, (int)(p - tx_buf));
        WICED_BT_TRACE("hci_iap2_send_tx_complete - proto ID:2 handle:%d result:%d \n\r", handle, result);
    }
    else
    {
        WICED_BT_TRACE( "ERROR:iap2 connection handle fail:%d %d %d\r\n", handle, iap2_connection_handle[NAVI_APP_HANDLE_BYTE], iap2_connection_handle[Y_APP_HANDLE_BYTE] );
    }

#if 1
        WICED_BT_TRACE( "iap2> trans_rc: %d\n\r", trans_rc );
#endif
}

/*
 * iAP2 connection down callback
 */
void iap2_connection_down_callback(uint16_t handle)
{
    uint8_t   tx_buf[9];
    uint8_t  *p = tx_buf;

    WICED_BT_TRACE("%s handle:%d\n\r", __FUNCTION__, handle);

    wiced_stop_timer(&iap2_transport_flow_control_timer);
    wiced_deinit_timer(&iap2_transport_flow_control_timer);

    wiced_stop_timer(&iap2_ota_flow_control_timer);
    wiced_deinit_timer(&iap2_ota_flow_control_timer);

    iap2_total_bytes_received    = 0;

    *p++ = (handle & 0xff);
    *p++ = ((handle >> 8) & 0xff);

    get_iap2_connection_handle( (uint16_t *)iap2_connection_handle );

    //Byte[0]: (2-bytes) Connection handle
    memcpy(&tx_buf[0], &handle, sizeof(uint16_t));
    //Byte[2]: (1-byte) SPP application type specified in BT_spp_app_type_e
    if( handle == iap2_connection_handle[NAVI_APP_HANDLE_BYTE] )
    {
        tx_buf[2] = BT_SPP_APP_NAVILITE;
        WICED_BT_TRACE("%s proto_ID:1, NAVI_APP, handle:%d\n\r", __FUNCTION__, handle);
        iap2_connection_handle[NAVI_APP_HANDLE_BYTE] = 0;
        set_iap2_connection_handle( (uint16_t *)iap2_connection_handle );
    }
    else if( handle == iap2_connection_handle[Y_APP_HANDLE_BYTE] )
    {
        tx_buf[2] = BT_SPP_APP_MOTOCON;
        WICED_BT_TRACE("%s proto_ID:2, Yconnect, handle:%d\n\r", __FUNCTION__, handle);
        iap2_connection_handle[Y_APP_HANDLE_BYTE] = 0;
        set_iap2_connection_handle( (uint16_t *)iap2_connection_handle );
    }
    else
    {
        tx_buf[2] = BT_SPP_APP_TYPE_INVALID;
        WICED_BT_TRACE( "ERROR:iap2 connection handle fail:%d %d %d\n\r", handle, iap2_connection_handle[NAVI_APP_HANDLE_BYTE], iap2_connection_handle[Y_APP_HANDLE_BYTE] );
    }
    wiced_transport_send_data(HCI_CONTROL_IAP2_EVENT_DISCONNECTED_GARMIN, &tx_buf[0], 9);
}

/*
 * Process data received over EA session.  Return TRUE if we were able to allocate buffer to
 * deliver to the host.
 */
wiced_bool_t iap2_rx_data_callback(uint16_t session_handle, uint8_t* data, uint32_t data_len)
{
    uint8_t* tx_buf = NULL;
    uint8_t send_data_count = 0;
    uint8_t rest_data_len = 0;
    uint8_t tx_buf_len = MIN(data_len + 3, MAX_TRANSPORT_DATA_SIZE);

    WICED_BT_TRACE( "%s :session_handle =%d, navi app handle =%d, Yconnect handle=%d, length = %d\r\n",
                    __func__, session_handle, iap2_connection_handle[NAVI_APP_HANDLE_BYTE], iap2_connection_handle[Y_APP_HANDLE_BYTE], data_len);
    //Byte[0]: (2-bytes) Connection handle
    memcpy(tx_buf, &session_handle, sizeof(uint16_t));
    //Byte[2]: (1-byte) SPP application type specified in BT_spp_app_type_e
    if( session_handle == iap2_connection_handle[NAVI_APP_HANDLE_BYTE] )
    {

        *(tx_buf+2) = BT_SPP_APP_NAVILITE;
    }
    else if( session_handle == iap2_connection_handle[Y_APP_HANDLE_BYTE] )
    {
        *(tx_buf+2) = BT_SPP_APP_MOTOCON;
    }
    else
    {
        WICED_BT_TRACE( "RX ERROR: invaild iap2 handle:%d %d %d\r\n", session_handle, iap2_connection_handle[NAVI_APP_HANDLE_BYTE], iap2_connection_handle[Y_APP_HANDLE_BYTE] );
        *(tx_buf+2) = BT_SPP_APP_TYPE_CNT;
    }
    //Byte[3]: (n-bytes) Data from remote device, where n <= SPP_DATA_MAX_SIZE
    while( (data_len - send_data_count) > (MAX_TRANSPORT_DATA_SIZE -3) ) /*3 byte for handle and length*/
    {
        WICED_BT_TRACE("IAP2 rx data is too big, separate sending\r\n");
        //Byte[3]: (n-bytes) Data from remote device, where n <= SPP_DATA_MAX_SIZE
        memcpy(tx_buf+3, data+send_data_count, MAX_TRANSPORT_DATA_SIZE -3);
        wiced_transport_send_data( HCI_CONTROL_SPP_EVENT_RX_DATA_GARMIN, data, MAX_TRANSPORT_DATA_SIZE);
        send_data_count += (MAX_TRANSPORT_DATA_SIZE -3);
        memset(tx_buf+3, 0, MIN(data_len + 3, MAX_TRANSPORT_DATA_SIZE));
    }
    WICED_BT_TRACE("IAP2 rx data is under uart transmit limit, sending all data\r\n");
    //Byte[3]: (n-bytes) Data from remote device, where n <= SPP_DATA_MAX_SIZE
    memcpy(tx_buf+3, data+send_data_count, data_len - send_data_count);
    wiced_transport_send_data( HCI_CONTROL_SPP_EVENT_RX_DATA_GARMIN, data, (data_len - send_data_count)+3);
    wiced_bt_free_buffer(tx_buf);

    return WICED_TRUE;
}

/*
 * The timeout function is periodically called if we are not able to send data over UART
 */
void iap2_transport_flow_control_timeout(uint32_t param)
{
    int num_buffers_left;

    //Disable the flow control if 3/4 buffers available
    if ((num_buffers_left = wiced_transport_get_buffer_count(iap2_host_trans_pool)) > ((TRANS_MAX_BUFFERS * 3) / 4))
    {
        // time to tell library that we can accept data again
        wiced_bt_iap2_rx_flow_enable ((uint16_t)param, WICED_TRUE);
        WICED_BT_TRACE("IAP2 flow enabled buffers left:%d\n\r", num_buffers_left);
    }
    else
    {
        WICED_BT_TRACE("IAP2 flow still disabled left:%d\n\r", num_buffers_left);
        wiced_start_timer(&iap2_transport_flow_control_timer, 50);
    }
}

/*
* The timeout function is periodically called if we are not able to send data OTA
*/
void iap2_ota_flow_control_timeout(uint32_t param)
{
    if (iap2_ota_flow_controlled)
    {
        // check if there is place for more data, if true tell the MCU that the buffer is done
        // otherwise start flow control timer, to wait until buffer becomes available
        if (wiced_bt_iap2_can_send_more_data())
        {
            iap2_ota_flow_controlled = WICED_FALSE;
            iap2_tx_complete_callback((uint16_t)param, 0);
        }
        else
        {
            wiced_start_timer(&iap2_ota_flow_control_timer, 50);
        }
    }
}
