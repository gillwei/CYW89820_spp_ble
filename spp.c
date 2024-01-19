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
 * SPP Application for 20XXX devices.
 *
 * SPP application uses SPP profile library to establish, terminate, send and receive SPP
 * data over BR/EDR. This sample supports single a single SPP connection.
 *
 * Following compilation flags are important for testing
 *  HCI_TRACE_OVER_TRANSPORT - configures HCI traces to be routed to the WICED HCI interface
 *  WICED_BT_TRACE_ENABLE    - enables WICED_BT_TRACEs.  You can also modify makefile.mk to build
 *                             with _debug version of the library
 *  SEND_DATA_ON_INTERRUPT   - if defined, the app will send 1Meg of data on application button push
 *  SEND_DATA_ON_TIMEOUT     - if defined, the app will send 4 bytes every second while session is up
 *  LOOPBACK_DATA            - if enabled, the app sends back received data
 *
 * To demonstrate the app, work through the following steps.
 * 1. Build and download the application to the WICED board.
 * 2. Open the BT/BLE Profile Client Control application and open the port for WICED HCI for the device.
 *    Default baud rate configured in the application is defined by the BSP HCI_UART_DEAULT_BAUD #define,
 *    usually either 3M or 115200 depending on board UART capabilities.
 * 3. Run the BTSpy program to view protocol and application traces.
 *    See "BT/BLE Profile Client Control" and "BT Spy" in chip-specifc readme.txt for more information about these apps.
 * 4. On Windows 10 PCs, right click on the Bluetooth icon in the system tray and
 *    select 'Add a Bluetooth Device'. Find and pair with the spp app. That should create an incoming and an outgoing
 *    COM port on your computer. Right click on the Bluetooth icon in the system tray and
 *    select 'Open Settings', scroll down and select "More Bluetooth options" and then
 *    select the 'COM Ports' tab.
 * 5. Use application such as Term Term to open the outgoing COM port. Opening the port
 *    will create the SPP connection.
 * 6. Type any key on the terminal of the outgoing COM port, the spp application will receive the key.
 * 7. By default, (SEND_DATA_ON_INTERRUPT=1) the application sends 1 MB data to the peer application on every
 *    App button press on the WICED board.
 * 8. If desired, edit the spp.c file to configure the application to send data on a timer to the peer application by
 *    setting SEND_DATA_ON_INTERRUPT=0 and SEND_DATA_ON_TIMEOUT=1*
 *
 * Features demonstrated
 *  - Use of SPP library
 *
 *  Note: This snippet app does not support WICED HCI Control and may use transport only for tracing.
 *  If you route traces to WICED HCI UART, use ClientControl app with baud rate equal to that
 *  set in the wiced_transport_cfg_t structure below (currently set at HCI_UART_DEFAULT_BAUD, either 3 Mbps or 115200
 *  depending on the capabilities of the board used).
 */

#include "sparcommon.h"
#include "wiced.h"
#include "wiced_gki.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_sdp.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_uuid.h"
#include "wiced_hal_nvram.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_cfg.h"
#include "wiced_bt_spp.h"
#include "wiced_hci.h"
#include "wiced_timer.h"
#include "wiced_transport.h"
#include "wiced_platform.h"
#include "wiced_memory.h"
#include "string.h"
#include "wiced_bt_stack.h"
#include "wiced_bt_rfcomm.h"
#include "wiced_bt_iap2.h"
#include "hci_control_api.h"
#include "hci_control_device.h"
#include "hci_control_test.h"
#include "wiced_app.h"
#include "hci_control_misc.h"
#include "hci_control_api_extend.h"
#include "hci_control_api_ex.h"
#if defined(CYW20706A2) || defined(CYW43012C0)
#include "wiced_bt_app_hal_common.h"
#endif

#if defined(WICED_APP_LE_INCLUDED) && defined(WICED_APP_LE_SLAVE_CLIENT_INCLUDED)
#include "hci_control.h"
#include "hci_control_le.h"
#include "le_slave.h"
#endif

#define HCI_TRACE_OVER_TRANSPORT            0   // If defined HCI traces are send over transport/WICED HCI interface
// configure either SEND_DATA_ON_INTERRUPT or SEND_DATA_ON_TIMEOUT, but not both
// CYW9M2BASE-43012BT does not support SEND_DATA_ON_INTERRUPT because the platform does not have button connected to BT board.
#if !defined (NO_BUTTON_SUPPORT)
#define SEND_DATA_ON_INTERRUPT              1   // If defined application button causes 1Meg of data to be sent
#else
#define SEND_DATA_ON_INTERRUPT              0
#endif
#if defined(SEND_DATA_ON_INTERRUPT) && (SEND_DATA_ON_INTERRUPT==0)
#define SEND_DATA_ON_TIMEOUT                1   // If defined application sends 4 bytes of data every second
#endif
//#define LOOPBACK_DATA                     1   // If defined application loops back received data

#define WICED_EIR_BUF_MAX_SIZE              264
#define SPP_NVRAM_ID                        WICED_NVRAM_VSID_START

/* Max TX packet to be sent over SPP */
#define MAX_TX_BUFFER                       1017
#define TRANS_MAX_BUFFERS                   2
#define TRANS_UART_BUFFER_SIZE              1024
#define SPP_MAX_PAYLOAD                     1007

#if SEND_DATA_ON_INTERRUPT
#include "wiced_hal_gpio.h"
#include "wiced_platform.h"

#define APP_TOTAL_DATA_TO_SEND             1000000
#define BUTTON_GPIO                         WICED_P30

#if defined(CYW43012C0)
#define WICED_PLATFORM_BUTTON_1          WICED_P00
#ifndef WICED_GPIO_BUTTON
#define WICED_GPIO_BUTTON                WICED_PLATFORM_BUTTON_1
#endif
#ifndef WICED_GPIO_BUTTON_DEFAULT_STATE
#define WICED_GPIO_BUTTON_DEFAULT_STATE  GPIO_PIN_OUTPUT_HIGH
#endif
#endif

int     app_send_offset = 0;
uint8_t app_send_buffer[SPP_MAX_PAYLOAD];
uint32_t time_start = 0;
#endif

/*link modify :TBD, should we keep these?*/
#define USER_CONFIRM_RESULT_DATA_LENGTH     (BD_ADDR_LEN + sizeof( uint32_t ) + MAX_DEVICE_NAME_LEN )
#define HCI_CONTROL_EVENT_PAIRING_COMPLETE_PAIR_RESULT_DATA_LENGTH  1
#define HCI_CONTROL_MISC_EVENT_PAIRING_CLT_DATA_LENGTH              ( 1 + HCI_CONTROL_EVENT_PAIRING_COMPLETE_PAIR_RESULT_DATA_LENGTH + BD_ADDR_LEN )
//#define CONNECTION_STATUS_DATA_LENGTH       ( 3 + sizeof( uint16_t ) + BD_ADDR_LEN ) // include transport type, reason, connected, handle, BD address
#define CONNECTION_STATUS_DATA_LENGTH       ( 3 +  BD_ADDR_LEN ) // include  BD address,transport type, connected, reason
#define MISC_SECURITY_FAIL_EVT_DATA_LENGTH           ( 2 + BD_ADDR_LEN ) // operation result(1) + status(1) + BD address(6)

#ifdef SEND_DATA_ON_TIMEOUT
wiced_timer_t spp_app_timer;
void app_timeout(uint32_t count);
#endif

#define SDP_ATTR_CLASS_ID_128(uuid)                                      \
    SDP_ATTR_ID(ATTR_ID_SERVICE_CLASS_ID_LIST), SDP_ATTR_SEQUENCE_1(17), \
        ((UUID_DESC_TYPE << 3) | SIZE_SIXTEEN_BYTES), uuid

#define SDP_ATTR_UUID128( uuid )          ( ( UUID_DESC_TYPE << 3 ) | SIZE_SIXTEEN_BYTES ), uuid

static char                            paired_device_name[MAX_DEVICE_NAME_LEN] = {0};
char                                   default_device_name[MAX_DEVICE_NAME_LEN] = {'U','N','K','N','O','W','N','\0'};
static uint8_t                         paired_device_bd_addr[BD_ADDR_LEN];
static wiced_bt_remote_name_cback_t    p_remote_name_result_cb;
static wiced_bt_device_link_keys_t     saved_link_key;
static uint8_t                         user_confirm_result_data[USER_CONFIRM_RESULT_DATA_LENGTH] = { 0 };
static uint8_t                         get_device_name_flag = FALSE;
static uint8_t                         ble_pairing_start = FALSE;
static uint8_t                         spp_connected = FALSE;
static uint8_t                         spp_connected_2 = FALSE;
uint8_t                                remote_name_retry = 0; //MAX_DEVICE_NAME_RETRY_TIME

BT_paired_device_info_t                pairing_device;
BT_paired_device_list_t*               paired_list = NULL;
uint8_t                                sw_version[2]={GARMIN_SW_MAJOR_VER, GARMIN_SW_MINOR_VER};

wiced_bt_connection_status_change_cback_t  wiced_bt_connection_status_change_cb;

/** BTC user confirm result */
typedef enum BTC_USER_CONFIRM_RESULT
{
    USER_CONFIRM_FAIL,
    USER_CONFIRM_SUCCESS
} btc_user_confirm_result_t;


/*****************************************************************************
**  Structures
*****************************************************************************/
#if 0  //link note:use our own define?
#if defined (CYW20706A2)
#define SPP_RFCOMM_SCN               2
#else
#define SPP_RFCOMM_SCN               1
#endif
#endif

#define IAP2_RFCOMM_SCN              2

#define MAX_TX_RETRY                 30
#define TX_RETRY_TIMEOUT             100 // msec

static void         spp_connection_up_callback(uint16_t handle, uint8_t* bda);
static void         spp_connection_down_callback(uint16_t handle);
static wiced_bool_t spp_rx_data_callback(uint16_t handle, uint8_t* p_data, uint32_t data_len);
static void spp_service_not_found_callback(void);
static void spp_connection_failed_callback(void);

static void hci_iap2_spp_write_eir(void);
extern void hci_send_command_status_evt(uint16_t code,uint8_t status); //why extern this
extern void hci_iap2_handle_command(uint16_t cmd_opcode,uint8_t* p_data,uint32_t data_len);

static void spp_connection_up_callback_2(uint16_t handle,uint8_t* bda);
static void spp_connection_down_callback_2(uint16_t handle);
static void spp_service_not_found_callback_2(void);
static void spp_connection_failed_callback_2(void);
static wiced_bool_t spp_rx_data_callback_2(uint16_t handle,uint8_t* p_data,uint32_t data_len);

extern void hci_control_handle_read_buffer_stats( void );

wiced_bt_spp_reg_t spp_reg =
{
    SPP_RFCOMM_SCN,                     /* RFCOMM service channel number for SPP connection */
    MAX_TX_BUFFER,                      /* RFCOMM MTU for SPP connection */
    spp_connection_up_callback,         /* SPP connection established */
    spp_connection_failed_callback,     /* SPP connection establishment failed, not used because this app never initiates connection */
    spp_service_not_found_callback,     /* SPP service not found, not used because this app never initiates connection */
    spp_connection_down_callback,       /* SPP connection disconnected */
    spp_rx_data_callback,               /* Data packet received */
};


wiced_bt_spp_reg_t spp_reg_2 =
    {
    SPP_RFCOMM_SCN_2,                     /* RFCOMM service channel number for SPP connection */
    MAX_TX_BUFFER,                        /* RFCOMM MTU for SPP connection */
    spp_connection_up_callback_2,         /* SPP connection established */
    spp_connection_failed_callback_2,     /* SPP connection establishment failed, not used because this app never initiates connection */
    spp_service_not_found_callback_2,     /* SPP service not found, not used because this app never initiates connection */
    spp_connection_down_callback_2,       /* SPP connection disconnected */
    spp_rx_data_callback_2,               /* Data packet received */
};

wiced_transport_buffer_pool_t*  host_trans_pool;
uint16_t                        spp_handle = 0;
uint16_t                        spp_handle_2 = 0;
wiced_timer_t                   app_tx_timer;
uint32_t                        spp_rx_bytes = 0;
uint32_t                        spp_rx_bytes_2 = 0;
uint32_t                        spp_tx_retry_count = 0;

const uint8_t app_sdp_db[] = // Define SDP database
{
    SDP_ATTR_SEQUENCE_2(308),
    // SPP Service
    SDP_ATTR_SEQUENCE_1(69),                                                    // 2 bytes
        SDP_ATTR_RECORD_HANDLE(0x10001),                                    // 8 bytes
        SDP_ATTR_CLASS_ID(NAVI_UUID_U16),                                   // 8
        SDP_ATTR_RFCOMM_PROTOCOL_DESC_LIST(SPP_RFCOMM_SCN),                 // 17 bytes
        SDP_ATTR_BROWSE_LIST,                                               // 8
        SDP_ATTR_PROFILE_DESC_LIST(NAVI_UUID_U16, 0x0102),                  // 13 byte
        SDP_ATTR_SERVICE_NAME(10),                                          // 15
        'S', 'P', 'P', 'S', 'E', 'R', 'V', 'E', 'R', '1',

    // SPP Service
    SDP_ATTR_SEQUENCE_1(69),                                                // 2 bytes
        SDP_ATTR_RECORD_HANDLE(0x10001),                                    // 8 bytes
        SDP_ATTR_CLASS_ID(YCONNECT_UUID_U16),                               // 8
        SDP_ATTR_RFCOMM_PROTOCOL_DESC_LIST(SPP_RFCOMM_SCN_2),               // 17 bytes
        SDP_ATTR_BROWSE_LIST,                                               // 8
        SDP_ATTR_PROFILE_DESC_LIST(YCONNECT_UUID_U16, 0x0102),              // 13 byte
        SDP_ATTR_SERVICE_NAME(10),                                          // 15
        'S', 'P', 'P', 'S', 'E', 'R', 'V', 'E', 'R', '2',

    // iAP2 Service
    SDP_ATTR_SEQUENCE_1(93),
        SDP_ATTR_RECORD_HANDLE(0x10002),                                    //8
        SDP_ATTR_CLASS_ID_128(IAP2_ACCESSORY_UUID),                         //22
        SDP_ATTR_ID(ATTR_ID_PROTOCOL_DESC_LIST),
        SDP_ATTR_SEQUENCE_1(31),                                            // 36
            SDP_ATTR_SEQUENCE_1(3),
                SDP_ATTR_UUID16(UUID_PROTOCOL_L2CAP),
            SDP_ATTR_SEQUENCE_1(5),
                SDP_ATTR_UUID16(UUID_PROTOCOL_RFCOMM),
                SDP_ATTR_VALUE_UINT1(IAP2_RFCOMM_SCN),
            SDP_ATTR_SEQUENCE_1(17),
                SDP_ATTR_UUID128(IAP2_ACCESSORY_UUID),
            SDP_ATTR_ID(ATTR_ID_BT_PROFILE_DESC_LIST), SDP_ATTR_SEQUENCE_1(22), // 27
            SDP_ATTR_SEQUENCE_1(20),
                SDP_ATTR_UUID128(IAP2_ACCESSORY_UUID),
                SDP_ATTR_VALUE_UINT2(0x0100),

     // Device ID service
     SDP_ATTR_SEQUENCE_1(69),                                                // 2 bytes, length of the record
         SDP_ATTR_RECORD_HANDLE(0x10003),                                    // 8 byte
         SDP_ATTR_CLASS_ID(UUID_SERVCLASS_PNP_INFORMATION),                  // 8
         SDP_ATTR_PROTOCOL_DESC_LIST(1),                                     // 18
         SDP_ATTR_UINT2(ATTR_ID_SPECIFICATION_ID, 0x103),                    // 6
         SDP_ATTR_UINT2(ATTR_ID_VENDOR_ID, 0x0f),                            // 6
         SDP_ATTR_UINT2(ATTR_ID_PRODUCT_ID, 0x0401),                         // 6
         SDP_ATTR_UINT2(ATTR_ID_PRODUCT_VERSION, 0x0001),                    // 6
         SDP_ATTR_BOOLEAN(ATTR_ID_PRIMARY_RECORD, 0x01),                     // 5
         SDP_ATTR_UINT2(ATTR_ID_VENDOR_ID_SOURCE, DI_VENDOR_ID_SOURCE_BTSIG) // 6
};

// Length of the SDP database
const uint16_t app_sdp_db_len = sizeof(app_sdp_db);

uint8_t pincode[WICED_PIN_CODE_LEN] = { 0x30, 0x30, 0x30, 0x30 };

extern const wiced_bt_cfg_settings_t wiced_bt_cfg_settings;
extern const wiced_bt_cfg_buf_pool_t wiced_bt_cfg_buf_pools[WICED_BT_CFG_NUM_BUF_POOLS];

#if 0   //defined WICED_BT_TRACE_ENABLE || defined HCI_TRACE_OVER_TRANSPORT
const wiced_transport_cfg_t transport_cfg =
{
    .type = WICED_TRANSPORT_UART,
    .cfg =
    {
        .uart_cfg =
        {
            .mode = WICED_TRANSPORT_UART_HCI_MODE,
            .baud_rate =  HCI_UART_DEFAULT_BAUD
        },
    },
    .rx_buff_pool_cfg =
    {
        .buffer_size  = TRANS_UART_BUFFER_SIZE,
        .buffer_count = 1
    },
    .p_status_handler    = NULL,
    .p_data_handler      = NULL,
    .p_tx_complete_cback = NULL
};
#endif

/*******************************************************************
 * Function Prototypes
 ******************************************************************/
static wiced_bt_dev_status_t app_management_callback (wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data);
static void                  app_write_eir(void);
static int                   app_write_nvram(int nvram_id, int data_len, void *p_data);
static int                   app_read_nvram(int nvram_id, void *p_data, int data_len);

#if SEND_DATA_ON_INTERRUPT
static void                  app_tx_ack_timeout(uint32_t param);
static void                  app_interrupt_handler(void *data, uint8_t port_pin);
#endif
#ifdef HCI_TRACE_OVER_TRANSPORT
static void                  app_trace_callback(wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t* p_data);
#endif


extern void     wiced_bt_trace_array( const char *string, const uint8_t* array, const uint16_t len );
#if defined (CYW20706A2)
extern BOOL32 wiced_hal_puart_select_uart_pads(UINT8 rxdPin, UINT8 txdPin, UINT8 ctsPin, UINT8 rtsPin);
extern wiced_result_t wiced_bt_app_init( void );
#endif

#ifndef CYW20706A2
extern uint64_t clock_SystemTimeMicroseconds64();
#else
#include "rtc.h"
uint64_t clock_SystemTimeMicroseconds64(void)
{
    tRTC_REAL_TIME_CLOCK rtcClock;
    rtc_getRTCRawClock(&rtcClock);
    // To convert 128 kHz rtc timer to milliseconds divide it by 131: //128 kHz = 128 * 1024 = 131072; to microseconds: 1000000 / 131072 = 7.62939453125 (7.63)
    return rtcClock.rtc64 * 763 / 100;
}
#endif

void hci_iap2_init();
extern void wiced_hci_vsc_lib_init(void);       //link note:SDK 3.0 default no use this, should we keep this?
#if CONFIG_TX_PWR
#else
extern void poke_power_setting(uint8_t value);  //link note:SDK 3.0 default no use this, should we keep this?
#endif
static void spp_store_linkkey_name(void);

/*******************************************************************
 * Function Definitions
 ******************************************************************/

void buffer_report(char *msg)
{
    wiced_bt_buffer_statistics_t buffer_stats[5];
    wiced_result_t result;

    result = wiced_bt_get_buffer_usage (buffer_stats, sizeof(buffer_stats));

    if (result == WICED_BT_SUCCESS)
    {
        WICED_BT_TRACE("%s: pool %x %d %d/%d/%d\r\n", msg,
                buffer_stats[1].pool_id,
                buffer_stats[1].pool_size,
                buffer_stats[1].current_allocated_count,
                buffer_stats[1].max_allocated_count,
                buffer_stats[1].total_count);
        WICED_BT_TRACE("%s: pool %x %d %d/%d/%d\r\n", msg,
                buffer_stats[2].pool_id,
                buffer_stats[2].pool_size,
                buffer_stats[2].current_allocated_count,
                buffer_stats[2].max_allocated_count,
                buffer_stats[2].total_count);
    }
    else
        WICED_BT_TRACE("buffer_report: wiced_bt_get_buffer_usage failed, returned %d\r\n", result);
}

/*
 * Entry point to the application. Set device configuration and start BT
 * stack initialization.  The actual application initialization will happen
 * when stack reports that BT device is ready
 */
APPLICATION_START()
{
    wiced_result_t result;
    WICED_BT_TRACE("APPLICATION_START 0628 test by link,Free mem:%d\r\n", wiced_memory_get_free_bytes());
#if defined WICED_BT_TRACE_ENABLE || defined HCI_TRACE_OVER_TRANSPORT
#if 1
    hci_control_init();
#else
    wiced_transport_init(&transport_cfg);
#endif

    // create special pool for sending data to the MCU
    host_trans_pool = wiced_transport_create_buffer_pool(TRANS_UART_BUFFER_SIZE, TRANS_MAX_BUFFERS);

    // Set the debug uart as WICED_ROUTE_DEBUG_NONE to get rid of prints
    // wiced_set_debug_uart(WICED_ROUTE_DEBUG_NONE);

    // Set to PUART to see traces on peripheral uart(puart) if platform has PUART
#ifdef NO_PUART_SUPPORT
    // wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_WICED_UART );
#else
    wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_PUART );
#if defined (CYW20706A2)
    // wiced_hal_puart_select_uart_pads( WICED_PUART_RXD, WICED_PUART_TXD, 0, 0);
#endif
#endif

    // Set to HCI to see traces on HCI uart - default if no call to wiced_set_debug_uart()
    // wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_HCI_UART );

    // Use WICED_ROUTE_DEBUG_TO_WICED_UART to send formatted debug strings over the WICED
    // HCI debug interface to be parsed by ClientControl/BtSpy.
    //wiced_set_debug_uart(WICED_ROUTE_DEBUG_TO_WICED_UART);
#endif

WICED_BT_TRACE( "BT 89820 V%d.%d, Build %s %s \n\r", GARMIN_SW_MAJOR_VER, GARMIN_SW_MINOR_VER, __DATE__, __TIME__ );
WICED_BT_TRACE( "HCI TRACE:%d Baudrate:%d, Free mem:%d, 0709 test with delay\r\n", HCI_TRACE_OVER_TRANSPORT, HCI_UART_APP_BAUDRATE, wiced_memory_get_free_bytes());
    hci_control_handle_read_buffer_stats();
    /* Initialize Stack and Register Management Callback */
    // Register call back and configuration with stack
    wiced_bt_stack_init(app_management_callback, &wiced_bt_cfg_settings, wiced_bt_cfg_buf_pools);

#ifdef WICED_APP_LE_INCLUDED
    hci_control_le_init();
#endif

#ifdef WICED_APP_LE_SLAVE_CLIENT_INCLUDED
    le_slave_app_init();
#endif
    WICED_BT_TRACE("APPLICATION_START end,Free mem:%d\r\n", wiced_memory_get_free_bytes());
    hci_control_handle_read_buffer_stats();
}

/*
 * SPP application initialization is executed after BT stack initialization is completed.
 */
void application_init(void)
{
    wiced_bt_gatt_status_t gatt_status;
    wiced_result_t         result;
    uint8_t                tx_buf[8];
    uint16_t               *p_opcode = (uint16_t*) tx_buf;
    WICED_BT_TRACE("application_init ,Free mem:%d\r\n", wiced_memory_get_free_bytes());
    hci_control_handle_read_buffer_stats();

#if defined (CYW20706A2)
    /* Initialize wiced app */
    wiced_bt_app_init();

    /* Initialize the RTC block */
    rtc_init();
#endif

#if SEND_DATA_ON_INTERRUPT
#if !defined (CYW20706A2) && !defined (CYW43012C0)
    /* Configure the button available on the platform */
    wiced_platform_register_button_callback( WICED_PLATFORM_BUTTON_1, app_interrupt_handler, NULL, WICED_PLATFORM_BUTTON_RISING_EDGE);
#elif defined(CYW20706A2) || defined(CYW43012C0)
/* Initializes the GPIO driver */
    wiced_bt_app_hal_init();
    wiced_hal_gpio_configure_pin(WICED_GPIO_BUTTON, WICED_GPIO_BUTTON_SETTINGS( GPIO_EN_INT_RISING_EDGE ), WICED_GPIO_BUTTON_DEFAULT_STATE );
    wiced_hal_gpio_register_pin_for_interrupt(WICED_GPIO_BUTTON, app_interrupt_handler, NULL);
#endif // CYW20706A2 && CYW43012C0
    // init timer that we will use for the rx data flow control.
    wiced_init_timer(&app_tx_timer, app_tx_ack_timeout, 0, WICED_MILLI_SECONDS_TIMER);
#endif // SEND_DATA_ON_INTERRUPT

    app_write_eir();

#if defined (CYW20706A2)
    // Initialize RFCOMM.  We will not be using application buffer pool and will rely on the
    // stack pools configured in the wiced_bt_cfg.c
    wiced_bt_rfcomm_init(MAX_TX_BUFFER, 1);
#endif

    // Initialize SPP library
    if( wiced_bt_spp_startup( &spp_reg ) == WICED_SUCCESS )
    {
        WICED_BT_TRACE( "wiced_bt_spp_startup:  OK \n\r" );
    }
    else
    {
        WICED_BT_TRACE( "wiced_bt_spp_startup:  NOK \n\r" );
    }
    if( wiced_bt_spp_startup_2( &spp_reg_2 ) == WICED_SUCCESS )
    {
        WICED_BT_TRACE( "wiced_bt_spp_startup_2:  OK \n\r" );
    }
    else
    {
        WICED_BT_TRACE( "wiced_bt_spp_startup_2:  NOK \n\r" );
    }
    // iAP2 app init
    hci_iap2_init();

#if HCI_TRACE_OVER_TRANSPORT
    // There is a virtual HCI interface between upper layers of the stack and
    // the controller portion of the chip with lower layers of the BT stack.
    // Register with the stack to receive all HCI commands, events and data.
    wiced_bt_dev_register_hci_trace(app_trace_callback);
#endif
    WICED_BT_TRACE( "wiced_bt_sdp_db_init: sdp db size =%d \n\r", sizeof(app_sdp_db));
    /* create SDP records */
    if( wiced_bt_sdp_db_init((uint8_t *)app_sdp_db, sizeof(app_sdp_db)) == WICED_TRUE)
    {
        WICED_BT_TRACE( "wiced_bt_sdp_db_init:  OK \n\r" );
    }
    else
    {
        WICED_BT_TRACE( "wiced_bt_sdp_db_init:  NOK \n\r" );
    }
    /* Allow peer to pair */
    wiced_bt_set_pairable_mode(WICED_TRUE, 0);

    hci_control_post_init();

	wiced_bt_dev_set_discoverability( BTM_GENERAL_DISCOVERABLE,
									  wiced_bt_cfg_settings.br_edr_scan_cfg.inquiry_scan_interval,
									  wiced_bt_cfg_settings.br_edr_scan_cfg.inquiry_scan_window
									  );

	wiced_bt_dev_set_connectability( BTM_CONNECTABLE,
									 wiced_bt_cfg_settings.br_edr_scan_cfg.page_scan_interval,
									 wiced_bt_cfg_settings.br_edr_scan_cfg.page_scan_window
									 );

// This application will always configure device connectable and discoverable
#ifdef ADV_FOREVER
//    wiced_bt_dev_set_discoverability( BTM_GENERAL_DISCOVERABLE,
//                                      wiced_bt_cfg_settings.br_edr_scan_cfg.inquiry_scan_interval,
//                                      wiced_bt_cfg_settings.br_edr_scan_cfg.inquiry_scan_window
//                                      );
//
//    wiced_bt_dev_set_connectability( BTM_CONNECTABLE,
//                                     wiced_bt_cfg_settings.br_edr_scan_cfg.page_scan_interval,
//                                     wiced_bt_cfg_settings.br_edr_scan_cfg.page_scan_window
//                                     );
#else
    wiced_bt_dev_set_discoverability( BTM_NON_DISCOVERABLE,
                                      wiced_bt_cfg_settings.br_edr_scan_cfg.inquiry_scan_interval,
                                      wiced_bt_cfg_settings.br_edr_scan_cfg.inquiry_scan_window
                                      );

    wiced_bt_dev_set_connectability( BTM_CONNECTABLE,
                                     wiced_bt_cfg_settings.br_edr_scan_cfg.page_scan_interval,
                                     wiced_bt_cfg_settings.br_edr_scan_cfg.page_scan_window
                                     );
#endif

    wiced_bt_dev_register_connection_status_change( wiced_bt_connection_status_change_cb );

    //hci_read_nvram_link_keys();  //link note:should we keept this function?

#if SEND_DATA_ON_TIMEOUT
    /* Starting the app timers, seconds timer and the ms timer  */
    if (wiced_init_timer(&spp_app_timer, app_timeout, 0, WICED_SECONDS_PERIODIC_TIMER) == WICED_SUCCESS)
    {
        wiced_start_timer(&spp_app_timer, 1);
    }
#endif

//wiced_hci_vsc_lib_init();
#if 0
#if CONFIG_TX_PWR
    tx_buf[0] = 0xc9;   // 0xfdc9, Write_Tx_Power_Table
    tx_buf[1] = 0xfd;
    tx_buf[2] = 2;
    tx_buf[3] = 0;  // BR
    tx_buf[4] = CONFIGURED_BTC_TX_POWER;  // dBm
    result = wiced_bt_dev_vendor_specific_command( *p_opcode, tx_buf[2], &tx_buf[3], NULL );
    WICED_BT_TRACE( "Config BTC Tx power:0x%02x result:%d\n\r", CONFIGURED_BTC_TX_POWER, result );

    tx_buf[3] = 1;  // BLE
    tx_buf[4] = CONFIGURED_BLE_TX_POWER;  // dBm
    result = wiced_bt_dev_vendor_specific_command( *p_opcode, tx_buf[2], &tx_buf[3], NULL );
    WICED_BT_TRACE( "Config BLE Tx power:0x%02x result:%d\n\r", CONFIGURED_BLE_TX_POWER, result );
#else
    // add for BR/EDR power setting
    poke_power_setting( POKE_POWER );
    WICED_BT_TRACE("poke_power_setting value : %d\n\r", POKE_POWER);
#endif
#endif

    /*get pairlist buffer from pool*/
    WICED_BT_TRACE("size of device info =%d  , size of linkkey = %d \r\n", sizeof(BT_paired_device_info_t),
                                                                           sizeof(wiced_bt_device_link_keys_t));
    WICED_BT_TRACE("get pair list pool , pair list size = %d \r\n", sizeof(BT_paired_device_list_t));
    paired_list = wiced_bt_get_buffer(sizeof(BT_paired_device_list_t));
    if(paired_list == NULL)
    {
        WICED_BT_TRACE("get pair list from pool fail\r\n");
    }
    else
    {
        WICED_BT_TRACE("init pair list \r\n");
        memset(paired_list, 0, sizeof(BT_paired_device_list_t));
    }
    /*reload paired deivce list from NVM*/
    garmin_nvm_read_paired_device_num();
    if(paired_list->paired_device_num > MAX_PAIRED_DEVICE_NUM)
    {
        WICED_BT_TRACE("pair list in nvm crupt, erase all pair list nvm\r\n");
        garmin_nvm_erase_nvm_storage();
        paired_list->paired_device_num = 0;
    }
    else
    {
        garmin_nvm_read_paired_list();
        garmin_nvm_dump_pair_list();
        garmin_update_pairlist_to_mcu();
    }
    WICED_BT_TRACE("application_init finish,Free mem:%d\r\n", wiced_memory_get_free_bytes());
    hci_control_handle_read_buffer_stats();


    wiced_transport_send_data( HCI_CONTROL_EVENT_DEVICE_STARTED_GARMIN, sw_version, 2 );
}

/*
 *  Management callback receives various notifications from the stack
 */
wiced_result_t app_management_callback(wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data)
{
    wiced_result_t                      result = WICED_BT_SUCCESS;
    wiced_bt_dev_status_t               dev_status;
    wiced_bt_dev_pairing_info_t*        p_pairing_info;
    wiced_bt_dev_encryption_status_t*   p_encryption_status;
    int                                 bytes_written, bytes_read;
    int                                 nvram_id;
    wiced_bt_power_mgmt_notification_t* p_power_mgmt_notification;
    uint8_t                             numeric_pin_u8[sizeof(uint32_t)];
    wiced_bt_device_address_t           ble_remote_pairing_address;
    uint8_t                             ble_pairing_clt_evt_data[HCI_CONTROL_MISC_EVENT_PAIRING_CLT_DATA_LENGTH];
    uint8_t                             btc_pairing_clt_evt_data[HCI_CONTROL_MISC_EVENT_PAIRING_CLT_DATA_LENGTH];
    uint8_t                             linkkey_request_addr[BD_ADDR_LEN];
    uint8_t*                            p_local_keys;
    wiced_bt_dev_security_failed_t*     p_security_fail_data;
    uint8_t                             security_fail_data[MISC_SECURITY_FAIL_EVT_DATA_LENGTH];

    uint8_t device_id;
    wiced_bt_dev_le_key_type_t ble_key_type;
    wiced_bt_device_address_t  identical_addr;

    WICED_BT_TRACE("app_management_callback,Free mem:%d,event %d\r\n", wiced_memory_get_free_bytes(), event);
    switch(event)
    {
    /* Bluetooth  stack enabled */
    case BTM_ENABLED_EVT:
        application_init();
        //WICED_BT_TRACE("Free mem:%d", cfa_mm_MemFreeBytes());
        break;

    case BTM_DISABLED_EVT:
        break;

    case BTM_PIN_REQUEST_EVT:
        WICED_BT_TRACE("remote address= %B\r\n", p_event_data->pin_request.bd_addr);
        wiced_bt_dev_pin_code_reply(*p_event_data->pin_request.bd_addr,result/*WICED_BT_SUCCESS*/,4, &pincode[0]);
        break;

    case BTM_USER_CONFIRMATION_REQUEST_EVT:
        /* This application always confirms peer's attempt to pair */
        WICED_BT_TRACE( "BTM_USER_CONFIRMATION_REQUEST_EVT\n\r" );
        memcpy( user_confirm_result_data, &p_event_data->user_confirmation_request.bd_addr[0], BD_ADDR_LEN);

        numeric_pin_u8[0] = (uint8_t)p_event_data->user_confirmation_request.numeric_value;
        numeric_pin_u8[1] = (uint8_t)( p_event_data->user_confirmation_request.numeric_value >> 8 );
        numeric_pin_u8[2] = (uint8_t)( p_event_data->user_confirmation_request.numeric_value >> 16 );
        numeric_pin_u8[3] = (uint8_t)( p_event_data->user_confirmation_request.numeric_value >> 24 );
        memcpy( user_confirm_result_data + BD_ADDR_LEN, numeric_pin_u8, sizeof( uint32_t ) );
        wiced_bt_dev_confirm_req_reply( WICED_BT_SUCCESS, p_event_data->user_confirmation_request.bd_addr );
//        if( TRUE == ble_pairing_start )
//        {
//            WICED_BT_TRACE( "Send BLE HCI_CONTROL_EVENT_USER_CONFIRMATION_GARMIN result:%d numeric_value: %d\n\r", result, numeric_pin_u8 );
//            wiced_bt_dev_confirm_req_reply( WICED_BT_SUCCESS, p_event_data->user_confirmation_request.bd_addr );
//        }
//        else if( ( FALSE == ble_pairing_start ) && ( TRUE ==get_device_name_flag )  )
//        {
//            result = wiced_transport_send_data( HCI_CONTROL_EVENT_USER_CONFIRMATION_GARMIN, user_confirm_result_data, USER_CONFIRM_RESULT_DATA_LENGTH );
//            WICED_BT_TRACE( "Send BTC HCI_CONTROL_MISC_EVENT_USER_CONFIRM_RESULT result:%d numeric_value: %d\n\r", result, numeric_pin_u8 );
//            get_device_name_flag = FALSE;
//        }
//        else if( ( FALSE == ble_pairing_start ) && ( FALSE ==get_device_name_flag )  )
//        {
//            result = wiced_transport_send_data( HCI_CONTROL_EVENT_USER_CONFIRMATION_GARMIN, user_confirm_result_data, USER_CONFIRM_RESULT_DATA_LENGTH );
//            WICED_BT_TRACE( "ERROR:Not get BTC device name\n\r" );
//            WICED_BT_TRACE( "Send BTC HCI_CONTROL_EVENT_USER_CONFIRMATION_GARMIN result:%d numeric_value: %d\n\r", result, numeric_pin_u8 );
//        }
        break;

    case BTM_PAIRING_IO_CAPABILITIES_BR_EDR_REQUEST_EVT:
        /* This application supports only Just Works pairing */
        // Save BT address for the remote device name request
        WICED_BT_TRACE( "BTM_PAIRING_IO_CAPABILITIES_BR_EDR_REQUEST_EVT bda %B\r\n", p_event_data->pairing_io_capabilities_br_edr_request.bd_addr );
        p_event_data->pairing_io_capabilities_br_edr_request.local_io_cap   = BTM_IO_CAPABILITIES_DISPLAY_AND_YES_NO_INPUT;
        p_event_data->pairing_io_capabilities_br_edr_request.auth_req       = BTM_AUTH_SINGLE_PROFILE_GENERAL_BONDING_YES;
        memcpy( paired_device_bd_addr, p_event_data->user_confirmation_request.bd_addr, BD_ADDR_LEN );

        result = wiced_bt_dev_get_remote_name( paired_device_bd_addr, p_remote_name_result_cb );
        WICED_BT_TRACE( "Get remote name result:%d\n\r",result );
        ble_pairing_start = FALSE;
        break;

    case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT:
        WICED_BT_TRACE( "BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT bda %B\n\r", p_event_data->pairing_io_capabilities_ble_request.bd_addr );
        p_event_data->pairing_io_capabilities_ble_request.local_io_cap      = BTM_IO_CAPABILITIES_NONE;
        p_event_data->pairing_io_capabilities_ble_request.oob_data          = BTM_OOB_NONE;
        p_event_data->pairing_io_capabilities_ble_request.auth_req          = BTM_LE_AUTH_REQ_SC_MITM_BOND;
        p_event_data->pairing_io_capabilities_ble_request.max_key_size      = 0x10;
        p_event_data->pairing_io_capabilities_ble_request.init_keys         = BTM_LE_KEY_PENC|BTM_LE_KEY_PID|BTM_LE_KEY_PCSRK|BTM_LE_KEY_LENC;
        p_event_data->pairing_io_capabilities_ble_request.resp_keys         = BTM_LE_KEY_PENC|BTM_LE_KEY_PID|BTM_LE_KEY_PCSRK|BTM_LE_KEY_LENC;
        ble_pairing_start = TRUE;

        //result = wiced_bt_dev_get_ble_keys(p_event_data->pairing_io_capabilities_ble_request.bd_addr, &ble_key_type);
        //WICED_BT_TRACE( "test to get BLE key result =%d, mask =%d \n\r", result, ble_key_type);

        break;

    case BTM_PAIRING_COMPLETE_EVT:
        wiced_bt_trace_array( "BTM_PAIRING_COMPLETE_EVT,addr = %d", p_event_data->pairing_complete.bd_addr, BD_ADDR_LEN);
        WICED_BT_TRACE( "transport type = %d\r\n", p_event_data->pairing_complete.transport);
        p_pairing_info = &p_event_data->pairing_complete.pairing_complete_info;

        if( BT_TRANSPORT_BR_EDR == p_event_data->pairing_complete.transport )
            {
            btc_pairing_clt_evt_data[0] = BT_TRANSPORT_BR_EDR;
            btc_pairing_clt_evt_data[1] = p_pairing_info->br_edr.status;
            memcpy( &btc_pairing_clt_evt_data[2], &( paired_device_bd_addr ), BD_ADDR_LEN );
            result = wiced_transport_send_data( HCI_CONTROL_MISC_EVENT_PAIRING_CLT, btc_pairing_clt_evt_data, HCI_CONTROL_MISC_EVENT_PAIRING_CLT_DATA_LENGTH );
            }
        else if ( BT_TRANSPORT_LE == p_event_data->pairing_complete.transport )
            {
            ble_pairing_start = FALSE;
            WICED_BT_TRACE( "BLE Pairing Complete:%d reason:%d bd_address:%B\n\r", p_pairing_info->ble.status, p_pairing_info->ble.reason, p_pairing_info->ble.resolved_bd_addr );
            //ble_remote_pairing_address = p_pairing_info->ble.resolved_bd_addr;
            ble_pairing_clt_evt_data[0] = BT_TRANSPORT_LE;
            ble_pairing_clt_evt_data[1] = p_pairing_info->ble.status;
            memcpy( &ble_pairing_clt_evt_data[2], &( p_pairing_info->ble.resolved_bd_addr ), BD_ADDR_LEN );
            result = wiced_transport_send_data( HCI_CONTROL_MISC_EVENT_PAIRING_CLT, ble_pairing_clt_evt_data, HCI_CONTROL_MISC_EVENT_PAIRING_CLT_DATA_LENGTH );
            }
        result = WICED_BT_USE_DEFAULT_SECURITY;
        break;

    case BTM_ENCRYPTION_STATUS_EVT:
        p_encryption_status = &p_event_data->encryption_status;
        WICED_BT_TRACE( "Encryption Status Event: bd (%B) res %d\n\r", p_encryption_status->bd_addr, p_encryption_status->result );

// Added for ANCS, 20May21
#if 1   // Begin
#ifdef WICED_APP_LE_SLAVE_CLIENT_INCLUDED
        if (p_encryption_status->transport == BT_TRANSPORT_LE)
            le_slave_encryption_status_changed(p_encryption_status);
#endif
        hci_control_send_encryption_changed_evt( p_encryption_status->result, p_encryption_status->bd_addr );
#endif   // End of added, 20May21
        break;

    case BTM_SECURITY_REQUEST_EVT:
        wiced_bt_ble_security_grant( p_event_data->security_request.bd_addr, WICED_BT_SUCCESS );
        break;

    case BTM_SECURITY_FAILED_EVT:
        p_security_fail_data = &p_event_data->security_failed;
        WICED_BT_TRACE( "BTM_SECURITY_FAILED_EVT: bd (%B) result:%d status:%d\n\r", p_security_fail_data->bd_addr, p_security_fail_data->status, p_security_fail_data->hci_status );
        security_fail_data[0] = (uint8_t)p_security_fail_data->status;
        security_fail_data[1] = (uint8_t)p_security_fail_data->hci_status;
        memcpy( &(security_fail_data[2]), p_security_fail_data->bd_addr, BD_ADDR_LEN );
        wiced_transport_send_data( HCI_CONTROL_MISC_EVENT_SECURITY_FAIL, security_fail_data, MISC_SECURITY_FAIL_EVT_DATA_LENGTH );
        break;

    case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT:
#if 0   // reserve for ALLTEK Sample code
        /* This application supports a single paired host, we can save keys under the same NVRAM ID overwriting previous pairing if any */
        //app_write_nvram(PAIR_DEVICE1_NVRAM_ID + , sizeof(wiced_bt_device_link_keys_t), &p_event_data->paired_device_link_keys_update);
#else
        // Save updated link key for remote device name request
        //memcpy( &saved_link_key, &p_event_data->paired_device_link_keys_update, sizeof( wiced_bt_device_link_keys_t ) );
        // Store linkkey and name into Flash
        //spp_store_linkkey_name();
        WICED_BT_TRACE( "BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT, dump link key struct\n\r");
        garmin_nvm_dump_link_key(&p_event_data->paired_device_link_keys_update);

        /*check device addr is in pair list or not*/
        device_id = garmin_nvm_find_device_id(p_event_data->paired_device_link_keys_update.bd_addr);
        if(INVAILD_ID == device_id)
        {
            BT_paired_device_info_t new_device;
            uint8_t           data[1 + BD_ADDR_LEN + MAX_DEVICE_NAME_LEN];
            wiced_result_t    result_tt;

            WICED_BT_TRACE( "device not in paired, add new paired device into paired list\n\r");
            memcpy(&(new_device.device_name[0]), &(paired_device_name[0]), MAX_DEVICE_NAME_LEN);
            memcpy(&(new_device.link_key), &p_event_data->paired_device_link_keys_update, sizeof( wiced_bt_device_link_keys_t ));
            /*add device to list*/
            garmin_nvm_add_paired_device(&new_device);
            /*becasue device list is change, save into NVRAM*/
            garmin_nvm_save_paired_list();
            /*save to resolution DB*/
            result_tt = wiced_bt_dev_add_device_to_address_resolution_db(&(p_event_data->paired_device_link_keys_update));
            WICED_BT_TRACE( "set link key to resolution DB result = %d \n\r", result_tt);
            /*also sent to MCU*/ /*link note: TBD*/
            garmin_update_pairlist_to_mcu();

            garmin_nvm_dump_pair_list();
        }
        else
        {
            //link note:TBD,check if it is update by BLE second pairing(work around)
            WICED_BT_TRACE( "device in paired list, update data id = %d \n\r", device_id);
        }
#endif
        break;


    case  BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
#if 0   // reserve for ALLTEK Sample code
        /* read existing key from the NVRAM  */
        if (app_read_nvram(SPP_NVRAM_ID, &p_event_data->paired_device_link_keys_request, sizeof(wiced_bt_device_link_keys_t)) != 0)
        {
            result = WICED_BT_SUCCESS;
        }
        else
        {
            result = WICED_BT_ERROR;
            WICED_BT_TRACE("Key retrieval failure\n");
        }
#endif
#if 0       //reserve our old code
        memcpy( linkkey_request_addr, p_event_data->paired_device_link_keys_request.bd_addr, BD_ADDR_LEN );
        /* read existing key from the NVRAM  */
        nvram_id = hci_hal_control_find_nvram_id( linkkey_request_addr, BD_ADDR_LEN );
        if( nvram_id != 0 )
            {
            bytes_read = wiced_hal_read_nvram( nvram_id, sizeof( wiced_bt_device_link_keys_t ), (uint8_t *)&(p_event_data->paired_device_link_keys_request), &result );
            result = WICED_BT_SUCCESS;
            WICED_BT_TRACE("Read:nvram_id:%d bytes:%d\n\r", nvram_id, bytes_read);
            }
        else
            {
            result = WICED_BT_ERROR;
            WICED_BT_TRACE( "Key retrieval failure\n\r" );
            }
#endif
        WICED_BT_TRACE( "BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT : bd (%B)\n\r", p_event_data->paired_device_link_keys_request.bd_addr);
        //result = wiced_bt_get_identity_address(p_event_data->paired_device_link_keys_request.bd_addr,identical_addr);
        //WICED_BT_TRACE( "Find identical addr : bd (%B), result =%d \n\r", identical_addr, result);
        /*check device addr is in pair list or not*/
        device_id = garmin_nvm_find_device_id(p_event_data->paired_device_link_keys_request.bd_addr);
        if(INVAILD_ID == device_id)
        {
            WICED_BT_TRACE( "Find no pairing record, linkkey missing, restart pairing flow \n\r");

            wiced_transport_send_data(HCI_CONTROL_EVENT_AUTH_LOST_GARMIN, &(p_event_data->paired_device_link_keys_request.bd_addr[0]), BD_ADDR_LEN);
            result = WICED_BT_ERROR;
        }
        else
        {
            wiced_result_t    result_tt;
            WICED_BT_TRACE( "Find paired device info, id =%d \n\r", device_id);
            result_tt = wiced_bt_dev_add_device_to_address_resolution_db(&p_event_data->paired_device_link_keys_update);
            WICED_BT_TRACE( "set  link key to resolution DB result = %d \n\r", result_tt);
            memcpy(&(p_event_data->paired_device_link_keys_request),
                   &(paired_list->device[device_id].link_key),
                   sizeof(wiced_bt_device_link_keys_t));
            result = WICED_BT_SUCCESS;
        }
        break;

    case BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT:
        /* save keys to NVRAM */
        p_local_keys = (uint8_t*)&p_event_data->local_identity_keys_update;
        wiced_hal_write_nvram ( WICED_NVRAM_VSID_START, sizeof( wiced_bt_local_identity_keys_t ), p_local_keys ,&result );
        WICED_BT_TRACE("local keys save to NVRAM result: %d\n\r", result);
        break;

    case  BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT:
        /* read keys from NVRAM */
        p_local_keys = (uint8_t *)&p_event_data->local_identity_keys_request;
        wiced_hal_read_nvram( WICED_NVRAM_VSID_START, sizeof(wiced_bt_local_identity_keys_t), p_local_keys, &result );
        WICED_BT_TRACE("local keys read from NVRAM result: %d\n\r",  result);
        break;

    case BTM_POWER_MANAGEMENT_STATUS_EVT:
        p_power_mgmt_notification = &p_event_data->power_mgmt_notification;
        WICED_BT_TRACE("Power mgmt status event: bd (%B) status:%d hci_status:%d\r\n", p_power_mgmt_notification->bd_addr, \
                p_power_mgmt_notification->status, p_power_mgmt_notification->hci_status);
        break;

#ifdef WICED_APP_LE_INCLUDED
    case BTM_BLE_SCAN_STATE_CHANGED_EVT:
        hci_control_le_scan_state_changed( p_event_data->ble_scan_state_changed );
        break;

    case BTM_BLE_ADVERT_STATE_CHANGED_EVT:
        hci_control_le_advert_state_changed( p_event_data->ble_advert_state_changed );
#ifdef ADV_FOREVER   // advertisement forever
        if( BTM_BLE_ADVERT_OFF == p_event_data->ble_advert_state_changed )
        {
            //link note: why public? and why state change and we set advertise again?
        	WICED_BT_TRACE("ADV_FOREVER is be setup \n\r");
            wiced_bt_start_advertisements( BTM_BLE_ADVERT_UNDIRECTED_HIGH, BLE_ADDR_PUBLIC, NULL );
        }
#endif
        break;

    case BTM_BLE_CONNECTION_PARAM_UPDATE:
        WICED_BT_TRACE("Interval: 0x%04x ", p_event_data->ble_connection_param_update.conn_interval);
        WICED_BT_TRACE("Latency: 0x%04x ", p_event_data->ble_connection_param_update.conn_latency);
        WICED_BT_TRACE("Timeout: 0x%04x\n\r", p_event_data->ble_connection_param_update.supervision_timeout);
        ble_pairing_start = TRUE;
        break;

#endif

    default:
        result = WICED_BT_USE_DEFAULT_SECURITY;
        WICED_BT_TRACE("unhandle evnet\n\r");
        break;
    }
    return result;
}


/*
 *  Prepare extended inquiry response data.  Current version publishes device name and 16bit
 *  SPP service.
 */
void app_write_eir(void)
{
    uint8_t *pBuf;
    uint8_t *p;
    uint8_t length;
    uint16_t eir_length;
    uint8_t iap2_service_uuid[] = { IAP2_ACCESSORY_UUID };

    pBuf = (uint8_t *)wiced_bt_get_buffer(WICED_EIR_BUF_MAX_SIZE);
    WICED_BT_TRACE("hci_control_write_eir %x\r\n", pBuf);

    if (!pBuf)
    {
        WICED_BT_TRACE("app_write_eir %x\r\n", pBuf);
        return;
    }

    p = pBuf;

    length = strlen((char *)wiced_bt_cfg_settings.device_name);

    *p++ = length + 1;
    *p++ = BT_EIR_COMPLETE_LOCAL_NAME_TYPE;        // EIR type full name
    memcpy(p, wiced_bt_cfg_settings.device_name, length);
    p += length;

    *p++ = 2 + 1;                                   // Length of 16 bit services
    *p++ = BT_EIR_COMPLETE_16BITS_UUID_TYPE;        // 0x03 EIR type full list of 16 bit service UUIDs
    *p++ = NAVI_UUID_U16 & 0xff;
    *p++ = ( NAVI_UUID_U16 >> 8 ) & 0xff;

    *p++ = 2 + 1;                                   // Length of 16 bit services
    *p++ = BT_EIR_COMPLETE_16BITS_UUID_TYPE;        // 0x03 EIR type full list of 16 bit service UUIDs
    *p++ = YCONNECT_UUID_U16 & 0xff;
    *p++ = ( YCONNECT_UUID_U16 >> 8 ) & 0xff;

    /*add iap2 EIR*/
    *p++ = 16 + 1;                                  // length of 128 bit services + 1
    *p++ = BT_EIR_COMPLETE_128BITS_UUID_TYPE;       // 0x07 EIR type full list of 128 bit service UUIDs
    memcpy( p, iap2_service_uuid, 16 );
    p += 16;

    *p++ = 0;                                       // end of EIR Data is 0

    eir_length = (uint16_t) (p - pBuf);

    // print EIR data
    wiced_bt_trace_array("EIR :", pBuf, MIN(p-pBuf, 100));
    wiced_bt_dev_write_eir(pBuf, eir_length);

    return;
}

/*
 * The function invoked on timeout of app seconds timer.
 */
#if SEND_DATA_ON_TIMEOUT
void app_timeout(uint32_t count)
{
    static uint32_t timer_count = 0;
    timer_count++;
    wiced_bool_t ret;
    WICED_BT_TRACE("app_timeout: %d, handle %d \r\n", timer_count, spp_handle);
    if (spp_handle != 0)
    {
        ret = wiced_bt_spp_send_session_data(spp_handle, (uint8_t *)&timer_count, sizeof(uint32_t));
        if (ret != WICED_TRUE)
            WICED_BT_TRACE("wiced_bt_spp_send_session_data failed, ret = %d\r\n", ret);
    }
}
#endif

/*
 * SPP connection up callback
 */
void spp_connection_up_callback(uint16_t handle, uint8_t* bda)
{
    uint8_t   tx_buf[9];
    wiced_result_t result;

    WICED_BT_TRACE( "%s handle:%d address:%B\n\r ", __FUNCTION__, handle, bda );
    //Byte[0]: (6-bytes) Bluetooth device address
    memcpy(&tx_buf[0], bda, BD_ADDR_LEN);
    //Byte[6]: (2-bytes) Connection handle
    memcpy(&tx_buf[6], handle, sizeof(uint16_t));
    //Byte[8]: (1-byte) SPP application type specified in BT_spp_app_type_e
    tx_buf[8] = BT_SPP_APP_NAVILITE;
    //link note: TBD
    spp_handle = handle;
    spp_rx_bytes = 0;
    spp_connected = TRUE;

    wiced_transport_send_data(HCI_CONTROL_SPP_EVENT_CONNECTED_GARMIN, &tx_buf[0], 9);
}
void spp_connection_up_callback_2(uint16_t handle,uint8_t* bda)
{
    uint8_t   tx_buf[9];
    wiced_result_t result;

    WICED_BT_TRACE( "%s handle:%d address:%B\n\r ", __FUNCTION__, handle, bda );
    //Byte[0]: (6-bytes) Bluetooth device address
    memcpy(&tx_buf[0], bda, BD_ADDR_LEN);
    //Byte[6]: (2-bytes) Connection handle
    memcpy(&tx_buf[6], handle, sizeof(uint16_t));
    //Byte[8]: (1-byte) SPP application type specified in BT_spp_app_type_e
    tx_buf[8] = BT_SPP_APP_MOTOCON;
    //link note: TBD
    spp_handle_2 = handle;
    spp_rx_bytes_2 = 0;
    spp_connected_2 = TRUE;

    wiced_transport_send_data(HCI_CONTROL_SPP_EVENT_CONNECTED_GARMIN, &tx_buf[0], 9);
}
/*
 * SPP connection down callback
 */
void spp_connection_down_callback(uint16_t handle)
{
    uint8_t   tx_buf[3];
    WICED_BT_TRACE("%s handle:%d rx_bytes:%d\r\n", __FUNCTION__, handle, spp_rx_bytes);
#if defined(SEND_DATA_ON_INTERRUPT) && (SEND_DATA_ON_INTERRUPT==1)
    app_send_offset = 0;
    spp_tx_retry_count = 0;

    if(wiced_is_timer_in_use(&app_tx_timer))
    wiced_stop_timer(&app_tx_timer);
#endif
    //Byte[0]: (2-bytes) Connection handle
    memcpy(&tx_buf[0], handle, sizeof(uint16_t));
    //Byte[2]: (1-byte) SPP application type specified in BT_spp_app_type_e
    tx_buf[2] = BT_SPP_APP_NAVILITE;
    spp_handle = 0;
    spp_connected = FALSE;
    wiced_transport_send_data( HCI_CONTROL_SPP_EVENT_DISCONNECTED_GARMIN, &tx_buf[0], 3 );
}
void spp_connection_down_callback_2(uint16_t handle)
{
    uint8_t   tx_buf[3];
    WICED_BT_TRACE("%s handle:%d rx_bytes:%d\r\n", __FUNCTION__, handle, spp_rx_bytes);
#if defined(SEND_DATA_ON_INTERRUPT) && (SEND_DATA_ON_INTERRUPT==1)
    app_send_offset = 0;
    spp_tx_retry_count = 0;

    if(wiced_is_timer_in_use(&app_tx_timer))
    wiced_stop_timer(&app_tx_timer);
#endif
    //Byte[0]: (2-bytes) Connection handle
    memcpy(&tx_buf[0], handle, sizeof(uint16_t));
    //Byte[2]: (1-byte) SPP application type specified in BT_spp_app_type_e
    tx_buf[2] = BT_SPP_APP_MOTOCON;
    spp_handle_2 = 0;
    spp_connected_2 = FALSE;
    wiced_transport_send_data( HCI_CONTROL_SPP_EVENT_DISCONNECTED_GARMIN, &tx_buf[0], 3 );
}
/*
 * Service not found failed indication from the library
 */
void spp_service_not_found_callback(void)
{
    uint8_t   tx_buf[10];
    uint8_t  *p = tx_buf;

    WICED_BT_TRACE( "%s\n\r", __FUNCTION__ );

    *p++ = 0;
    *p++ = 0;

    wiced_transport_send_data( HCI_CONTROL_SPP_EVENT_SERVICE_NOT_FOUND, tx_buf, (int)( p - tx_buf ) );
}
void spp_service_not_found_callback_2(void)
{
    uint8_t   tx_buf[10];
    uint8_t  *p = tx_buf;

    WICED_BT_TRACE( "%s\n\r", __FUNCTION__ );

    *p++ = 0;
    *p++ = 0;

    wiced_transport_send_data( HCI_CONTROL_SPP_EVENT_SERVICE_NOT_FOUND_2, tx_buf, (int)( p - tx_buf ) );
}

/*
 * SPP connection build failed callback
 */
void spp_connection_failed_callback(void)
{
    WICED_BT_TRACE( "%s:app type =%d \n\r", __FUNCTION__, BT_SPP_APP_NAVILITE );

    wiced_transport_send_data( HCI_CONTROL_SPP_EVENT_CONNECTION_FAILED_GARMIN, BT_SPP_APP_NAVILITE, 1 );
}
void spp_connection_failed_callback_2(void)
{
    WICED_BT_TRACE( "%s:app type =%d \n\r", __FUNCTION__, BT_SPP_APP_MOTOCON );

    wiced_transport_send_data( HCI_CONTROL_SPP_EVENT_CONNECTION_FAILED_GARMIN, BT_SPP_APP_MOTOCON, 1 );
}
/*
 * Process data received over EA session.  Return TRUE if we were able to allocate buffer to
 * deliver to the host.
 */
wiced_bool_t spp_rx_data_callback(uint16_t handle, uint8_t* p_data, uint32_t data_len)
{
    uint8_t* tx_buf = NULL;
    uint8_t send_data_count = 0;
    uint8_t rest_data_len = 0;
    uint8_t tx_buf_len = MIN(data_len + 3, MAX_TRANSPORT_DATA_SIZE);
    tx_buf = wiced_bt_get_buffer(tx_buf_len);   //allocate buf to send to MCU
    if(tx_buf == NULL)
    {
        WICED_BT_TRACE("get spp buf to send to MCU fail\r\n");
        return WICED_FALSE;
    }

    spp_rx_bytes += data_len;
//    int i;
//    wiced_bt_buffer_statistics_t buffer_stats[4];

//    wiced_bt_get_buffer_usage (buffer_stats, sizeof(buffer_stats));

//    WICED_BT_TRACE("0:%d/%d 1:%d/%d 2:%d/%d 3:%d/%d\n", buffer_stats[0].current_allocated_count, buffer_stats[0].max_allocated_count,
//                   buffer_stats[1].current_allocated_count, buffer_stats[1].max_allocated_count,
//                   buffer_stats[2].current_allocated_count, buffer_stats[2].max_allocated_count,
//                   buffer_stats[3].current_allocated_count, buffer_stats[3].max_allocated_count);
//    buffer_report("spp_rx_data_callback");

//    wiced_result_t wiced_bt_get_buffer_usage (&buffer_stats, sizeof(buffer_stats));

    //Byte[0]: (2-bytes) Connection handle
    memcpy(tx_buf, spp_handle, sizeof(uint16_t));
    //Byte[2]: (1-byte) SPP application type specified in BT_spp_app_type_e
    *(tx_buf+2) = BT_SPP_APP_NAVILITE;

    if( ( SPP_MAX_PAYLOAD - 17 ) != data_len )
    {
        WICED_BT_TRACE( "0x%02x%02x : 0x%02x%02x rx_byte:%d \n\r", p_data[0], p_data[1], p_data[data_len - 2], p_data[data_len - 1], spp_rx_bytes );
    }
    while( (data_len - send_data_count) > (MAX_TRANSPORT_DATA_SIZE -3) ) /*3 byte for handle and length*/
    {
        WICED_BT_TRACE("spp rx data is too big, separate sending\r\n");

        //Byte[3]: (n-bytes) Data from remote device, where n <= SPP_DATA_MAX_SIZE
        memcpy(tx_buf+3, p_data+send_data_count, MAX_TRANSPORT_DATA_SIZE -3);
        wiced_transport_send_data( HCI_CONTROL_SPP_EVENT_RX_DATA_GARMIN, p_data, MAX_TRANSPORT_DATA_SIZE);
        send_data_count += (MAX_TRANSPORT_DATA_SIZE -3);
        memset(tx_buf+3, 0, MIN(data_len + 3, MAX_TRANSPORT_DATA_SIZE));
    }
    WICED_BT_TRACE("spp rx data is under uart transmit limit, sending all data\r\n");
    //Byte[3]: (n-bytes) Data from remote device, where n <= SPP_DATA_MAX_SIZE
    memcpy(tx_buf+3, p_data+send_data_count, data_len - send_data_count);
    wiced_transport_send_data( HCI_CONTROL_SPP_EVENT_RX_DATA_GARMIN, p_data, (data_len - send_data_count)+3);
    wiced_bt_free_buffer(tx_buf);

    WICED_BT_TRACE("%s handle:%d len:%d %02x-%02x, total rx %d\r\n", __FUNCTION__, handle, data_len, p_data[0], p_data[data_len - 1], spp_rx_bytes);

#if LOOPBACK_DATA
    return wiced_bt_spp_send_session_data(handle, p_data, data_len);
#else
    return WICED_TRUE;
#endif
}
wiced_bool_t spp_rx_data_callback_2(uint16_t handle,uint8_t* p_data,uint32_t data_len)
{
    uint8_t* tx_buf = NULL;
    uint8_t send_data_count = 0;
    uint8_t rest_data_len = 0;
    uint8_t tx_buf_len = MIN(data_len + 3, MAX_TRANSPORT_DATA_SIZE);
    tx_buf = wiced_bt_get_buffer(tx_buf_len);   //allocate buf to send to MCU
    if(tx_buf == NULL)
    {
        WICED_BT_TRACE("get spp buf to send to MCU fail\r\n");
        return WICED_FALSE;
    }
    spp_rx_bytes_2 += data_len;

//    int i;
//    wiced_bt_buffer_statistics_t buffer_stats[4];

//    wiced_bt_get_buffer_usage (buffer_stats, sizeof(buffer_stats));

//    WICED_BT_TRACE("0:%d/%d 1:%d/%d 2:%d/%d 3:%d/%d\n", buffer_stats[0].current_allocated_count, buffer_stats[0].max_allocated_count,
//                   buffer_stats[1].current_allocated_count, buffer_stats[1].max_allocated_count,
//                   buffer_stats[2].current_allocated_count, buffer_stats[2].max_allocated_count,
//                   buffer_stats[3].current_allocated_count, buffer_stats[3].max_allocated_count);
//    buffer_report("spp_rx_data_callback");

//    wiced_result_t wiced_bt_get_buffer_usage (&buffer_stats, sizeof(buffer_stats));

    //Byte[0]: (2-bytes) Connection handle
    memcpy(tx_buf, spp_handle, sizeof(uint16_t));
    //Byte[2]: (1-byte) SPP application type specified in BT_spp_app_type_e
    *(tx_buf+2) = BT_SPP_APP_MOTOCON;

    if( ( SPP_MAX_PAYLOAD - 17 ) != data_len )
    {
        WICED_BT_TRACE( "0x%02x%02x : 0x%02x%02x rx_byte:%d \n\r", p_data[0], p_data[1], p_data[data_len - 2], p_data[data_len - 1], spp_rx_bytes_2 );
    }
    while( (data_len - send_data_count + 3) > MAX_TRANSPORT_DATA_SIZE ) /*3 byte for handle and length*/
    {
        WICED_BT_TRACE("spp rx data is too big, separate sending\r\n");
        //Byte[3]: (n-bytes) Data from remote device, where n <= SPP_DATA_MAX_SIZE
        memcpy(tx_buf+3, p_data+send_data_count, MAX_TRANSPORT_DATA_SIZE -3);
        wiced_transport_send_data( HCI_CONTROL_SPP_EVENT_RX_DATA_GARMIN, p_data, MAX_TRANSPORT_DATA_SIZE);
        send_data_count += (MAX_TRANSPORT_DATA_SIZE -3);
        memset(tx_buf+3, 0, MAX_TRANSPORT_DATA_SIZE);
    }
    WICED_BT_TRACE("spp rx data is under uart transmit limit, sending all data\r\n");
    //Byte[3]: (n-bytes) Data from remote device, where n <= SPP_DATA_MAX_SIZE
    memcpy(tx_buf+3, p_data+send_data_count, data_len - send_data_count);
    wiced_transport_send_data( HCI_CONTROL_SPP_EVENT_RX_DATA_GARMIN, p_data, (data_len - send_data_count)+3);
    wiced_bt_free_buffer(tx_buf);

    WICED_BT_TRACE("%s handle:%d len:%d %02x-%02x, total rx %d\r\n", __FUNCTION__, handle, data_len, p_data[0], p_data[data_len - 1], spp_rx_bytes_2);

#if LOOPBACK_DATA
    return wiced_bt_spp_send_session_data(handle, p_data, data_len);
#else
    return WICED_TRUE;
#endif
}
/*
 * Handle various SPP commands received over UART
 */
void spp_handle_command(uint16_t cmd_opcode, uint8_t* p, uint32_t data_len)
{
    uint16_t handle = p[0] | (p[1] << 8);
    BT_spp_app_type_e app_type = BT_SPP_APP_TYPE_INVALID;
    BD_ADDR addr;
    uint16_t handle_garmin = 0;
    WICED_BT_TRACE("spp_handle_command, opcode:0x%x, len:%d\r\npdata:\t", cmd_opcode, data_len);
    for (int i=0 ; i<data_len ; i++)
    {
        WICED_BT_TRACE("%x ", p[i]);
    }
    WICED_BT_TRACE("\r\n");

    switch (cmd_opcode)
    {
    case HCI_CONTROL_SPP_COMMAND_CONNECT:
        wiced_bt_spp_connect( p );
        break;

    case HCI_CONTROL_SPP_COMMAND_DISCONNECT:
        wiced_bt_spp_disconnect(handle);
        break;

    case HCI_CONTROL_SPP_COMMAND_DATA:
        WICED_BT_TRACE("COMMAND_DATA handle:%d length:%d\r\n", handle, data_len - 2);

        if (data_len > SPP_MAX_PACKET_SIZE)
        {
            WICED_BT_TRACE("too many bytes\r\n");
            return;
        }
        //      wiced_bt_get_buffer_usage (buffer_stats, sizeof(buffer_stats));

        //      WICED_BT_TRACE("0:%d/%d 1:%d/%d 2:%d/%d 3:%d/%d\r\n", buffer_stats[0].current_allocated_count, buffer_stats[0].max_allocated_count,
        //                     buffer_stats[1].current_allocated_count, buffer_stats[1].max_allocated_count,
        //                     buffer_stats[2].current_allocated_count, buffer_stats[2].max_allocated_count,
        //                     buffer_stats[3].current_allocated_count, buffer_stats[3].max_allocated_count);

        // first 2 bytes should be set by the application with the sessio
        wiced_bt_spp_send_session_data(handle, p + 2, data_len - 2);
        break;

/*Add by Garmin define command */
    case HCI_CONTROL_SPP_COMMAND_CONNECT_GARMIN:

        /* p[0]~p[5] :bt address*/
        /* link note : why should we need reverse address*/
        memcpy(&(addr[0]), p, BD_ADDR_LEN);
        /* p[6] : BT_spp_app_type_e*/
        app_type = *(p + 6);
        WICED_BT_TRACE("Garmin SPP connect command, addr(rev):%B app type:%d\r\n", &addr[0], app_type);
        switch(app_type)
        {
            case BT_SPP_APP_NAVILITE:
            {
                wiced_bt_spp_connect(addr);
                break;
            }
            case BT_SPP_APP_MOTOCON:
            {
                wiced_bt_spp_connect_2(addr);
                break;
            }
        }

        break;
    case HCI_CONTROL_SPP_COMMAND_DISCONNECT_GARMIN:

        /* p[0]~p[1] :Connection handle*/
        memcpy(&(handle_garmin), p, 2);
        /* p[2] : BT_spp_app_type_e*/
        app_type = *(p + 2);
        WICED_BT_TRACE("Garmin SPP disconnect command, handle:%d app type:%d\r\n", handle_garmin, app_type);
        wiced_bt_spp_disconnect(handle_garmin);
//link note: should we use app_type?
#if 0
        switch(app_type)
        {
            case BT_SPP_APP_NAVILITE:
            {
                wiced_bt_spp_disconnect(handle_garmin);
                break;
            }
            case BT_SPP_APP_MOTOCON:
            {
                wiced_bt_spp_disconnect(handle_garmin);
                break;
            }
        }
#endif
        break;
    case HCI_CONTROL_SPP_COMMAND_DATA_GARMIN:

        /* p[0]~p[1] :Connection handle*/
        memcpy(&(handle_garmin), p, 2);
        /* p[2] : BT_spp_app_type_e*/
        app_type = *(p + 2);
        WICED_BT_TRACE("Garmin SPP send data command, handle:%d app type:%d,data len =%d\r\n", handle_garmin, app_type, data_len-3);
        if (data_len > SPP_MAX_PACKET_SIZE)
        {
            WICED_BT_TRACE("too many bytes\r\n");
            return;
        }
        //      wiced_bt_get_buffer_usage (buffer_stats, sizeof(buffer_stats));

        //      WICED_BT_TRACE("0:%d/%d 1:%d/%d 2:%d/%d 3:%d/%d\r\n", buffer_stats[0].current_allocated_count, buffer_stats[0].max_allocated_count,
        //                     buffer_stats[1].current_allocated_count, buffer_stats[1].max_allocated_count,
        //                     buffer_stats[2].current_allocated_count, buffer_stats[2].max_allocated_count,
        //                     buffer_stats[3].current_allocated_count, buffer_stats[3].max_allocated_count);

        // first 3 bytes should be set by the application with the sessio
        wiced_bt_spp_send_session_data(handle_garmin, p + 3, data_len - 3);
//link note: should we use app_type?
#if 0
        switch(app_type)
        {
            case BT_SPP_APP_NAVILITE:
            {
                wiced_bt_spp_send_session_data(handle_garmin, p + 3, data_len - 3);
                break;
            }
            case BT_SPP_APP_MOTOCON:
            {
                wiced_bt_spp_send_session_data(handle_garmin, p + 3, data_len - 3);
                break;
            }
        }
#endif
        break;
    default:
        WICED_BT_TRACE("spp_handle_command:unknown opcode %x\r\n", cmd_opcode);
        break;
    }
}

/*
 * Write NVRAM function is called to store information in the NVRAM.
 */
//note by link:this function in SDK 2.6 is hci_control_write_nvram, but now seperate as another function
int app_write_nvram(int nvram_id, int data_len, void *p_data)
{
    wiced_result_t  result;
    int             bytes_written = wiced_hal_write_nvram(nvram_id, data_len, (uint8_t*)p_data, &result);

    WICED_BT_TRACE("NVRAM ID:%d written :%d bytes result:%d\r\n", nvram_id, bytes_written, result);
    return (bytes_written);
}

/*
 * Write NVRAM function is called to store information in the NVRAM.
 */
// link note: we add this function in SDK 2.6 by ourself, should we keep this function?
int hci_control_write_nvram_device_name(int nvram_id, int data_len,void *p_data)
{
    wiced_result_t              result;
    int                         bytes_written;

    bytes_written = wiced_hal_write_nvram( nvram_id, data_len, (uint8_t*)p_data, &result );
    WICED_BT_TRACE( "written :%d bytes id:%d\n\r", bytes_written, nvram_id );

    return data_len;
}

/*
 * Read data from the NVRAM and return in the passed buffer
 */
int app_read_nvram(int nvram_id, void *p_data, int data_len)
{
    uint16_t        read_bytes = 0;
    wiced_result_t  result;

    //if (data_len >= sizeof(wiced_bt_device_link_keys_t))
    //{
        read_bytes = wiced_hal_read_nvram(nvram_id, data_len, p_data, &result);
        WICED_BT_TRACE("NVRAM ID:%d read out of %d bytes:%d result:%d\r\n", nvram_id, data_len, read_bytes, result);
    //}
    return (read_bytes);
}
/*
 * Find NVRAM id from
 */
// link note: we add this function in SDK 2.6 by ourself, should we keep this function?
int hci_hal_control_find_nvram_id( uint8_t *p_data, int len )
{
    int                    nvram_id;
    wiced_result_t         result = WICED_SUCCESS;
    uint8_t               read_data_ptr[BD_ADDR_LEN] = { 0 };

    WICED_BT_TRACE( "%s\r\n", __FUNCTION__ );

    for( nvram_id = SPP_NVRAM_ID ; result == WICED_SUCCESS; nvram_id += 2 )
    {
        wiced_hal_read_nvram( nvram_id, BD_ADDR_LEN, read_data_ptr, &result );

        if( 0 == memcmp( p_data, read_data_ptr, len ) )
        {
            WICED_BT_TRACE( "Find nvram id success:%d\r\n\r", nvram_id );
            for( uint16_t i = 0; i < BD_ADDR_LEN; i++ )
            {
                WICED_BT_TRACE( " %02x", read_data_ptr[i] );
            }
            WICED_BT_TRACE( "\n\r" );
            return nvram_id;
        }
    }
    WICED_BT_TRACE( "Can not find nvram id\n\r" );
    return HCI_CONTROL_INVALID_NVRAM_ID;
}

#if SEND_DATA_ON_INTERRUPT
/*
 * Test function which sends as much data as possible.
 */
void app_send_data(void)
{
    int i;
    wiced_bool_t ret;

    while ((spp_handle != 0) && (app_send_offset != APP_TOTAL_DATA_TO_SEND))
    {
        int bytes_to_send = app_send_offset + SPP_MAX_PAYLOAD < APP_TOTAL_DATA_TO_SEND ? SPP_MAX_PAYLOAD : APP_TOTAL_DATA_TO_SEND - app_send_offset;
        ret = wiced_bt_spp_can_send_more_data(spp_handle);
        if(!ret)
        {
            // buffer_report(" app_send_data can't send");
            // WICED_BT_TRACE(" ! return from wiced_bt_spp_can_send_more_data\n");
            break;
        }
        for (i = 0; i < bytes_to_send; i++)
        {
            app_send_buffer[i] = app_send_offset + i;
        }
        ret = wiced_bt_spp_send_session_data(spp_handle, app_send_buffer, bytes_to_send);
        if(ret != WICED_TRUE)
        {
            // WICED_BT_TRACE(" ! return from wiced_bt_spp_send_session_data\n");
            break;
        }
        app_send_offset += bytes_to_send;
        spp_tx_retry_count = 0;
    }
    // Check if we were able to send everything
    if (app_send_offset < APP_TOTAL_DATA_TO_SEND)
    {
        if(spp_tx_retry_count >= MAX_TX_RETRY)
        {
        WICED_BT_TRACE("Reached max tx retries! Terminating transfer!\r\n");
        WICED_BT_TRACE("Make sure peer device is providing us credits\r\n");
        app_send_offset = 0;
        }
        else
        {
            WICED_BT_TRACE("wiced_start_timer app_tx_timer %d\r\n", app_send_offset);
        wiced_start_timer(&app_tx_timer, TX_RETRY_TIMEOUT);
        spp_tx_retry_count++;
        }
    }
    else
    {
        uint32_t time_tx = clock_SystemTimeMicroseconds64() / 1000 - time_start;
        WICED_BT_TRACE("sent %d in %dmsec (%dKbps)\r\n", APP_TOTAL_DATA_TO_SEND, time_tx, APP_TOTAL_DATA_TO_SEND * 8 / time_tx);
        app_send_offset = 0;
    }
}

/*
 * Test function which start sending data.
 */
void app_interrupt_handler(void *data, uint8_t port_pin)
{
    WICED_BT_TRACE("gpio_interrupt_handler pin:%d send_offset:%d\r\n", port_pin, app_send_offset);
    time_start = clock_SystemTimeMicroseconds64() / 1000;

     /* Get the status of interrupt on P# */
    if (wiced_hal_gpio_get_pin_interrupt_status(BUTTON_GPIO))
    {
        /* Clear the GPIO interrupt */
        wiced_hal_gpio_clear_pin_interrupt_status(BUTTON_GPIO);
    }
    // If we are already sending data, do nothing
    if (app_send_offset != 0)
        return;

    app_send_data();
}

/*
 * The timeout function is periodically called while we are sending big amount of data
 */
void app_tx_ack_timeout(uint32_t param)
{
    app_send_data();
}
#endif


#ifdef HCI_TRACE_OVER_TRANSPORT
/*
 *  Pass protocol traces up over the transport
 */
void app_trace_callback(wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t* p_data)
{
    wiced_transport_send_hci_trace(host_trans_pool, type, length, p_data);
}
#endif

/*
 * Allocate nvram_id to save new NVRAM chunk
 */
// link note: we add this function in SDK 2.6 by ourself, should we keep this function?
int hci_hal_alloc_nvram_id(void)
{
    int                    nvram_id;
    int                    read_bytes;
    wiced_result_t         result;
    wiced_bt_device_link_keys_t    linkkey;

    for( nvram_id = SPP_NVRAM_ID ; read_bytes != 0; nvram_id += 2 )
    {
        read_bytes = wiced_hal_read_nvram( nvram_id, sizeof( wiced_bt_device_link_keys_t ), (uint8_t*)&linkkey, &result );
        if( ( 0 == read_bytes ) || ( result != WICED_SUCCESS ) )
        {
            //WICED_BT_TRACE( "%s nvram_id:%d, readbyte:%d\n\r", __FUNCTION__, nvram_id, read_bytes );
            return nvram_id;
        }
    }
    WICED_BT_TRACE( "%s ERROR, return 0\n\r", __FUNCTION__ );
    return 0;
}

/*link note: add cbk function by our self, should we move to a specific file?*/
void p_remote_name_result_cb(wiced_bt_dev_remote_name_result_t *p_remote_name_result_test)
{
    WICED_BT_TRACE( "%s: Name:%s\n\r ", __FUNCTION__, p_remote_name_result_test->remote_bd_name );
    WICED_BT_TRACE( "Name size = %d\n\r ", sizeof(p_remote_name_result_test->remote_bd_name) );
    memset( paired_device_name, 0, MAX_DEVICE_NAME_LEN );
    if(memcmp(paired_device_name, p_remote_name_result_test->remote_bd_name, MAX_DEVICE_NAME_LEN) == 0)
    {
        WICED_BT_TRACE( "get null point, check retry time =%d \n\r ", remote_name_retry);
        if(remote_name_retry < MAX_REMOTE_NAME_RETRY_TIME)
        {
            WICED_BT_TRACE( "retry get remote device name \n\r ");
            wiced_bt_dev_get_remote_name( paired_device_bd_addr, p_remote_name_result_cb );
            remote_name_retry++;
        }
         else
        {
            WICED_BT_TRACE( "over retry time, transfer default device name \n\r ");
            get_device_name_flag = TRUE;
            memcpy( paired_device_name, default_device_name, strlen(default_device_name) );
            memcpy( &(user_confirm_result_data[BD_ADDR_LEN + sizeof( uint32_t )]), paired_device_name, MAX_DEVICE_NAME_LEN );
            remote_name_retry = 0;
        }
    }
    else
    {
        memcpy( paired_device_name, p_remote_name_result_test->remote_bd_name, MAX_DEVICE_NAME_LEN );
        get_device_name_flag = TRUE;
        remote_name_retry = 0;
        memcpy( &(user_confirm_result_data[BD_ADDR_LEN + sizeof( uint32_t )]), paired_device_name, MAX_DEVICE_NAME_LEN );
    }
}

// link note: we add this function in SDK 2.6 by ourself, should we keep this function?
void spp_store_linkkey_name(void)
{
    int               bytes_written;
    int               nvram_id_linkkey;
    int               nvram_id_name;
    wiced_result_t    result;
    uint8_t           data[1 + BD_ADDR_LEN + MAX_DEVICE_NAME_LEN];

    /* Check if we already have information saved for this bd_addr */
    if( ( nvram_id_linkkey = hci_hal_control_find_nvram_id( saved_link_key.bd_addr, BD_ADDR_LEN ) ) == 0 )
    {
        /* This is the first time, allocate id for the new memory chunk */
        nvram_id_linkkey = hci_hal_alloc_nvram_id();
    }
    bytes_written = hci_control_write_nvram( nvram_id_linkkey, sizeof(wiced_bt_device_link_keys_t), &saved_link_key, 0 );
    WICED_BT_TRACE( "%s link key nvram ID:%d bytes_written:%d\n\r", __FUNCTION__, nvram_id_linkkey, bytes_written );

    WICED_BT_TRACE("linkkey : ");
    for (int i=0 ; i < 16 ; i++ )
    {
        WICED_BT_TRACE("%02x ",saved_link_key.key_data.br_edr_key[i]);
    }
    WICED_BT_TRACE("\n\r");

    if( 0 == bytes_written )
    {
        WICED_BT_TRACE( "linkkey alloc fail, return\n\r" );
        return;
    }

    nvram_id_name = nvram_id_linkkey + 1;

    bytes_written = hci_control_write_nvram_device_name( nvram_id_name, MAX_DEVICE_NAME_LEN, paired_device_name );
    WICED_BT_TRACE( "Name nvram id:%d bytes_written:%d\n\r", nvram_id_name, bytes_written );

    if( 0 == bytes_written )
    {
        //hci_delete_nvram( nvram_id_linkkey );
        hci_control_delete_nvram(nvram_id_linkkey, 0);
        WICED_BT_TRACE( "Name alloc fail, delete address and linkkey\n\r" );
    }

    // Update the pair information
    data[0] = ( nvram_id_linkkey - WICED_NVRAM_VSID_APP ) / 2;  // pair device index
    memcpy( &(data[1]), &(saved_link_key.bd_addr[0]), BD_ADDR_LEN );
    memcpy( &data[1 + BD_ADDR_LEN], &paired_device_name, MAX_DEVICE_NAME_LEN );
    wiced_transport_send_data( HCI_CONTROL_MISC_EVENT_READ_PAIR_INFO, &(data[0]),  1 + BD_ADDR_LEN + MAX_DEVICE_NAME_LEN );
}

/*link note: add api from MCU by our self, should we keep this function?*/
void spp_user_confirm_result(uint8_t result)
{
    WICED_BT_TRACE( "%s result: %d\n\r", __FUNCTION__, result );

    if( USER_CONFIRM_SUCCESS == result )
    {
        wiced_bt_dev_confirm_req_reply( WICED_BT_SUCCESS, paired_device_bd_addr );
    }
    else
    {
        wiced_bt_dev_confirm_req_reply( SMP_CONFIRM_VALUE_ERR, paired_device_bd_addr );
    }
}

/*link note: add by our self but no use, should we keep this function?*/
uint8_t return_spp_connect_status(void)
{
    return spp_connected;
}

void wiced_bt_connection_status_change_cb(wiced_bt_device_address_t bd_addr, uint8_t *p_features, wiced_bool_t is_connected,
                                          uint16_t handle, wiced_bt_transport_t transport, uint8_t reason)
{
    uint8_t connecion_status_data[CONNECTION_STATUS_DATA_LENGTH];
#if 0
    connecion_status_data[0] = is_connected;
    connecion_status_data[1] = reason;
    connecion_status_data[2] = (uint8_t)transport;
    connecion_status_data[3] = (uint8_t)handle;
    connecion_status_data[4] = (uint8_t)( handle >> 8 );
    memcpy( &(connecion_status_data[5]), bd_addr, BD_ADDR_LEN );
    WICED_BT_TRACE( "%s, Addr:%B, feature:%d, connect:%d, Transport:%d Reason:%d Handle:%d\r\n", __FUNCTION__, bd_addr, *p_features, is_connected, transport, reason, handle );

    wiced_transport_send_data( HCI_CONTROL_EVENT_CONNECTION_STATUS, connecion_status_data, CONNECTION_STATUS_DATA_LENGTH );
#else
/*Add by Garmin define event */
    memcpy( &(connecion_status_data[0]), bd_addr, BD_ADDR_LEN);
    connecion_status_data[6] = (uint8_t)transport;
    connecion_status_data[7] = is_connected;
    connecion_status_data[8] = reason;
    wiced_transport_send_data( HCI_CONTROL_EVENT_CONNECTION_STATUS_GARMIN, connecion_status_data, CONNECTION_STATUS_DATA_LENGTH );
#endif
}

#if 1
/*Temp to place garmin_nvm_storage related*/
/*******************************************************************************
**
** Function         bdcmp
**
** Description      Compare bd addr b to a.
**
**
** Returns          Zero if b==a, nonzero otherwise (like memcmp).
**
*******************************************************************************/
void garmin_nvm_read_paired_device_num(void)
{
    int byte_read;
    wiced_result_t              add_key_result;
    WICED_BT_TRACE( "%s \n\r", __func__ );
    /*read paired device list first*/
    byte_read = app_read_nvram(PAIRLIST_NUM_NVRAM_ID, &(paired_list->paired_device_num), sizeof(uint8_t));
    WICED_BT_TRACE( "byte_read:%d, device number = %d \n\r", byte_read, paired_list->paired_device_num );
}

/*******************************************************************************
**
** Function         bdcmp
**
** Description      Compare bd addr b to a.
**
**
** Returns          Zero if b==a, nonzero otherwise (like memcmp).
**
*******************************************************************************/
void garmin_nvm_read_paired_list(void)
{
    int byte_read;
    wiced_result_t              add_key_result;
    WICED_BT_TRACE( "%s \n\r", __func__ );
    wiced_bt_device_link_keys_t test_linkkey;
    /*read paired device info from each device*/
    for(uint8_t i =0; i < paired_list->paired_device_num; i++)
    {
        /* read from NVM*/
        byte_read = app_read_nvram(PAIR_DEVICE1_NVRAM_ID + i, &(paired_list->device[i]), sizeof(BT_paired_device_info_t));
        WICED_BT_TRACE( "byte_read:%d, device id  = %d \n\r", byte_read, i);
        /* save into stack db*/
        add_key_result = wiced_bt_dev_add_device_to_address_resolution_db(&(paired_list->device[i].link_key));
        garmin_nvm_dump_link_key(&(paired_list->device[i].link_key));
        WICED_BT_TRACE( "add_key_result:%d\n\r", add_key_result );
        /* sent to MCU*/
    }
}

/*******************************************************************************
**
** Function         bdcmp
**
** Description      Compare bd addr b to a.
**
**
** Returns          Zero if b==a, nonzero otherwise (like memcmp).
**
*******************************************************************************/
void garmin_nvm_save_paired_list(void)
{
    int byte_read;
    wiced_result_t              add_key_result;
    /*save paired device list first*/
    byte_read = app_write_nvram(PAIRLIST_NUM_NVRAM_ID, sizeof(uint8_t), &(paired_list->paired_device_num));
    WICED_BT_TRACE( "byte_write:%d, device number = %d \n\r", byte_read, paired_list->paired_device_num );
    /*read paired device info from each device*/
    for(uint8_t i =0; i < MAX_PAIRED_DEVICE_NUM; i++)
    {
        /* save from NVM*/
        byte_read = app_write_nvram(PAIR_DEVICE1_NVRAM_ID + i, sizeof(BT_paired_device_info_t), &(paired_list->device[i]));
        WICED_BT_TRACE( "byte_write:%d, device id  = %d \n\r", byte_read, i);
    }
}

/*******************************************************************************
**
** Function         bdcmp
**
** Description      Compare bd addr b to a.
**
**
** Returns          Zero if b==a, nonzero otherwise (like memcmp).
**
*******************************************************************************/
void garmin_nvm_save_paired_device(BT_paired_device_info_t* device)
{
    int byte_read;
    wiced_result_t              add_key_result;
    /*save paired device list first*/
    byte_read = app_write_nvram(PAIRLIST_NUM_NVRAM_ID, sizeof(uint8_t), &(paired_list->paired_device_num));
    WICED_BT_TRACE( "byte_write:%d, device number = %d \n\r", byte_read, paired_list->paired_device_num );
    /*read paired device info from each device*/
    for(uint8_t i =0; i < paired_list->paired_device_num; i++)
    {
        /* save from NVM*/
        byte_read = app_write_nvram(PAIR_DEVICE1_NVRAM_ID + i, sizeof(BT_paired_device_info_t), &(paired_list->device[i]));
        WICED_BT_TRACE( "byte_write:%d, device id  = %d \n\r", byte_read, i);
    }
}

/*******************************************************************************
**
** Function         bdcmp
**
** Description      Compare bd addr b to a.
**
**
** Returns          Zero if b==a, nonzero otherwise (like memcmp).
**
*******************************************************************************/
void garmin_nvm_save_paired_device_by_id(uint8_t* device_id)
{
    int byte_read;
    wiced_result_t              add_key_result;
    /*save paired device list first*/
    byte_read = app_write_nvram(PAIRLIST_NUM_NVRAM_ID, sizeof(uint8_t), &(paired_list->paired_device_num));
    WICED_BT_TRACE( "byte_write:%d, device number = %d \n\r", byte_read, paired_list->paired_device_num );
    /*read paired device info from each device*/
    for(uint8_t i =0; i < paired_list->paired_device_num; i++)
    {
        /* save from NVM*/
        byte_read = app_write_nvram(PAIR_DEVICE1_NVRAM_ID + i, sizeof(BT_paired_device_info_t), &(paired_list->device[i]));
        WICED_BT_TRACE( "byte_write:%d, device id  = %d \n\r", byte_read, i);
    }
}

/*******************************************************************************
**
** Function         bdcmp
**
** Description      Compare bd addr b to a.
**
**
** Returns          Zero if b==a, nonzero otherwise (like memcmp).
**
*******************************************************************************/
void garmin_nvm_dump_pair_list()
{
    BT_paired_device_info_t handle_device;
    WICED_BT_TRACE( "%s \n\r", __func__ );
    /*dump out pair device number*/
    WICED_BT_TRACE( "paired device number = %d \n\r", paired_list->paired_device_num);
    for(uint8_t i = 0;i < paired_list->paired_device_num;i++)
    {
        WICED_BT_TRACE( "==========================\n\r");
        WICED_BT_TRACE( "device ID = %d \n\r", i);
        WICED_BT_TRACE( "device type = %d \n\r", paired_list->device[i].device_type);
        WICED_BT_TRACE( "device name = %s \n\r", &(paired_list->device[i].device_name[0]));
        garmin_nvm_dump_link_key(&(paired_list->device[i].link_key));
#if 0
        wiced_bt_trace_array( "device addr = ", &(paired_list->device[i].link_key.bd_addr[0]), BD_ADDR_LEN);
        WICED_BT_TRACE( "BR/EDR link key type= %d", &(paired_list->device[i].link_key.key_data.br_edr_key[0]));
        wiced_bt_trace_array( "BR/EDR link key = ", &(paired_list->device[i].link_key.key_data.br_edr_key[0]), LINK_KEY_LEN);
        WICED_BT_TRACE( "BLE key mask = %d \r\n", &(paired_list->device[i].link_key.key_data.le_keys_available_mask));
        WICED_BT_TRACE( "BLE addr type = %d \r\n", &(paired_list->device[i].link_key.key_data.ble_addr_type));
        WICED_BT_TRACE( "BLE static addr type= %d \r\n", &(paired_list->device[i].link_key.key_data.static_addr_type));
        wiced_bt_trace_array( "BLE static addr = ", &(paired_list->device[i].link_key.key_data.static_addr[0]), BD_ADDR_LEN);
        wiced_bt_trace_array( "BLE key irk = ", &(paired_list->device[i].link_key.key_data.le_keys.irk[0]), BT_OCTET16_LEN);
        wiced_bt_trace_array( "BLE key pltk = ", &(paired_list->device[i].link_key.key_data.le_keys.pltk[0]), BT_OCTET16_LEN);
        wiced_bt_trace_array( "BLE key pcsrk = ", &(paired_list->device[i].link_key.key_data.le_keys.pcsrk[0]), BT_OCTET16_LEN);
        wiced_bt_trace_array( "BLE key lltk = ", &(paired_list->device[i].link_key.key_data.le_keys.lltk[0]), BT_OCTET16_LEN);
        wiced_bt_trace_array( "BLE key lcsrk = ", &(paired_list->device[i].link_key.key_data.le_keys.lcsrk[0]), BT_OCTET16_LEN);
#endif
        WICED_BT_TRACE( "==========================\n\r");
    }
}

/*******************************************************************************
**
** Function         bdcmp
**
** Description      Compare bd addr b to a.
**
**
** Returns          Zero if b==a, nonzero otherwise (like memcmp).
**
*******************************************************************************/
void garmin_nvm_dump_link_key(wiced_bt_device_link_keys_t* link_key)
{
    wiced_bt_trace_array( "device addr = ", &(link_key->bd_addr[0]), BD_ADDR_LEN);
    WICED_BT_TRACE( "BR/EDR link key type= %d", (link_key->key_data.br_edr_key_type));
    wiced_bt_trace_array( "BR/EDR link key = ", &(link_key->key_data.br_edr_key[0]), LINK_KEY_LEN);
    WICED_BT_TRACE( "BLE key mask = %d \r\n", (link_key->key_data.le_keys_available_mask));
    WICED_BT_TRACE( "BLE addr type = %d \r\n", (link_key->key_data.ble_addr_type));
    WICED_BT_TRACE( "BLE static addr type= %d \r\n", (link_key->key_data.static_addr_type));
    wiced_bt_trace_array( "BLE static addr = ", &(link_key->key_data.static_addr[0]), BD_ADDR_LEN);
    wiced_bt_trace_array( "BLE key irk = ", &(link_key->key_data.le_keys.irk[0]), BT_OCTET16_LEN);
    wiced_bt_trace_array( "BLE key pltk = ", &(link_key->key_data.le_keys.pltk[0]), BT_OCTET16_LEN);
    wiced_bt_trace_array( "BLE key pcsrk = ", &(link_key->key_data.le_keys.pcsrk[0]), BT_OCTET16_LEN);
    wiced_bt_trace_array( "BLE key lltk = ", &(link_key->key_data.le_keys.lltk[0]), BT_OCTET16_LEN);
    wiced_bt_trace_array( "BLE key lcsrk = ", &(link_key->key_data.le_keys.lcsrk[0]), BT_OCTET16_LEN);
}
/*******************************************************************************
**
** Function         bdcmp
**
** Description      Compare bd addr b to a.
**
**
** Returns          Zero if b==a, nonzero otherwise (like memcmp).
**
*******************************************************************************/
uint8_t garmin_nvm_find_device_id(wiced_bt_device_address_t addr)
{
    BT_paired_device_info_t handle_device;
    WICED_BT_TRACE( "%s \n\r", __func__ );
    /*dump out pair device number*/
    WICED_BT_TRACE( "paired device number = %d \n\r", paired_list->paired_device_num);
    for(uint8_t i = 0;i < paired_list->paired_device_num;i++)
    {
        if( 0 == bdcmp(addr, paired_list->device[i].link_key.bd_addr))
        {
           WICED_BT_TRACE( "find devive in paired list, id = %d \n\r", i);
           return i;
        }
    }
    WICED_BT_TRACE( "Can not find devive in paired list\n\r");
    return INVAILD_ID;
}

/*******************************************************************************
**
** Function         bdcmp
**
** Description      Compare bd addr b to a.
**
**
** Returns          Zero if b==a, nonzero otherwise (like memcmp).
**
*******************************************************************************/
void garmin_nvm_add_paired_device(BT_paired_device_info_t* device)
{
    BT_paired_device_info_t handle_device;
    WICED_BT_TRACE( "%s \n\r", __func__ );
    if(paired_list->paired_device_num < MAX_PAIRED_DEVICE_NUM)
    {
        WICED_BT_TRACE( "device num is under max, add \n\r");
        memcpy(&(paired_list->device[paired_list->paired_device_num]), device, sizeof(BT_paired_device_info_t));
        paired_list->paired_device_num++;
    }
    else
    {
        WICED_BT_TRACE( "device num is over max, ignore \n\r");
    }
}

/*******************************************************************************
**
** Function         bdcmp
**
** Description      Compare bd addr b to a.
**
**
** Returns          Zero if b==a, nonzero otherwise (like memcmp).
**
*******************************************************************************/
void garmin_nvm_delete_paired_device(uint8_t id)
{
    int byte_read;
    wiced_result_t              add_key_result;
    /*remove the device with device id*/
    memset(&(paired_list->device[id]), 0, sizeof(BT_paired_device_info_t));
    paired_list->paired_device_num--;
    /*shift the empty position in list*/
    for(uint8_t i =id; i < paired_list->paired_device_num; i++)
    {
        memcpy(&(paired_list->device[id]), &(paired_list->device[id+1]), sizeof(BT_paired_device_info_t));
    }
}
/*******************************************************************************
**
** Function         bdcmp
**
** Description      Compare bd addr b to a.
**
**
** Returns          Zero if b==a, nonzero otherwise (like memcmp).
**
*******************************************************************************/
void garmin_nvm_erase_nvm_storage(void)
{
    wiced_result_t result;
    wiced_hal_delete_nvram( PAIRLIST_NUM_NVRAM_ID, &result );
    WICED_BT_TRACE( "erase NVM pair list num, NVM id= %d, result =%d \n\r", PAIRLIST_NUM_NVRAM_ID, result);
    for(uint8_t i =0; i< MAX_PAIRED_DEVICE_NUM; i++)
    {
        wiced_hal_delete_nvram(PAIR_DEVICE1_NVRAM_ID+i, &result);
        WICED_BT_TRACE( "erase NVM pair device[%d], NVM id=%d, result =%d \n\r", i, PAIR_DEVICE1_NVRAM_ID+i, result);
    }
}
/*******************************************************************************
**
** Function         bdcmp
**
** Description      Compare bd addr b to a.
**
**
** Returns          Zero if b==a, nonzero otherwise (like memcmp).
**
*******************************************************************************/
void garmin_update_pairlist_to_mcu(void)
{
    uint8_t tx_buf[43];
    WICED_BT_TRACE( "%s: total paired device  =%d \n\r", __func__, paired_list->paired_device_num);
    /* Send paired Num */
    for(uint8_t i =0;i < paired_list->paired_device_num; i++)
    {
        //Byte[0]: (1-byte) Number of total paired devices
        tx_buf[0] = paired_list->paired_device_num;
        //Byte[1]: (1-byte) Number of current device
        tx_buf[1] = i;
        //Byte[2]: (6-bytes) Bluetooth device address
        memcpy(&tx_buf[2], &(paired_list->device[i].link_key.bd_addr[0]), BD_ADDR_LEN);
        //Byte[8]: (32-bytes) Bluetooth device name
        memcpy(&tx_buf[8], &(paired_list->device[i].device_name[0]), 32);
        //Byte[40]: (1-byte) Device type specified in BT_device_type_e
        tx_buf[40] = paired_list->device[i].device_type;
        //Byte[41]: (1-byte) Whether or not the device has lost its authentication
        tx_buf[41] = 0;     //TBD
        //Byte[42]: (1-byte) Whether or not the device supports iAP connection
        tx_buf[42] = 0;     //TBD
        wiced_transport_send_data(HCI_CONTROL_EVENT_PAIRED_DEVICE_LIST_GARMIN, &(tx_buf[0]), 43);
        WICED_BT_TRACE( "send paired device %d to MCU, total NUM=%d: \n\r", i, paired_list->paired_device_num);
    }
}
/*******************************************************************************
**
** Function         bdcmp
**
** Description      Compare bd addr b to a.
**
**
** Returns          Zero if b==a, nonzero otherwise (like memcmp).
**
*******************************************************************************/
int bdcmp(const BD_ADDR a, const BD_ADDR b)
{
    int i;

    for (i = BD_ADDR_LEN; i != 0; i--)
    {
        if (*a++ != *b++)
        {
            return -1;
        }
    }
    return 0;
}
#endif
