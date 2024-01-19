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
 *  watch.c
 *
 *  Watch Sample Application for 20XXX devices.
 *
 *  This app demonstrates Bluetooth A2DP source, AVRCP Controller/Target, Apple Media Service (AMS),
 *  Apple Notification Center Service (ANCS), and HFP Audio Gateway/Handsfree Unit.
 *  Features demonstrated
 *   - WICED BT A2DP Source APIs
 *   - WICED BT AVRCP (Controller/Target) APIs
 *   - WICED BT GATT APIs
 *   - Apple Media Service and Apple Notification Client Services (AMS and ANCS)
 *   - Handling of the UART WICED protocol
 *   - SDP and GATT descriptor/attribute configuration
 *   - HFP Audio Gateway role
 *   - HFP Handsfree Unit role
 *
 *  Instructions
 *  ------------
 *  To demonstrate the app, follow these steps -
 *
 *  1. Build and download the application to the WICED board.
 *  2. By default sleep in enabled in 20706A2, 20819A1, 20719B1, 20721B1 chips (*refer : Note).
 *  3. Open the BT/BLE Profile Client Control application and open the port for WICED HCI for the device.
 *     Default baud rate configured in the application is 3M.
 *  4. Use Client Control application to send various commands mentioned below.
 *  5. Run the BTSpy program to view protocol and application traces.
 *
 *  Note :
 *   For 20706A2 we are allowing normal PMU sleep.Therefore,the transport will be connected by default.So,no need to wake device.
 *   In other chips 207xx, 208xx please wake the device using configured wake pin. (Check  hci_control_sleep_config.device_wake_gpio_num).
 *
 *  See "BT/BLE Profile Client Control" and "BT Spy" in chip-specifc readme.txt for more information about these apps.
 *
 *  BR/EDR Audio Source and AVRC Target:
 *  - The Watch app can demonstrate how use to BR/EDR Audio Source and AVRC TG profiles.
 *  - Use buttons in AV Source tab.
 *  - To play sine wave sample, set the audio frequency to desired value (48kHz, 44.1kHz, etc.)
 *    and select the Media type as 'Sine Wave' in UI. In this case, built-in sine wave audio is played.
 *  - To play music from .wav file, select the Media type as File, browse and select a .wav file.
 *    In this case, audio for .wav file is routed over WICED HCI UART to the WICED board.
 *  - Put an audio sink device such as BT headphone/speaker in pairable mode.
 *  - Click on "Start" button for "BR/EDR Discovery" combo box to find the audio sink device.
 *  - Select the peer device in the BR/EDR Discovery combo box.
 *  - Click "Connect" button under AV Source tab.
 *  - Click "Start Streaming" button. Music will start playing on peer device.
 *  - The watch app uses AVRCP Target role. Once connected to headset/speaker,
 *    the app can send notifications for play status change (Play, Pause, Stop) and
 *    setting change (Repeat, Shuffle) to peer AVRCP controller (such as headset/speaker).
 *    Note: the songs shown in the AVRC TG UI and some settings such Repeat/Shuffle are for testing
 *    AVRC commands only, do not indicate the actual media played and will not change the media played.
 *
 *  BR/EDR AVRCP Controller:
 *  - The Watch app can demonstrate how use to AVRC CT profile.
 *  - Disconnect all devices if any connected.
 *  - Make an audio source device such as iPhone discoverable/pairable from Bluetooth Settings UI on phone.
 *  - Using "BR/EDR Discovery" "Start" button, search and select the device.
 *  - Use buttons in AVRC CT tab. Click Connect button and accept pairing.
 *  - Play music on audio source device and control the music via buttons in AVRC CT tab.
 *  - In Controller mode, pass-thru commands are executed via Play, Pause, Stop, etc. buttons.
 *  - Absolute volume change can be done via the drop down Volume or Vol Up/Down buttons.
 *  - Note that iPhone does does not support Vol buttons.
 *  - Note that music will continue to play on audio source device.
 *
 *  iOS ANCS and AMS GATT Services:
 *  - The Watch app can demonstrate how to use AMS and ANCS iOS services as below.
 *  - Disconnect all devices if any connected.
 *  - Select Pairable if it not checked.
 *  - Click on the "Start Adverts" button in GATT tab.
 *  - From the iPhone app such as 'LightBlue', find and connect to 'Watch' app.
 *  - Allow pairing with the iPhone.
 *  - AMS:
 *    - Play media on the iPhone.
 *    - Use buttons in AVRC CT tab to control the music.
 *    - Note that music will continue to play on iPhone.
 *  - ANCS:
 *    - Incoming calls and messages to the iPhone will be displayed on the ANCS buttons.
 *    - Make an incoming call to your iPhone. See notification displayed on UI to accept
 *      or reject the call. Send SMS to your iPhone to see notification. Similarly missed
 *      call notifications are seen.
 *
 *  BLE Client:
 *  - The Watch app can demonstrate BLE Client functionality as below.
 *  - Make sure there is a BT device with GATT services that is advertising. For example use app
 *    such as 'LightBlue' on your phone and create a 'Virtual Peripheral' such as 'Blood Pressure'
 *  - To find GATT devices: Click on the "Start" button for "BLE Discovery" combo box.
 *    Click on "Stop" button to end discovery.
 *  - To connect BLE device: Choose device from the "BLE Discovery" drop down combo box and
 *    click "Connect" button.
 *  - To discover services: Click on the "Discover Services" button
 *  - To discover characteristics: Enter the handles in the edit box and click
 *    on "Discover Characteristics"
 *  - To discover descriptors: Enter the handles in the edit box and click on
 *    "Discover Characteristics"
 *  - Enter the Handle and Hex Value to write to the remote device using buttons
 *     "Write" : Write hex value to remote handle
 *     "Write no rsp" : Write hex value without response to remote handle
 *     "Value Notify" : Write notification value to the remote handle
 *     "Value Indicate" : Write indication value to the remote handle
 *
 *  HFP Audio Gateway:
 *  - These targets support HFP Audio Gateway:
 *    CYW920721B2EVK-02, CYW920721M2EVK-01, CYW920721M2EVK-02, CYW9M2BASE-43012BT and CYW943012BTEVK-01
 *  - Build with "HFP_AG_INCLUDED=1" to enable AG. (disable Handsfree Unit simultaneously)
 *  - The Watch app can demonstrate how to use HFP AG as below.
 *  - Make a HFP Headset (headphone or earbuds) discoverable and pairable by its specific behavior.
 *  - In ClientControl, click on "Start" button for "BR/EDR Discovery" combo box to find the Headset device.
 *  - Select the peer device in the BR/EDR Discovery combo box.
 *  - Click "Connect" button under AG tab.
 *  - Click "Audio Connect" button. AG will create a SCO connection to Headset, wide-band speech supported.
 *  - Click "Audio Disconnect" button to remove SCO connection.

 *  HFP Handsfree Unit:
 *  - These targets support HFP Hands-free Unit by default:
 *    CYW920721B2EVK-02, CYW920721M2EVK-01, CYW920721M2EVK-02, CYW9M2BASE-43012BT and CYW943012BTEVK-01
 *  - To create handsfree connection with remote Audio Gateway (AG) device (such as mobile phone), using
 *    ClientControl and choose the Bluetooth address of the remote AG device from the BR/EDR combo box.
 *    Click "Connect" button under HF tab.
 *  - OR Put the device in discoverable and connectable mode and search for the device from AG device and connect
 *  - The following HF operations can be performed using the ClientControl HF tab
 *       Connect / Disconnect HF or SCO connection
 *       Answer / Hang-up the call
 *       Dial / Redial the number
 *       Control Held call (ex. hold call, release all held calls, etc.)
 *       Mic / Speaker gain control
 *
 *  SPI Transport:
 *  - Update the functionality of WICED_P06 from WICED_GPIO to WICED_SPI_1_SLAVE_READY in
 *    platform_gpio_pins[] to get SPI transport working for 20719B2
 */

/*******************************************************************************
 *                               Includes
 ******************************************************************************/
#include <string.h>
#if defined(WICED_APP_LE_SLAVE_CLIENT_INCLUDED) && defined(WICED_APP_ANCS_INCLUDED)
#include <wiced_bt_ams.h>
#include <wiced_bt_ancs.h>
#endif
#if 0	//unuse
#include <wiced_bt_avrc.h>
#include <wiced_bt_avrc_defs.h>
#include <wiced_bt_avrc_tg.h>
#endif
#include <wiced_bt_ble.h>
#include <wiced_bt_cfg.h>
#include <wiced_bt_gatt.h>
#include <wiced_bt_sdp.h>
#include <wiced_hal_puart.h>
#include <wiced_hal_gpio.h>
#ifdef CYW43012C0
#include <wiced_hal_watchdog.h>
#else
#include <wiced_hal_wdog.h>
#endif
#include <wiced_memory.h>
#include <wiced_platform.h>
#include <wiced_timer.h>
#include <wiced_transport.h>
#include <wiced_bt_stack.h>
#ifdef CYW9BT_AUDIO
#include <wiced_audio_manager.h>
#endif
#if ( defined(CYW20706A2) || defined(CYW20719B1) || defined(CYW20721B1) || defined(CYW43012C0) )
#include <wiced_bt_app_common.h>
#endif
#ifdef CYW20719B1
#include <wiced_bt_event.h>
#endif
#ifdef CYW20706A2
#if (WICED_HCI_TRANSPORT == WICED_HCI_TRANSPORT_SPI)
#include <wiced_trans_spi.h>
#endif
#endif
#if ( defined(CYW20719B1) || defined(CYW20719B2) || defined(CYW20721B1) || defined(CYW20721B2) || defined(WICEDX) || defined(CYW20819A1) )
#include <wiced_sleep.h>
#endif
#ifdef CYW20706A2
#include <wiced_power_save.h>
#if (WICED_HCI_TRANSPORT == WICED_HCI_TRANSPORT_SPI)
#include <wiced_hal_pspi.h>
#endif
#endif
#include "hci_control.h"
#if 0	//unuse
#include "hci_control_audio.h"
#include "hci_control_rc_controller.h"
#include "hci_control_rc_target.h"
#include "hci_control_test.h"
#endif
#include "hci_control_le.h"
#ifdef WICED_APP_HFP_AG_INCLUDED
#include "hci_control_hfp_ag.h"
#include <wiced_bt_hfp_ag.h>
#endif
#ifdef WICED_APP_HFP_HF_INCLUDED
#include "hci_control_hfp_hf.h"
#include <wiced_bt_hfp_hf_int.h>
#endif
#if 0	//unuse
#include "wiced_app.h"
#include "wiced_app_cfg.h"
#endif
#ifdef AUTO_ELNA_SWITCH
#include "cycfg_pins.h"
#include "wiced_hal_rfm.h"
#ifndef TX_PU
#define TX_PU   CTX
#endif
#ifndef RX_PU
#define RX_PU   CRX
#endif
#endif

#include "wiced_app.h"
#include "wiced_hal_nvram.h"
/*****************************************************************************
**  Constants
*****************************************************************************/
#define WICED_HS_EIR_BUF_MAX_SIZE               264
#define KEY_INFO_POOL_BUFFER_SIZE   145 //Size of the buffer used for holding the peer device key info
#define KEY_INFO_POOL_BUFFER_COUNT  5  //Correspond's to the number of peer devices modify by link:origin value is 5

#define SECI_BAUD_RATE    2000000  // Applicable for 20719B1 and 20721B1 when Coex is used

#ifdef CYW20819A1
#define WICED_TRANSPORT_BUFFER_COUNT    1
#else
#define WICED_TRANSPORT_BUFFER_COUNT    2
#endif

/*****************************************************************************
**  Structures
*****************************************************************************/
typedef struct
{
    void    *p_next;
    uint16_t nvram_id;
    uint8_t  chunk_len;
    uint8_t  data[1];
} hci_control_nvram_chunk_t;

/******************************************************
 *               Variables Definitions
 ******************************************************/
uint8_t avrcp_profile_role = AVRCP_TARGET_ROLE;
#if defined(WICED_APP_HFP_AG_INCLUDED)
uint8_t hfp_profile_role = HFP_AUDIO_GATEWAY_ROLE;
#elif defined(WICED_APP_HFP_HF_INCLUDED)
uint8_t hfp_profile_role = HFP_HANDSFREE_UNIT_ROLE;
#endif

hci_control_nvram_chunk_t *p_nvram_first = NULL;

/* HS control block */
#if BTA_DYNAMIC_MEMORY == WICED_FALSE
hci_control_cb_t  hci_control_cb;
#endif

wiced_transport_buffer_pool_t* transport_pool;   // Trans pool for sending the RFCOMM data to host
#if BTSTACK_VER >= 0x01020000
wiced_bt_pool_t                *p_key_info_pool;  //Pool for storing the  key info
#else
wiced_bt_buffer_pool_t*        p_key_info_pool;  //Pool for storing the  key info
#endif

#if BTSTACK_VER >= 0x01020000
extern wiced_bt_heap_t *p_default_heap;
#endif

// memory optimizations
#if defined(CYW43012C0)
uint8_t g_wiced_memory_pre_init_enable = 2;     // 2: MEMORY_PRE_INIT_ENABLE_A2DP_SOURCE
uint8_t g_wiced_memory_pre_init_max_ble_connections = 4;
uint8_t g_wiced_memory_pre_init_num_ble_rl = 16;
#endif

extern const wiced_bt_cfg_settings_t wiced_bt_cfg_settings;

/******************************************************
 *               Function Declarations
 ******************************************************/
static void hci_control_transport_status( wiced_transport_type_t type );
static uint32_t hci_control_proc_rx_cmd( uint8_t *p_data, uint32_t length );
#ifdef NEW_DYNAMIC_MEMORY_INCLUDED
static void hci_control_transport_tx_cplt_cback(void);
#else
static void hci_control_transport_tx_cplt_cback(wiced_transport_buffer_pool_t* p_pool);
#endif
static void hci_control_handle_reset_cmd( void );
static void hci_control_handle_trace_enable( uint8_t *p_data );
static void hci_control_device_handle_command( uint16_t cmd_opcode, uint8_t* p_data, uint32_t data_len );
static void hci_control_handle_set_local_bda( uint8_t *p_bda );
static void hci_control_inquiry( uint8_t enable );
static void hci_control_handle_set_visibility( uint8_t discoverability, uint8_t connectability );
static void hci_control_handle_set_pairability ( uint8_t pairing_allowed );
#if (defined(COEX_SUPPORTED) && COEX_SUPPORTED == WICED_TRUE)
static void hci_control_handle_enable_disable_coex ( wiced_bool_t enable );
#endif
static void hci_control_handle_read_local_bda( void );
static void hci_control_handle_user_confirmation( uint8_t *p_bda, uint8_t accept_pairing );
//static void hci_control_handle_read_buffer_stats( void );  //modify by link to check buf
static void hci_control_send_device_started_evt( void );
extern wiced_result_t wiced_bt_avrc_ct_cleanup( void );
#if (defined(SLEEP_SUPPORTED) && (SLEEP_SUPPORTED == WICED_TRUE))
static void hci_control_sleep_configure(void);
#endif
extern void hci_control_handle_read_buffer_stats( void );     //modify by link to check buf
void spp_handle_command(uint16_t cmd_opcode, uint8_t* p, uint32_t data_len);
void hci_iap2_handle_command(uint16_t cmd_opcode, uint8_t* p_data, uint32_t data_len);

/******************************************************************************
 *                                Variable/Structure/type Definitions
 ******************************************************************************/

#if ( !defined(CYW43012C0) && (WICED_HCI_TRANSPORT == WICED_HCI_TRANSPORT_SPI))

#ifndef CYW20706A2
#ifdef CYW20819A1
#define SPI_GPIO_CFG    SPI_PIN_CONFIG(WICED_P09, WICED_P15, WICED_P06, WICED_P17)
#else
#define SLAVE1_P01_CS_P10_CLK_P28_MOSI_P29_MISO     0x010A1C1D
#define SPI_GPIO_CFG    SLAVE1_P01_CS_P10_CLK_P28_MOSI_P29_MISO
#endif
#endif
const wiced_transport_cfg_t transport_cfg =
{
    .type = WICED_TRANSPORT_SPI,
    .cfg =
    {
        .spi_cfg =
        {
#ifdef CYW20706A2
            .dev_role            = SPI_SLAVE_ROLE,
            .spi_gpio_cfg        = SLAVE1_P02_CS_P03_CLK_P00_MOSI_P25_MISO, /**< Pins to use for the data and clk lines. Refer  spiffdriver.h for details */
            .spi_pin_pull_config = INPUT_PIN_FLOATING,
#elif (CYW20719B2 || CYW20721B2)
#else
            .dev_role            = SPI_SLAVE,
            .spi_gpio_cfg        = SPI_GPIO_CFG, /**< Pins to use for the data and clk lines. Refer  spiffdriver.h for details */
            .spi_pin_pull_config = INPUT_PIN_PULL_DOWN,
#endif
            .clock_speed         = 0,
            .endian              = SPI_MSB_FIRST,
            .polarity            = SPI_SS_ACTIVE_LOW,
            .mode                = SPI_MODE_0,
#ifdef CYW20706A2
            .cs_pin              =  0,
            .slave_ready_pin     =  WICED_P15
#elif (CYW20719B2 || CYW20721B2)
#else
            .cs_pin              =  0,
            .slave_ready_pin     =  WICED_P06
#endif
        },
    },
    .rx_buff_pool_cfg =
    {
        .buffer_size  = TRANS_SPI_BUFFER_SIZE,
        .buffer_count = WICED_TRANSPORT_BUFFER_COUNT
    },
    .p_status_handler    = hci_control_transport_status,
    .p_data_handler      = hci_control_proc_rx_cmd,
    .p_tx_complete_cback = hci_control_transport_tx_cplt_cback
};

#else

const wiced_transport_cfg_t transport_cfg =
{
    .type = WICED_TRANSPORT_UART,
    .cfg =
    {
        .uart_cfg =
        {
            .mode = WICED_TRANSPORT_UART_HCI_MODE,
#if 1
			.baud_rate =  3000000//115200
#else
            .baud_rate =  HCI_UART_DEFAULT_BAUD
#endif
        },
    },
#ifdef NEW_DYNAMIC_MEMORY_INCLUDED
    .heap_config =
    {
        .data_heap_size = 1024 * 4 + 1500 * 2,
        .hci_trace_heap_size = 1024 * 2,
        .debug_trace_heap_size = 1024,
    },
#else
    .rx_buff_pool_cfg =
    {
        .buffer_size  = TRANS_UART_BUFFER_SIZE,
        .buffer_count = WICED_TRANSPORT_BUFFER_COUNT
    },
#endif
    .p_status_handler    = hci_control_transport_status,
    .p_data_handler      = hci_control_proc_rx_cmd,
    .p_tx_complete_cback = hci_control_transport_tx_cplt_cback
};

#endif

/******************************************************
 *               Function Definitions
 ******************************************************/

void hci_control_init(void)
{
    memset(&hci_control_cb, 0, sizeof(hci_control_cb));

    wiced_transport_init(&transport_cfg);
}

void hci_control_post_init(void)
{
#if 1
    /* Allow peer to pair */
    hci_control_cb.pairing_allowed = WICED_TRUE;
#endif

#ifdef WICED_APP_LE_INCLUDED
    hci_control_le_enable(&wiced_bt_cfg_settings);
#endif

#ifdef WICED_APP_AUDIO_SRC_INCLUDED
    av_app_init();
#endif
#ifdef WICED_APP_AUDIO_RC_CT_INCLUDED
    /* Initialize AVRC Target and Controler */
    if (avrcp_profile_role == AVRCP_CONTROLLER_ROLE)
    {
        hci_control_rc_controller_init();
    }
#endif
#ifdef WICED_APP_AUDIO_RC_TG_INCLUDED
    if (avrcp_profile_role == AVRCP_TARGET_ROLE)
    {
        hci_control_rc_target_init();
        wiced_bt_avrc_tg_register();
    }
#endif
#if (defined(WICED_APP_HFP_AG_INCLUDED) || defined(WICED_APP_HFP_HF_INCLUDED))
    /* Perform the rfcomm init before hfp start up */
    if( (wiced_bt_rfcomm_result_t)wiced_bt_rfcomm_init( 200, 5 ) != WICED_BT_RFCOMM_SUCCESS )
    {
        WICED_BT_TRACE("Error Initializing RFCOMM - HFP failed\r\n");
        return;
    }
#endif
#ifdef WICED_APP_HFP_AG_INCLUDED
    hci_control_ag_init();
#endif
#ifdef WICED_APP_HFP_HF_INCLUDED
    hci_control_hf_init();
#endif

    // Disable while streaming audio over the uart.
    //wiced_bt_dev_register_hci_trace(hci_control_hci_packet_cback);

    // Creating a buffer pool for holding the peer devices's key info
#if BTSTACK_VER >= 0x01020000
    p_key_info_pool = wiced_bt_create_pool("pki", KEY_INFO_POOL_BUFFER_SIZE,
            KEY_INFO_POOL_BUFFER_COUNT, NULL);
#else
    p_key_info_pool = wiced_bt_create_pool( KEY_INFO_POOL_BUFFER_SIZE, KEY_INFO_POOL_BUFFER_COUNT );
#endif
    if (p_key_info_pool == NULL)
        WICED_BT_TRACE("Err: wiced_bt_create_pool failed\r\n");

#if (defined(SLEEP_SUPPORTED) && (SLEEP_SUPPORTED == WICED_TRUE))
    hci_control_sleep_configure();
#endif

#if (defined(COEX_SUPPORTED) && COEX_SUPPORTED == WICED_TRUE)
#if defined(CYW20719B1) || defined(CYW20719B2) || defined(CYW20721B1) || defined(CYW20721B2) || defined(CYW20819A1)
    wiced_bt_coex_enable(SECI_BAUD_RATE);
#else
    wiced_bt_coex_enable();
#endif
#endif

#ifdef CYW20706A2
    // Tell Host that App is started
    hci_control_send_device_started_evt();
#endif

#ifdef AUTO_ELNA_SWITCH
    wiced_hal_rfm_auto_elna_switch(1, TX_PU, RX_PU);
#endif
}

#if (defined(SLEEP_SUPPORTED) && (SLEEP_SUPPORTED == WICED_TRUE))

#ifndef CYW20706A2
wiced_timer_t hci_control_app_timer;

void hci_control_timeout( uint32_t count )
{
    WICED_BT_TRACE("Idle Timeout.\r\n");

    hci_control_cb.application_state = HCI_CONTROL_STATE_IDLE;
}

/*
* The function invoked on timeout of app seconds timer.
*/
void hci_control_init_timer(void)
{
    /* Start idle timer to enter to sleep */
    if ( wiced_init_timer( &hci_control_app_timer, hci_control_timeout, 0, WICED_SECONDS_TIMER ) == WICED_SUCCESS )
    {
        if ( wiced_start_timer( &hci_control_app_timer, 10 ) !=  WICED_SUCCESS )
        {
            WICED_BT_TRACE("idle timer start failure\r\n");
            return;
        }
    }
    else
    {
        WICED_BT_TRACE("idle timer init fail \r\n");
    }
}

/*
* The function handles sleep.
*/
static uint32_t hci_control_sleep_hanlder(wiced_sleep_poll_type_t type )
{
    uint32_t ret = WICED_SLEEP_NOT_ALLOWED;

    switch(type)
    {
        case WICED_SLEEP_POLL_SLEEP_PERMISSION:
            if( hci_control_cb.application_state == HCI_CONTROL_STATE_IDLE )
            {
#ifdef ENABLE_SDS_SLEEP
                ret = WICED_SLEEP_ALLOWED_WITH_SHUTDOWN;
#else
                ret = WICED_SLEEP_ALLOWED_WITHOUT_SHUTDOWN;
#endif
            }
            break;

        case WICED_SLEEP_POLL_TIME_TO_SLEEP:
            if( hci_control_cb.application_state == HCI_CONTROL_STATE_NOT_IDLE )
            {
                ret = 0;
            }
            else
            {
                ret = WICED_SLEEP_MAX_TIME_TO_SLEEP;
            }
            break;
    }
    return ret;
}
#endif // !CYW20706A2

/*
* The function does sleep configuration based on platform.
*/
void hci_control_sleep_configure()
{
    WICED_BT_TRACE("[%s] \r\n",__FUNCTION__);
#ifdef CYW20706A2
    {
        wiced_ptu_clock_bits_t ptu_clock_bits;
        wiced_ptu_aux_clock_bits_t ptu_aux_clock_bits;
        ptu_clock_bits     = (  GATE_ON_SDIO_CLK_BIT |
                                PCM_CLK_EN_BIT  |
                                GATE_ON_SDIO_HOST_CLK_BIT
                             );
        ptu_aux_clock_bits = (  PCM_DEV_CLK_EN_BIT |
                                ABURST_CLK_EN_BIT  |
                                SPIFFY2_TPORT_CLK_EN_BIT |
                                SPIFFY2_HCLK_CLK_EN_BIT
                             );
#if (WICED_HCI_TRANSPORT == WICED_HCI_TRANSPORT_SPI)
        //SPI Sleep Init
        //P2 is the SPI CS and the device should wake when CS is asserted.
        //I2S_CLK/PCM_CLK/P2/P37/P28 available on the same pad B7 on the 20706-P49 package.
        //So configuring I2S_CLK(BT_GPIO_6) as the DEV_WAKE.
        //SPI CS is configured as active low, so DEV_WAKE should be active low
        wiced_trans_spi_sleep_config( WICED_GPIO_06, WICED_WAKE_GPIO_ACTIVE_LOW );
#else
        wiced_sleep_config( WICED_TRUE, WICED_WAKE_GPIO_ACTIVE_LOW, WICED_WAKE_GPIO_ACTIVE_HIGH );
#endif
        wiced_ptu_clock_disable(ptu_clock_bits, ptu_aux_clock_bits);
    }
#else // !20703

    {

        /*sleep configuration*/
        static wiced_sleep_config_t    hci_control_sleep_config;
        // For UART mode MCU should control wake gpio and configure the sleep
        hci_control_sleep_config.sleep_mode             = WICED_SLEEP_MODE_TRANSPORT;
        hci_control_sleep_config.device_wake_gpio_num   = WICED_GPIO_PIN_BUTTON;
        hci_control_sleep_config.device_wake_mode       = WICED_SLEEP_WAKE_ACTIVE_LOW;
        hci_control_sleep_config.device_wake_source     = WICED_SLEEP_WAKE_SOURCE_GPIO;
        hci_control_sleep_config.host_wake_mode         = WICED_SLEEP_WAKE_ACTIVE_HIGH;
        hci_control_sleep_config.sleep_permit_handler   = hci_control_sleep_hanlder;

        // Init Idle timer
        hci_control_init_timer();

        if( wiced_sleep_get_boot_mode() == WICED_SLEEP_COLD_BOOT )
        {
            WICED_BT_TRACE( "Watch Cold Start \r\n" );
        }
        else
        {
            // If warm start, retrieve data from retention ram if needed
            WICED_BT_TRACE( "Watch Warm Start \r\n" );
        }
        hci_control_cb.application_state = HCI_CONTROL_STATE_NOT_IDLE;

        wiced_sleep_configure( &hci_control_sleep_config );
    }
#endif // CYW20706A2

}
#endif // end (defined(SLEEP_SUPPORTED) && (SLEEP_SUPPORTED == WICED_TRUE))
/*
 *  Prepare extended inquiry response data.  Current version publishes headset,
 *  handsfree and generic audio services.
 */
void hci_control_write_eir( void )
{
    uint8_t *pBuf;
    uint8_t *p, *p_tmp;
    uint8_t nb_uuid = 0;
    uint8_t length;

    //Allocating a buffer from the public pool
    pBuf = (uint8_t *)wiced_bt_get_buffer( WICED_HS_EIR_BUF_MAX_SIZE );

    WICED_BT_TRACE( "hci_control_write_eir %x\r\n", pBuf );

    if ( !pBuf )
    {
        return;
    }

    p = pBuf;

    length = strlen( (char *)wiced_bt_cfg_settings.device_name );
    UINT8_TO_STREAM(p, length + 1);
    UINT8_TO_STREAM(p, BT_EIR_COMPLETE_LOCAL_NAME_TYPE);
    memcpy( p, wiced_bt_cfg_settings.device_name, length );
    p += length;

    // Add other BR/EDR UUIDs
    p_tmp = p;      // We don't now the number of UUIDs for the moment
    p++;
    UINT8_TO_STREAM(p, BT_EIR_COMPLETE_16BITS_UUID_TYPE);
#ifdef WICED_APP_AUDIO_SRC_INCLUDED
    UINT16_TO_STREAM(p, UUID_SERVCLASS_AUDIO_SOURCE);       nb_uuid++;
#endif
#ifdef WICED_APP_AUDIO_RC_TG_INCLUDED
    UINT16_TO_STREAM(p, UUID_SERVCLASS_AV_REM_CTRL_TARGET); nb_uuid++;
#endif
#ifdef WICED_APP_AUDIO_RC_CT_INCLUDED
    UINT16_TO_STREAM(p, UUID_SERVCLASS_AV_REMOTE_CONTROL);  nb_uuid++;
    UINT16_TO_STREAM(p, UUID_SERVCLASS_AUDIO_SINK);         nb_uuid++;
#endif
#if (defined(WICED_APP_HFP_AG_INCLUDED) || defined(WICED_APP_HFP_HF_INCLUDED))
    UINT16_TO_STREAM(p, UUID_SERVCLASS_HEADSET);            nb_uuid++;
    UINT16_TO_STREAM(p, UUID_SERVCLASS_HF_HANDSFREE);       nb_uuid++;
    UINT16_TO_STREAM(p, UUID_SERVCLASS_GENERIC_AUDIO);      nb_uuid++;
#endif

    /* Now, we can update the UUID Tag's length */
    UINT8_TO_STREAM(p_tmp, (nb_uuid * LEN_UUID_16) + 1);

    // Last Tag
    UINT8_TO_STREAM(p, 0x00);

    // print EIR data
    wiced_bt_trace_array( "EIR :", ( uint8_t* )( pBuf ), MIN( p - ( uint8_t* )pBuf, 100 ) );
    wiced_bt_dev_write_eir( pBuf, (uint16_t)(p - pBuf) );

    /* Allocated buffer not anymore needed. Free it */
    wiced_bt_free_buffer( pBuf );

    return;
}

/*
 *  Process all HCI packet from
 */
#define COD_MAJOR_COMPUTER  1
#define COD_MAJOR_PHONE     2

void hci_control_hci_packet_cback( wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t* p_data )
{
#if (WICED_HCI_TRANSPORT == WICED_HCI_TRANSPORT_UART)
    // send the trace
#ifdef NEW_DYNAMIC_MEMORY_INCLUDED
    wiced_transport_send_hci_trace(type, p_data, length);
#else
    wiced_transport_send_hci_trace( NULL, type, length, p_data  );
#endif
#endif

#ifdef _HCI_CONTROL_TEST_H_
    if ( !test_command.test_executing )
        return;

    // If executing test command, need to send Command Complete event back to host app
    if( ( type == HCI_TRACE_EVENT ) && ( length >= 6 ) )
    {
        hci_control_handle_hci_test_event( p_data, length );
    }
#endif
}

/*
 * Process HCI commands from the MCU
 */
#ifdef __AMS_API_H
static void hci_control_ams_handle_command(uint16_t opcode, uint8_t *p_data, uint16_t payload_len)
{
    uint8_t status = HCI_CONTROL_STATUS_SUCCESS;
    uint8_t ams_remote_command_id;

    WICED_BT_TRACE("%s cmd_opcode 0x%02x\r\n", __func__ ,opcode);

    switch (opcode)
    {
    case HCI_CONTROL_AVRC_CONTROLLER_COMMAND_PLAY:
        ams_remote_command_id = AMS_REMOTE_COMMAND_ID_PLAY;
        break;
    case HCI_CONTROL_AVRC_CONTROLLER_COMMAND_PAUSE:
        ams_remote_command_id = AMS_REMOTE_COMMAND_ID_PAUSE;
        break;
    case HCI_CONTROL_AVRC_CONTROLLER_COMMAND_NEXT_TRACK:
        ams_remote_command_id = AMS_REMOTE_COMMAND_ID_NEXT_TRACK;
        break;
    case HCI_CONTROL_AVRC_CONTROLLER_COMMAND_PREVIOUS_TRACK:
        ams_remote_command_id = AMS_REMOTE_COMMAND_ID_PREVIOUS_TRACK;
        break;
    case HCI_CONTROL_AVRC_CONTROLLER_COMMAND_VOLUME_UP:
        ams_remote_command_id = AMS_REMOTE_COMMAND_ID_VOLUME_UP;
        break;
    case HCI_CONTROL_AVRC_CONTROLLER_COMMAND_VOLUME_DOWN:
        ams_remote_command_id = AMS_REMOTE_COMMAND_ID_VOLUME_DOWN;
        break;
    case HCI_CONTROL_AVRC_CONTROLLER_COMMAND_SET_REPEAT_MODE:
        ams_remote_command_id = AMS_REMOTE_COMMAND_ID_ADVANCED_REPEAT_MODE;
        break;
    case HCI_CONTROL_AVRC_CONTROLLER_COMMAND_SET_SHUFFLE_MODE:
        ams_remote_command_id = AMS_REMOTE_COMMAND_ID_ADVANCED_SHUFFLE_MODE;
        break;
    case HCI_CONTROL_AVRC_CONTROLLER_COMMAND_BEGIN_FAST_FORWARD:
        ams_remote_command_id = AMS_REMOTE_COMMAND_ID_SKIP_FORWARD;
        break;
    case HCI_CONTROL_AVRC_CONTROLLER_COMMAND_BEGIN_REWIND:
        ams_remote_command_id = AMS_REMOTE_COMMAND_ID_SKIP_BACKWARD;
        break;

    default:
        status = HCI_CONTROL_STATUS_UNKNOWN_COMMAND;
        break;
    }

    if (status == HCI_CONTROL_STATUS_SUCCESS)
    {
        wiced_bt_ams_client_send_remote_command(ams_remote_command_id);
    }

    hci_control_send_command_status_evt(HCI_CONTROL_AVRC_CONTROLLER_EVENT_COMMAND_STATUS, status);
}
#endif

/*
 * Process HCI commands from the MCU
 */
#ifdef __ANCS_API__H
static void hci_control_ancs_handle_command(uint16_t opcode, uint8_t *p_data, uint16_t payload_len)
{
    wiced_bool_t result;

    result = wiced_ancs_client_send_remote_command(p_data[0] + (p_data[1] << 8) + (p_data[2] << 16) + (p_data[3] << 24), p_data[4]);

    hci_control_send_command_status_evt(HCI_CONTROL_ANCS_EVENT_COMMAND_STATUS,
                                        result ? HCI_CONTROL_STATUS_SUCCESS : HCI_CONTROL_STATUS_FAILED );
}
#endif

/*
 * Handle received command over UART. Please refer to the WICED Smart Ready
 * Software User Manual (WICED-Smart-Ready-SWUM100-R) for details on the
 * HCI UART control protocol.
*/
static uint32_t hci_control_proc_rx_cmd( uint8_t *p_buffer, uint32_t length )
{
    uint16_t opcode;
    uint16_t payload_len;
    uint8_t *p_data = p_buffer;
    uint8_t  buffer_processed = WICED_TRUE;

    if ( !p_buffer )
    {
        return HCI_CONTROL_STATUS_INVALID_ARGS;
    }

    //Expected minimum 4 byte as the wiced header
    if(( length < 4 ) || (p_data == NULL))
    {
        WICED_BT_TRACE("invalid params\r\n");
#ifndef NEW_DYNAMIC_MEMORY_INCLUDED
        wiced_transport_free_buffer( p_buffer );
#endif
        return HCI_CONTROL_STATUS_INVALID_ARGS;
    }

    STREAM_TO_UINT16(opcode, p_data);       // Get OpCode
    STREAM_TO_UINT16(payload_len, p_data);  // Gen Payload Length

    WICED_BT_TRACE( "GROUP:0x%02x CODE:0x%02x, Payload length:%d\n\r", opcode>>8,(uint8_t)opcode, payload_len );
    WICED_BT_TRACE("hci_control_proc_rx_cmd,Free mem:%d\r\n", wiced_memory_get_free_bytes());
    switch((opcode >> 8) & 0xff)
    {
    case HCI_CONTROL_GROUP_DEVICE:
        hci_control_device_handle_command( opcode, p_data, payload_len );
        break;

#ifdef WICED_APP_LE_INCLUDED
    case HCI_CONTROL_GROUP_LE:
    case HCI_CONTROL_GROUP_GATT:
        hci_control_le_handle_command( opcode, p_data, payload_len );
        break;
#endif

#ifdef WICED_APP_AUDIO_SRC_INCLUDED
    case HCI_CONTROL_GROUP_AUDIO:
        hci_control_audio_handle_command( opcode, p_data, payload_len );
        break;
#endif
#ifdef WICED_APP_AUDIO_RC_TG_INCLUDED
    case HCI_CONTROL_GROUP_AVRC_TARGET:
        hci_control_avrc_handle_command( opcode, p_data, payload_len );
        break;
#endif

    case HCI_CONTROL_GROUP_AVRC_CONTROLLER:
#ifdef WICED_APP_AUDIO_RC_TG_INCLUDED
        if ((avrcp_profile_role == AVRCP_TARGET_ROLE) &&
            (hci_control_rc_target_is_connected()))
        {
            hci_control_avrc_handle_command( opcode, p_data, payload_len );
        }
        else
#endif
        {
#ifdef __AMS_API_H	//WICED_APP_LE_SLAVE_CLIENT_INCLUDED
            if ( wiced_bt_ams_client_connection_check() )
            {
                hci_control_ams_handle_command( opcode, p_data, payload_len );
            }
#endif
#if defined(__AMS_API_H) && defined(WICED_APP_AUDIO_RC_CT_INCLUDED)
            else
#endif
#ifdef WICED_APP_AUDIO_RC_CT_INCLUDED
            {
                hci_control_avrc_handle_ctrlr_command(opcode, p_data, payload_len);
            }
#endif
        }
        break;

#ifdef WICED_APP_TEST_INCLUDED
    case HCI_CONTROL_GROUP_TEST:
        hci_control_test_handle_command( opcode, p_data, payload_len );
        break;
#endif

#ifdef WICED_APP_ANCS_INCLUDED
    case HCI_CONTROL_GROUP_ANCS:
        hci_control_ancs_handle_command( opcode, p_data, payload_len );
        break;
#endif

#ifdef WICED_APP_HFP_AG_INCLUDED
    case HCI_CONTROL_GROUP_AG:
        hci_control_ag_handle_command( opcode, p_data, payload_len );
        break;
#endif

#ifdef WICED_APP_HFP_HF_INCLUDED
    case HCI_CONTROL_GROUP_HF:
        hci_control_hf_handle_command ( opcode, p_data, payload_len );
        break;
#endif

    case HCI_CONTROL_GROUP_MISC:
        hci_control_misc_handle_command(opcode, p_data, payload_len);
        break;

    case HCI_CONTROL_GROUP_SPP:
        spp_handle_command(opcode, p_data, payload_len);
        break;

    case HCI_CONTROL_GROUP_IAP2:
        hci_iap2_handle_command(opcode, p_data, payload_len);
        break;

    default:
        WICED_BT_TRACE( "unknown class code (opcode:%x)\r\n", opcode);
        break;
    }
    if (buffer_processed)
    {
#ifndef NEW_DYNAMIC_MEMORY_INCLUDED
        // Freeing the buffer in which data is received
        wiced_transport_free_buffer( p_buffer );
#endif
    }

    return HCI_CONTROL_STATUS_SUCCESS;
}

void hci_control_device_handle_command( uint16_t cmd_opcode, uint8_t* p_data, uint32_t data_len )
{
    uint8_t bytes_written;
    switch( cmd_opcode )
    {
    case HCI_CONTROL_COMMAND_RESET:
        hci_control_handle_reset_cmd( );
        break;

    case HCI_CONTROL_COMMAND_TRACE_ENABLE:
        hci_control_handle_trace_enable( p_data );
        break;

    case HCI_CONTROL_COMMAND_SET_LOCAL_BDA:
        hci_control_handle_set_local_bda( p_data );
        break;

    case HCI_CONTROL_COMMAND_PUSH_NVRAM_DATA:
        bytes_written = hci_control_write_nvram( p_data[0] | ( p_data[1] << 8 ), data_len - 2, &p_data[2], WICED_TRUE );
        WICED_BT_TRACE( "NVRAM write: %d dev: [%B]\r\n", bytes_written , &p_data[2]);
        break;

    case HCI_CONTROL_COMMAND_DELETE_NVRAM_DATA:
        hci_control_delete_nvram( p_data[0] | ( p_data[1] << 8 ), WICED_TRUE );
        WICED_BT_TRACE( "NVRAM delete: %d\r\n", p_data[0] | ( p_data[1] << 8 ) );
        break;

    case HCI_CONTROL_COMMAND_INQUIRY:
        hci_control_inquiry( *p_data );
        break;

    case HCI_CONTROL_COMMAND_SET_VISIBILITY:
        hci_control_handle_set_visibility( p_data[0], p_data[1] );
        break;

    case  HCI_CONTROL_COMMAND_SET_PAIRING_MODE:
        hci_control_handle_set_pairability( p_data[0] );
        break;
    case HCI_CONTROL_COMMAND_UNBOND:
        hci_control_handle_unbond(p_data[0]);
        break;
#if (defined(COEX_SUPPORTED) && COEX_SUPPORTED == WICED_TRUE)
    case HCI_CONTROL_COMMAND_ENABLE_COEX:
        hci_control_handle_enable_disable_coex( WICED_TRUE );
        break;

    case HCI_CONTROL_COMMAND_DISABLE_COEX:
        hci_control_handle_enable_disable_coex( WICED_FALSE );
        break;
#endif
    case HCI_CONTROL_COMMAND_READ_LOCAL_BDA:
        hci_control_handle_read_local_bda();
        break;

    case HCI_CONTROL_COMMAND_USER_CONFIRMATION:
        hci_control_handle_user_confirmation( p_data, p_data[6] );
        break;

    case HCI_CONTROL_COMMAND_READ_BUFF_STATS:
        hci_control_handle_read_buffer_stats ();
        break;

    default:
        WICED_BT_TRACE( "??? Unknown command code\r\n" );
        break;
    }
}

/*
 * handle reset command from UART
 */
void hci_control_handle_reset_cmd( void )
{
    hci_control_send_command_status_evt( HCI_CONTROL_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_SUCCESS );

    // trip watch dog now.
    wiced_hal_wdog_reset_system( );
}

/*
 * handle command from UART to configure traces
 */
void hci_control_handle_trace_enable( uint8_t *p_data )
{
    uint8_t hci_trace_enable = *p_data++;
    wiced_debug_uart_types_t route_debug = (wiced_debug_uart_types_t)*p_data;

    WICED_BT_TRACE("HCI Traces:%d DebugRoute:%d\r\n", hci_trace_enable, route_debug);

    if ( hci_trace_enable )
    {
        /* Register callback for receiving hci traces */
        // Disable while streaming audio over the uart.
        wiced_bt_dev_register_hci_trace( hci_control_hci_packet_cback );
    }
    else
    {
        wiced_bt_dev_register_hci_trace( NULL);
    }

#if (defined(SLEEP_SUPPORTED) && (SLEEP_SUPPORTED == WICED_TRUE))
    route_debug = WICED_ROUTE_DEBUG_TO_PUART;
#endif

// In SPI transport case, PUART is recommended for debug traces and is set to PUART by default.
#if (WICED_HCI_TRANSPORT != WICED_HCI_TRANSPORT_SPI)
    wiced_set_debug_uart( route_debug );
#endif

    hci_control_send_command_status_evt( HCI_CONTROL_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_SUCCESS );
}

/*
 * handle command to set local Bluetooth device address
 */
void hci_control_handle_set_local_bda( uint8_t *p_bda)
{
    BD_ADDR bd_addr;

    STREAM_TO_BDADDR(bd_addr,p_bda);
    wiced_bt_set_local_bdaddr( bd_addr, BLE_ADDR_PUBLIC );

    hci_control_send_command_status_evt( HCI_CONTROL_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_SUCCESS );
}

/*
 *  Handle read buffer statistics
 */
void hci_control_handle_read_buffer_stats( void )
{
#if BTSTACK_VER >= 0x01020000
    /*
     * Get statistics of default heap.
     * TODO: get statistics of stack heap (btu_cb.p_heap)
     */
    wiced_bt_heap_statistics_t heap_stat;

    if (wiced_bt_get_heap_statistics(p_default_heap, &heap_stat))
    {
        WICED_BT_TRACE("--- heap_size:%d ---\r\n", heap_stat.heap_size);
        WICED_BT_TRACE("max_single_allocation:%d max_heap_size_used:%d\r\n",
                heap_stat.max_single_allocation,
                heap_stat.max_heap_size_used);
        WICED_BT_TRACE("allocation_failure_count:%d current_largest_free_size:%d\r\n",
                heap_stat.allocation_failure_count,
                heap_stat.current_largest_free_size);
        WICED_BT_TRACE("current_num_allocations:%d current_size_allocated:%d\r\n",
                heap_stat.current_num_allocations,
                heap_stat.current_size_allocated);
        WICED_BT_TRACE("current_num_free_fragments:%d current_free_size\r\n",
                heap_stat.current_num_free_fragments,
                heap_stat.current_free_size);

        wiced_transport_send_data(HCI_CONTROL_EVENT_READ_BUFFER_STATS,
                (uint8_t *)&heap_stat, sizeof(heap_stat));
    }
#else /* !BTSTACK_VER */
    uint8_t buff_pools = 0;
#ifdef WICEDX
#define BUFF_POOLS 5
    wiced_bt_buffer_statistics_t buff_stats[BUFF_POOLS];
    buff_pools = BUFF_POOLS;
#else
    wiced_bt_buffer_statistics_t buff_stats[wiced_bt_get_number_of_buffer_pools()];
    buff_pools = wiced_bt_get_number_of_buffer_pools();
#endif
    wiced_result_t result;
    uint8_t i;

    result = wiced_bt_get_buffer_usage( buff_stats, sizeof( buff_stats ) );

    if( result == WICED_BT_SUCCESS )
    {
        // Print out the stats to trace
        WICED_BT_TRACE( "Buffer usage statistics:\r\n");

        for( i=0; i < buff_pools; i++) {
            WICED_BT_TRACE("pool_id:%d size:%d curr_cnt:%d max_cnt:%d total:%d\r\n",
                           buff_stats[i].pool_id, buff_stats[i].pool_size,
                           buff_stats[i].current_allocated_count, buff_stats[i].max_allocated_count,
                           buff_stats[i].total_count);
        }

        // Return the stats via WICED-HCI
        wiced_transport_send_data( HCI_CONTROL_EVENT_READ_BUFFER_STATS, (uint8_t*)&buff_stats, sizeof( buff_stats ) );
    }
    else
#endif /* BTSTACK_VER */
    {
        hci_control_send_command_status_evt( HCI_CONTROL_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_FAILED );
    }
}

/*
 *  Handle Inquiry result callback from teh stack, format and send event over UART
 */
void hci_control_inquiry_result_cback( wiced_bt_dev_inquiry_scan_result_t *p_inquiry_result, uint8_t *p_eir_data )
{
    int       i;
    uint8_t   len;
    uint8_t   tx_buf[300];
    uint16_t  code;
    uint8_t   *p = tx_buf;

    if ( p_inquiry_result == NULL )
    {
        code = HCI_CONTROL_EVENT_INQUIRY_COMPLETE;
        WICED_BT_TRACE( "inquiry complete \r\n");
    }
    else
    {
        code = HCI_CONTROL_EVENT_INQUIRY_RESULT;
        WICED_BT_TRACE( "inquiry result %B\r\n", p_inquiry_result->remote_bd_addr );
        for ( i = 0; i < 6; i++ )
            *p++ = p_inquiry_result->remote_bd_addr[5 - i];
        for ( i = 0; i < 3; i++ )
            *p++ = p_inquiry_result->dev_class[2 - i];
        *p++ = p_inquiry_result->rssi;

        // currently callback does not pass the data of the adv data, need to go through the data
        // zero len in the LTV means that there is no more data
        while ( ( p_eir_data != NULL ) && ( len = *p_eir_data ) != 0 )
        {
            // In the HCI event all parameters should fit into 255 bytes
            if ( p + len + 1 > tx_buf + 255 )
            {
                WICED_BT_TRACE( "Bad data\r\n" );
                break;
            }
            for ( i = 0; i < len + 1; i++ )
                *p++ = *p_eir_data++;
        }
    }
    wiced_transport_send_data( code, tx_buf, ( int )( p - tx_buf ) );
}

/*
 *  Handle Inquiry command received over UART
 */
void hci_control_inquiry( uint8_t enable )
{
    wiced_result_t           result;
    wiced_bt_dev_inq_parms_t params;

    if ( enable )
    {

        memset( &params, 0, sizeof( params ) );

        params.mode             = BTM_GENERAL_INQUIRY;
        params.duration         = 5;
        params.filter_cond_type = BTM_CLR_INQUIRY_FILTER;

        result = wiced_bt_start_inquiry( &params, &hci_control_inquiry_result_cback );
        WICED_BT_TRACE( "inquiry started:%d\r\n", result );
    }
    else
    {
        result = wiced_bt_cancel_inquiry( );
        WICED_BT_TRACE( "cancel inquiry:%d\r\n", result );
    }
    hci_control_send_command_status_evt( HCI_CONTROL_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_SUCCESS );
}

/*
 *  Handle Set Visibility command received over UART
 */
void hci_control_handle_set_visibility( uint8_t discoverability, uint8_t connectability )
{
    // we cannot be discoverable and not connectable
    if ( ( ( discoverability != 0 ) && ( connectability == 0 ) ) ||
           ( discoverability > 1 ) ||
           ( connectability > 1 ) )
    {
        hci_control_send_command_status_evt( HCI_CONTROL_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_INVALID_ARGS );
    }
    else
    {
        wiced_bt_dev_set_discoverability( ( discoverability != 0 ) ? BTM_GENERAL_DISCOVERABLE : BTM_NON_DISCOVERABLE ,
                                            BTM_DEFAULT_DISC_WINDOW,
                                            BTM_DEFAULT_DISC_INTERVAL);

        wiced_bt_dev_set_connectability( ( connectability != 0 ) ? WICED_TRUE : WICED_FALSE ,
                                            BTM_DEFAULT_CONN_WINDOW,
                                            BTM_DEFAULT_CONN_INTERVAL);

        hci_control_send_command_status_evt( HCI_CONTROL_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_SUCCESS );
    }
}

/*
 *  Handle Set Pairability command received over UART
 */
void hci_control_handle_set_pairability ( uint8_t pairing_allowed )
{
    uint8_t                   status = HCI_CONTROL_STATUS_SUCCESS;
    hci_control_nvram_chunk_t *p1 = NULL;

    if ( hci_control_cb.pairing_allowed != pairing_allowed )
    {
        if ( pairing_allowed )
        {
            // Check if key buffer pool has buffer available. If not, cannot enable pairing until nvram entries are deleted
            if (wiced_bt_get_buffer_count(p_key_info_pool) <= 0)
            {
                WICED_BT_TRACE( "Err: No more memory for Pairing\r\n" );
                pairing_allowed = 0; //The key buffer pool is full therefore we cannot allow pairing to be enabled
                status = HCI_CONTROL_STATUS_OUT_OF_MEMORY;
            }
        }

        hci_control_cb.pairing_allowed = pairing_allowed;
        wiced_bt_set_pairable_mode( hci_control_cb.pairing_allowed, 0 );
        WICED_BT_TRACE( " Set the pairing allowed to %d \r\n", hci_control_cb.pairing_allowed );
    }

    hci_control_send_command_status_evt( HCI_CONTROL_EVENT_COMMAND_STATUS, status );
}

/*
 *  Unpair the specified index device, if the unpair device index is not the last
 *  the pair device sequence need to rearranged
 */
void hci_control_handle_unbond(uint8_t paired_dev_index)
{
    wiced_result_t result;
    WICED_BT_TRACE("%s : device ID =%d\r\n", __func__ ,paired_dev_index);
    /*set delete paired device cmd to stack*/
    if(WICED_SUCCESS ==  wiced_bt_dev_delete_bonded_device(paired_list->device[paired_dev_index].link_key.bd_addr))
    {
        WICED_BT_TRACE("delet paired device in wiced success, remove from NVM and resoulution db\r\n");
        wiced_transport_send_data(HCI_CONTROL_EVENT_PAIRED_DEVICE_DELETED_GARMIN, &(paired_list->device[paired_dev_index].link_key.bd_addr[0]), BD_ADDR_LEN);
        /*remove link key from resolution DB*/
        result = wiced_bt_dev_remove_device_from_address_resolution_db(&(paired_list->device[paired_dev_index].link_key));
        WICED_BT_TRACE( "remove  link key to resolution DB result = %d \n\r", result);
        /*remove paired device from pairlist and NVM storage*/
        garmin_nvm_delete_paired_device(paired_dev_index);
        garmin_nvm_save_paired_list();
        garmin_update_pairlist_to_mcu();
    }
    else
    {
        WICED_BT_TRACE("delet paired device in wiced fail");
    }
}

#if (defined(COEX_SUPPORTED) && COEX_SUPPORTED == WICED_TRUE)
/*
 *  Handle Enable/Disable Coex command received over UART
 */
static void hci_control_handle_enable_disable_coex ( wiced_bool_t enable )
{
    if ( enable )
    {
        WICED_BT_TRACE( "Enabling Coex\r\n" );
#if defined(CYW20719B1) || defined(CYW20719B2) || defined(CYW20721B1) || defined(CYW20721B2) || defined(CYW20819A1)
        wiced_bt_coex_enable(0x1E8480);
#else
        wiced_bt_coex_enable();
#endif
    }
    else
    {
        WICED_BT_TRACE( "Disabling Coex\r\n" );
        wiced_bt_coex_disable();
    }

    hci_control_send_command_status_evt( HCI_CONTROL_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_SUCCESS );
}
#endif

/*
 *  Handle Get Local BDA command received over UART
 */
void hci_control_handle_read_local_bda( void )
{
    wiced_bt_device_address_t bda = { 0 };

    wiced_bt_dev_read_local_addr(bda);
    WICED_BT_TRACE("Local Bluetooth Address: [%B]\r\n", bda);

    wiced_transport_send_data( HCI_CONTROL_EVENT_READ_LOCAL_BDA, (uint8_t*)bda , 6 );
}

/*
 *  Handle User Confirmation received over UART
 */
void hci_control_handle_user_confirmation( uint8_t *p_bda, uint8_t accept_pairing )
{
    wiced_bt_device_address_t bd_addr;

    STREAM_TO_BDADDR(bd_addr,p_bda);
    wiced_bt_dev_confirm_req_reply( accept_pairing == WICED_TRUE ? WICED_BT_SUCCESS : WICED_BT_ERROR, bd_addr);

    hci_control_send_command_status_evt( HCI_CONTROL_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_SUCCESS );
}

/*
 *  Send Device Started event through UART
 */
void hci_control_send_device_started_evt( void )
{
    wiced_transport_send_data( HCI_CONTROL_EVENT_DEVICE_STARTED, NULL, 0 );

    WICED_BT_TRACE( "maxLinks:%d maxChannels:%d maxpsm:%d rfcom max links%d, rfcom max ports:%d\r\n",
            wiced_bt_cfg_settings.l2cap_application.max_links,
            wiced_bt_cfg_settings.l2cap_application.max_channels,
            wiced_bt_cfg_settings.l2cap_application.max_psm,
            wiced_bt_cfg_settings.rfcomm_cfg.max_links,
            wiced_bt_cfg_settings.rfcomm_cfg.max_ports );
}

/*
 *  Send Device Error event through UART
 */
void hci_control_send_device_error_evt( uint8_t fw_error_code, uint8_t app_error_code )
{
    uint8_t event_data[] = { 0, 0 };

    event_data[0] = app_error_code;
    event_data[1] = fw_error_code;

    WICED_BT_TRACE( "[hci_control_send_device_error_evt]  app_error_code=0x%02x  fw_error_code=0x%02x\r\n", event_data[0], event_data[1] );

    wiced_transport_send_data( HCI_CONTROL_EVENT_DEVICE_ERROR, event_data, 2 );
}

/*
* transfer command status event to UART
*/
void hci_control_send_command_status_evt( uint16_t code, uint8_t status )
{
    wiced_transport_send_data( code, &status, 1 );
}

/*
 *  Send Pairing Completed event through UART
 */
void hci_control_send_pairing_completed_evt( uint8_t status , wiced_bt_device_address_t bdaddr )
{
    int i;

    uint8_t event_data[BD_ADDR_LEN + sizeof(uint8_t)];
    int     cmd_bytes = 0;

    event_data[cmd_bytes++] = status;

    for ( i = 0 ; i < BD_ADDR_LEN; i++ )                     // bd address
        event_data[cmd_bytes++] = bdaddr[BD_ADDR_LEN - 1 - i];

    WICED_BT_TRACE( "pairing complete evt: %B as %B status %d\r\n", bdaddr, &event_data[1], status );

    wiced_transport_send_data( HCI_CONTROL_EVENT_PAIRING_COMPLETE, event_data, cmd_bytes );
}

/*
 *  Send User Confirmation Request event through UART
 */
void hci_control_send_user_confirmation_request_evt( BD_ADDR bda, uint32_t numeric_value )
{
    uint8_t buf[10];
    uint8_t *p = &buf[6];
    memcpy( buf, bda, BD_ADDR_LEN );
    *p++ = numeric_value & 0xff;
    *p++ = (numeric_value >> 8) & 0xff;
    *p++ = (numeric_value >> 16) & 0xff;
    *p++ = (numeric_value >> 24) & 0xff;
    wiced_transport_send_data( HCI_CONTROL_EVENT_USER_CONFIRMATION, buf, 10 );
}


/*
 *  Send Encryption Changed event through UART
 */
void hci_control_send_encryption_changed_evt( uint8_t encrypted ,  wiced_bt_device_address_t bdaddr )
{
    int i;
    uint8_t event_data[BD_ADDR_LEN + sizeof(uint8_t)];
    int     cmd_bytes = 0;

    event_data[cmd_bytes++] = encrypted;

    for ( i = 0 ; i < BD_ADDR_LEN; i++ )                     // bd address
        event_data[cmd_bytes++] = bdaddr[BD_ADDR_LEN - 1 - i];

    wiced_transport_send_data( HCI_CONTROL_EVENT_ENCRYPTION_CHANGED, event_data, cmd_bytes );
}

/*
 *  send audio connect complete event to UART
 */
wiced_result_t hci_control_audio_send_connect_complete( wiced_bt_device_address_t bd_addr, uint8_t status, uint16_t handle )
{
    int i;
    const int cmd_size = BD_ADDR_LEN + sizeof(handle) + sizeof(uint8_t);
    uint8_t event_data[BD_ADDR_LEN + sizeof(handle) + sizeof(uint8_t)];

    WICED_BT_TRACE( "[%s] %B status %x handle %x\r\n", __FUNCTION__, bd_addr, status, handle );

    //Build event payload
    if ( status == WICED_SUCCESS )
    {
        for ( i = 0; i < BD_ADDR_LEN; i++ )                     // bd address
            event_data[i] = bd_addr[BD_ADDR_LEN - 1 - i];

        event_data[i++] = handle & 0xff;                        //handle
        event_data[i++]   = ( handle >> 8 ) & 0xff;

#ifdef WICED_APP_AUDIO_RC_TG_INCLUDED
        event_data[i] = wiced_bt_avrc_tg_is_peer_absolute_volume_capable( );
#endif
        return wiced_transport_send_data( HCI_CONTROL_AUDIO_EVENT_CONNECTED, event_data, cmd_size );
    }
    else
    {
        return wiced_transport_send_data( HCI_CONTROL_AUDIO_EVENT_CONNECTION_FAILED, NULL, 0 );
    }
}

/*
 *  send audio disconnect complete event to UART
 */
wiced_result_t hci_control_audio_send_disconnect_complete( uint16_t handle, uint8_t status, uint8_t reason )
{
    uint8_t event_data[4];

    WICED_BT_TRACE( "[%s] %04x status %d reason %d\r\n", __FUNCTION__, handle, status, reason );

    //Build event payload
    event_data[0] = handle & 0xff;                          //handle
    event_data[1] = ( handle >> 8 ) & 0xff;
    event_data[2] = status;                                 // status
    event_data[3] = reason;                                 // reason(1 byte)

    return wiced_transport_send_data( HCI_CONTROL_AUDIO_EVENT_DISCONNECTED, event_data, 4 );
}

/*
 *  send audio connect complete event to UART
 */
wiced_result_t hci_control_audio_send_started_stopped( uint16_t handle, wiced_bool_t started )
{
    uint8_t event_data[2];

    WICED_BT_TRACE( "[%s] handle %04x started:%d", __FUNCTION__, handle, started );

    //Build event payload
    event_data[0] = handle & 0xff;                          //handle
    event_data[1] = ( handle >> 8 ) & 0xff;

    return wiced_transport_send_data(started ? HCI_CONTROL_AUDIO_EVENT_STARTED : HCI_CONTROL_AUDIO_EVENT_STOPPED, event_data, 2);
}

/*
 *  send AVRC event to UART
 */
wiced_result_t hci_control_send_avrc_target_event( int type, uint16_t handle )
{
    uint8_t event_data[2];

    WICED_BT_TRACE( "[%s] handle %04x\r\n", __FUNCTION__, handle );

    //Build event payload
    event_data[0] = handle & 0xff;                          //handle
    event_data[1] = ( handle >> 8 ) & 0xff;

    return wiced_transport_send_data( type, event_data, 2 );
}

/*********************************************************************************************
 * AVRCP controller event handlers
 *********************************************************************************************/
/*
 *  send avrcp controller complete event to UART
 */
wiced_result_t hci_control_avrc_send_connect_complete( wiced_bt_device_address_t bd_addr, uint8_t status, uint16_t handle )
{
    int i = 0;
    const int cmd_size = BD_ADDR_LEN + sizeof(handle) + sizeof(uint8_t);
    uint8_t event_data[BD_ADDR_LEN + sizeof(handle) + sizeof(uint8_t)];

    WICED_BT_TRACE( "[%s] %B status %x handle %x\r\n", __FUNCTION__, bd_addr, status, handle );

    //Build event payload
    if ( status == WICED_SUCCESS )
    {
        for ( ; i < BD_ADDR_LEN; i++ )                     // bd address
            event_data[i] = bd_addr[BD_ADDR_LEN - 1 - i];

        event_data[i++] = status;

        event_data[i++] = handle & 0xff;                        //handle
        event_data[i++] = ( handle >> 8 ) & 0xff;

    }
    else
    {
        event_data[i++] = status;
    }

    return wiced_transport_send_data( HCI_CONTROL_AVRC_CONTROLLER_EVENT_CONNECTED, event_data, i );
}

/*
 *  send avrcp controller disconnect complete event to UART
 */
wiced_result_t hci_control_avrc_send_disconnect_complete( uint16_t handle )
{
    uint8_t event_data[4];

    WICED_BT_TRACE( "[%s] handle: %04x\r\n", __FUNCTION__, handle );

    //Build event payload
    event_data[0] = handle & 0xff;                          //handle
    event_data[1] = ( handle >> 8 ) & 0xff;

    return wiced_transport_send_data( HCI_CONTROL_AVRC_CONTROLLER_EVENT_DISCONNECTED, event_data, 2 );
}

/*
 *  send avrcp controller connect complete event to UART
 */
wiced_result_t hci_control_avrc_send_play_status_change( uint16_t handle, uint8_t play_status )
{
    uint8_t event_data[3];

    WICED_BT_TRACE( "[%s] handle %04x\r\n", __FUNCTION__, handle );

    //Build event payload
    event_data[0] = handle & 0xff;                          //handle
    event_data[1] = ( handle >> 8 ) & 0xff;

    event_data[2] = play_status & 0xff;                      // play status

    return wiced_transport_send_data(HCI_CONTROL_AVRC_CONTROLLER_EVENT_PLAY_STATUS, event_data, 3);
}

/*
 *  send AVRC event to UART
 */

wiced_result_t hci_control_send_avrc_event( int type, uint8_t *p_data, uint16_t data_size )
{
    return wiced_transport_send_data( type, p_data, data_size );
}

/*
 * Write NVRAM function is called to store information in the RAM.  This can be called when
 * stack requires persistent storage, for example to save link keys.  In this case
 * data is also formatted and send to the host for real NVRAM storage.  The same function is
 * called when host pushes NVRAM chunks during the startup.  Parameter from_host in this
 * case is set to WICED_FALSE indicating that data does not need to be forwarded.
 */
int hci_control_write_nvram( int nvram_id, int data_len, void *p_data, wiced_bool_t from_host )
{
    uint8_t                    tx_buf[257];
    uint8_t                   *p = tx_buf;
    hci_control_nvram_chunk_t *p1;
    wiced_result_t            result;

    /* first check if this ID is being reused and release the memory chunk */
    hci_control_delete_nvram( nvram_id, WICED_FALSE );

    /* Allocating a buffer from the pool created for storing the peer info */
    if ( ( p1 = ( hci_control_nvram_chunk_t * )wiced_bt_get_buffer_from_pool( p_key_info_pool ) ) == NULL)
    {
        WICED_BT_TRACE( "Failed to alloc:%d\r\n", data_len );
        return ( 0 );
    }

    if ( wiced_bt_get_buffer_size( p1 ) < ( sizeof( hci_control_nvram_chunk_t ) + data_len - 1 ) )
    {
        WICED_BT_TRACE( "Insufficient buffer size, Buff Size %d, Len %d  \r\n",
                        wiced_bt_get_buffer_size( p1 ),
                        ( sizeof( hci_control_nvram_chunk_t ) + data_len - 1 ) );
        wiced_bt_free_buffer( p1 );
        return ( 0 );
    }

    p1->p_next    = p_nvram_first;
    p1->nvram_id  = nvram_id;
    p1->chunk_len = data_len;
    memcpy( p1->data, p_data, data_len );

    p_nvram_first = p1;

    wiced_bt_device_link_keys_t * p_keys = ( wiced_bt_device_link_keys_t *) p_data;
#ifdef CYW20706A2
    result = wiced_bt_dev_add_device_to_address_resolution_db( p_keys, p_keys->key_data.ble_addr_type );
#else
    result = wiced_bt_dev_add_device_to_address_resolution_db( p_keys );
#endif
    WICED_BT_TRACE("Updated Addr Resolution DB:%d\r\n", result );

    // If NVRAM chunk arrived from host, no need to send it back, otherwise send over transport
    if (!from_host)
    {
        *p++ = nvram_id & 0xff;
        *p++ = (nvram_id >> 8) & 0xff;
        memcpy(p, p_data, data_len);

        wiced_transport_send_data( HCI_CONTROL_EVENT_NVRAM_DATA, tx_buf, ( int )( data_len + 2 ) );
    }
    else
    {
        hci_control_send_command_status_evt( HCI_CONTROL_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_SUCCESS );
    }
    return (data_len);
}

/*
 * Find nvram_id of the NVRAM chunk with first bytes matching specified byte array
 */
int hci_control_find_nvram_id(uint8_t *p_data, int len)
{
    hci_control_nvram_chunk_t *p1;

    /* Go through the linked list of chunks */
    for (p1 = p_nvram_first; p1 != NULL; p1 = (hci_control_nvram_chunk_t *)p1->p_next)
    {
        WICED_BT_TRACE( "find %B %B len:%d\r\n", p1->data, p_data, len );
        if ( memcmp( p1->data, p_data, len ) == 0 )
        {
            return ( p1->nvram_id );
        }
    }
    return HCI_CONTROL_INVALID_NVRAM_ID;
}

/*
 * Delete NVRAM function is called when host deletes NVRAM chunk from the persistent storage.
 */
void hci_control_delete_nvram( int nvram_id, wiced_bool_t from_host )
{
    hci_control_nvram_chunk_t *p1, *p2;

    /* If Delete NVRAM data command arrived from host, send a Command Status response to ack command */
    if (from_host)
    {
        hci_control_send_command_status_evt( HCI_CONTROL_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_SUCCESS );
    }

    if ( p_nvram_first == NULL )
        return;

    /* Special case when need to remove the first chunk */
    if ( ( p_nvram_first != NULL ) && ( p_nvram_first->nvram_id == nvram_id ) )
    {
        p1 = p_nvram_first;

        if ( from_host && ( wiced_bt_dev_delete_bonded_device (p1->data) == WICED_ERROR ) )
        {
            WICED_BT_TRACE("ERROR: while Unbonding device \r\n");
        }
        else
        {
            p_nvram_first = (hci_control_nvram_chunk_t *)p_nvram_first->p_next;
            wiced_bt_free_buffer( p1 );
        }
        return;
    }

    /* Go through the linked list of chunks */
    for ( p1 = p_nvram_first; p1 != NULL; p1 = (hci_control_nvram_chunk_t *)p1->p_next )
    {
        p2 = (hci_control_nvram_chunk_t *)p1->p_next;

        if ( ( p2 != NULL ) && ( p2->nvram_id == nvram_id ) )
        {
            if ( from_host && ( wiced_bt_dev_delete_bonded_device (p2->data) == WICED_ERROR ) )
            {
                WICED_BT_TRACE("ERROR: while Unbonding device \r\n");
            }
            else
            {
                p1->p_next = p2->p_next;
                wiced_bt_free_buffer( p2 );
            }
            return;
        }
    }
}

/*
 * Read NVRAM actually finds the memory chunk in the RAM
 */
int hci_control_read_nvram( int nvram_id, void *p_data, int data_len )
{
    hci_control_nvram_chunk_t *p1;
    int                        data_read = 0;

    /* Go through the linked list of chunks */
    for ( p1 = p_nvram_first; p1 != NULL; p1 = p1->p_next )
    {
        if ( p1->nvram_id == nvram_id )
        {
            data_read = ( data_len < p1->chunk_len ) ? data_len : p1->chunk_len;
            memcpy( p_data, p1->data, data_read );
            break;
        }
    }
    return ( data_read );
}

/*
 * Allocate nvram_id to save new NVRAM chunk
 */
int hci_control_alloc_nvram_id( void )
{
    hci_control_nvram_chunk_t *p1 = p_nvram_first;
    int                        nvram_id;
    uint8_t                    allocated_key_pool_count;

    /* Go through the linked list of chunks */
    WICED_BT_TRACE ( "hci_control_alloc_nvram_id\r\n" );
    for ( nvram_id = HCI_CONTROL_FIRST_VALID_NVRAM_ID; p1 != NULL; nvram_id++ )
    {
        allocated_key_pool_count = 1;

        for ( p1 = p_nvram_first; p1 != NULL; p1 = (hci_control_nvram_chunk_t *)p1->p_next )
        {
            /* If the key buffer pool is becoming full, we need to notify the mcu and disable Pairing.
             * The mcu will need to delete some nvram entries and enable pairing in order to
             * pair with more devices */
            allocated_key_pool_count++;
            if ( ( allocated_key_pool_count == KEY_INFO_POOL_BUFFER_COUNT ) && ( hci_control_cb.pairing_allowed ) )
            {
                // Send Max Number of Paired Devices Reached event message
                wiced_transport_send_data( HCI_CONTROL_EVENT_MAX_NUM_OF_PAIRED_DEVICES_REACHED, NULL, 0 );

                hci_control_cb.pairing_allowed = WICED_FALSE;
                wiced_bt_set_pairable_mode( hci_control_cb.pairing_allowed, 0 );
            }

            if ( p1->nvram_id == nvram_id )
            {
                /* this nvram_id is already used */
                break;
            }
        }
        if ( p1 == NULL )
        {
            break;
        }
    }
    WICED_BT_TRACE ( "hci_control_alloc_nvram_id:%d\r\n", nvram_id );
    return ( nvram_id );
}



/*
 * Remote Control can work a target or a controller.  This function sets up the appropriate role.
 */
void hci_control_switch_avrcp_role(uint8_t new_role)
{
#if ( defined(WICED_APP_AUDIO_RC_TG_INCLUDED) && defined(WICED_APP_AUDIO_RC_CT_INCLUDED) )
    WICED_BT_TRACE ( "[%s] New Role: %d \r\n", __FUNCTION__, new_role);

    if (new_role != avrcp_profile_role)
    {
        switch (avrcp_profile_role)
        {
        case AVRCP_TARGET_ROLE:
            /* Shutdown the avrcp target */
            wiced_bt_avrc_tg_initiate_close();

            /* Initialize the avrcp controller */
            hci_control_rc_controller_init();

            avrcp_profile_role = new_role;
            break;

        case AVRCP_CONTROLLER_ROLE:
            /* Shutdown the avrcp controller */
            wiced_bt_avrc_ct_cleanup();

            /* Initialize the avrcp target */
            hci_control_rc_target_init();
            wiced_bt_avrc_tg_register();

            avrcp_profile_role = new_role;
            break;

        default:
            break;
        }
    }
#endif
}

/*
 * hci_control_transport_tx_cplt_cback.
 * This function is called when a Transport Buffer has been sent to the MCU
 */
#ifdef NEW_DYNAMIC_MEMORY_INCLUDED
static void hci_control_transport_tx_cplt_cback(void)
{
    return;
}
#else
static void hci_control_transport_tx_cplt_cback(wiced_transport_buffer_pool_t* p_pool)
{
    WICED_BT_TRACE( " hci_control_transport_tx_cplt_cback %x \r\n", p_pool );
}
#endif

static void hci_control_transport_status( wiced_transport_type_t type )
{
    WICED_BT_TRACE( " hci_control_transport_status %x \r\n", type );
    hci_control_send_device_started_evt();
#ifdef SWITCH_PTU_CHECK
    platform_transport_started = 1;
#endif
}

void hci_control_switch_hfp_role( uint8_t new_role )
{
#if (defined(WICED_APP_HFP_AG_INCLUDED) && defined(WICED_APP_HFP_HF_INCLUDED))
    WICED_BT_TRACE("[%s] Switch from %d to %d\r\n", __FUNCTION__, hfp_profile_role, new_role);
    /* switch to new role */
    hfp_profile_role = new_role;
#endif /* (defined(WICED_APP_HFP_AG_INCLUDED) && defined(WICED_APP_HFP_HF_INCLUDED)) */
    return;
}
