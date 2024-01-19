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

/** @file
*
* Prototypes and definitions for WICED IAP2 implementation
*
*/

#ifndef __IAP2_INT_H
#define __IAP2_INT_H

#include "wiced.h"
#include "wiced_timer.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_iap2.h"
#include "iap2_defs.h"

#define IAP2_HI_BYTE(X) (((X) >> 8) & 0xFF)
#define IAP2_LO_BYTE(X) ((X) & 0xFF)

#define     IAP2_BUFFER_POOL                    2
#if 1 //def WICED_BT_TRACE_ENABLE
#define     IAP2_TRACE                          WICED_BT_TRACE
#else
#define     IAP2_TRACE(...)
#endif

#define L2CAP_MIN_OFFSET                13      /* from l2c_api.h */
#define RFCOMM_MIN_OFFSET               5       /* from rfc_defs.h */
#define PORT_SUCCESS                    0       /* from port_api.h */

// Session IDs
#define IAP2_CONTROL_SESSION_ID                 0xA
#define IAP2_EA_SESSION_ID                      0xB

// Definitions for chip info registers
#define AUTH_CHIP_DEVICE_VERSION                0x00
#define AUTH_CHIP_FIRMWARE_VERSION              0x01
#define AUTH_CHIP_AUTH_PROTOCOL_VERSION_MAJOR   0x02
#define AUTH_CHIP_AUTH_PROTOCOL_VERSION_MINOR   0x03
#define AUTH_CHIP_DEVICE_ID                     0x04


/*
 * Define events processed by link state machine
 */
#define IAP2_LINK_TIMEOUT                   3
#define IAP2_LINK_DETECT                    4
#define IAP2_LINK_SYN_ACK                   5
#define IAP2_LINK_RFCOMM_DATA               6
#define IAP2_LINK_APP_DATA                  7

extern uint16_t     iap2_port_handle;

/* iAP2 control block */
typedef struct
{
#define     IAP2_TRANSPORT_STATE_IDLE       0
#define     IAP2_TRANSPORT_STATE_OPENING    1
#define     IAP2_TRANSPORT_STATE_OPEN       2
#define     IAP2_TRANSPORT_STATE_CLOSING    3

    uint8_t     state;                  /* state machine state */

    uint8_t     b_is_initiator;         /* initiator of the connection ( true ) or acceptor ( false ) */

    uint16_t    rfc_serv_handle;        /* RFCOMM server handle */
    uint16_t    rfc_conn_handle;        /* RFCOMM handle of connected service */
    uint8_t     server_scn;             /* server's scn */
    BD_ADDR     server_addr;            /* server's bd address */

    void        *p_sdp_discovery_db;                /* pointer to discovery database */
    wiced_bool_t flow_control_on;       /* Tx flow control */

} iap2_transport_scb_t;

typedef struct
{
    uint8_t     state;
    uint16_t    dataCurLen;
    uint8_t     dataChecksum;
    uint16_t    packetLen;
    uint16_t    bufferLen;
    uint8_t     pckData[kIAP2PacketHeaderLen];
    uint8_t     acked_seq;
    uint8_t     max_outstanding;
    uint8_t     max_unacked;
    BT_HDR*     p_prev_buf;
} iap2_recv_t;

extern iap2_recv_t iap2_recv;

extern BD_ADDR              bd_addr_connected;

extern wiced_timer_t        iap2_detect_timer;
extern wiced_timer_t        iap2_ack_timer;

extern wiced_bt_iap2_reg_t* p_iap2_reg;

wiced_result_t auth_chip_copy_device_certificate(uint8_t *out_certificate_ptr, uint16_t *out_certificate_len);
wiced_result_t auth_chip_platform_create_signature( const void * in_digest_ptr, uint16_t in_digest_len, uint8_t *out_signature_ptr, uint16_t *out_signature_len );

#ifdef __cplusplus
extern "C" {
#endif

extern uint8_t iap2_link_send_seq;
extern uint8_t iap2_link_recv_seq;

void iap2_link_process_event(uint8_t event, void *p_data);

void iap2_send_syn(uint8_t retry);
void iap2_send_incompatible(void);
void iap2_send_detect(void);
void iap2_send_ack(uint8_t session_id);
void iap2_send_tx_buf(BT_HDR *p_buf, uint8_t control, uint8_t seq, uint8_t ack, uint8_t session);

wiced_bool_t iap2_process_ea_session_data(uint8_t* data, uint32_t dataLen);

/* External Definition */
extern int PORT_Write (uint16_t handle, BT_HDR *p_buf);
extern int PORT_Read (uint16_t handle, BT_HDR **pp_buf);
extern void *GKI_getpoolbuf (uint8_t pool_id);
extern void *GKI_getbuf (uint16_t);
extern void GKI_freebuf (void *memPtr);
extern uint16_t GKI_get_pool_bufsize (UINT8 pool_id);
extern uint16_t GKI_poolcount (uint8_t pool_id);
extern uint16_t GKI_poolfreecount (uint16_t pool_id);
extern void iap2_recv_cleanup(void);
extern void iap2_recv_data(BT_HDR *p_buf);
extern uint16_t GKI_get_buf_size(void *p_buf);
extern int ulp_getRand (void);
extern wiced_bool_t iap2_process_control_session_data(uint8_t* data, uint32_t dataLen);

void get_iap2_connection_handle(uint16_t * handle_arry);
void set_iap2_connection_handle(uint16_t * handle_arry);

#ifdef __cplusplus
}
#endif

#endif
