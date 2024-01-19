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
 * This file implements the Miscellaneous Commands controlled over UART. Please refer to the
 * WICED HCI Control Protocol Software User Manual (WICED-SWUM10x-R) for additional
 * details on the HCI UART control protocol
 */


#ifndef _HCI_CONTROL_MISC_H_
#define _HCI_CONTROL_MISC_H_

#include "wiced_app.h"

/******************************************************************************
 *                          Types
 ******************************************************************************/


/******************************************************************************
 *                          Functions
 ******************************************************************************/
/* Miscellaneous Commands functions */
void hci_control_misc_read_pair_info
    (
    uint8_t pair_dev_index
    );

uint8_t hci_read_nvram_link_keys
    (
    void
    );

uint8_t hci_read_pair_dev_num
    (
    void
    );

void hci_pair_dev_list_return
    (
    uint8_t pair_dev_index
    );
void hci_control_le_handle_misc_advertise_cmd
    (
    uint8_t* device_name,
    uint32_t device_name_length
    );
#endif /* _HCI_CONTROL_MISC_H_ */
