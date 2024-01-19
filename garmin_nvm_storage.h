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



#ifndef _GARMINSPP_NVM_STORAGE_H_
#define _GARMINSPP_NVM_STORAGE_H_

#include "wiced_bt_types.h"
#include "wiced_bt_dev.h"
#define MAX_PAIRED_DEVICE_NUM         4
#define INVAILD_ID                    0xFF

//NVM ID that storage current paired device list num
#define PAIRLIST_NUM_NVRAM_ID         (WICED_NVRAM_VSID_START + 1)
//NVM ID that storage paired device 1, and device 2 would be device 1 ID+1 ,as go on
#define PAIR_DEVICE1_NVRAM_ID         (PAIRLIST_NUM_NVRAM_ID + 1)
#define TEST_NVRAM_ID         (WICED_NVRAM_VSID_START + 9)

typedef struct BT_paired_device_info
{
    uint8_t device_name[32];
    wiced_bt_device_type_t device_type;
    wiced_bt_device_link_keys_t link_key;
    uint8_t iap_support;
}BT_paired_device_info_t;

typedef struct BT_paired_device_list
{
    uint8_t                 paired_device_num;
    BT_paired_device_info_t device[MAX_PAIRED_DEVICE_NUM];
}BT_paired_device_list_t;

/*****************************************************************************
**  Function prototypes
*****************************************************************************/
/* api functions */
void garmin_nvm_read_paired_device_num(void);
void garmin_nvm_read_paired_list(void);
void garmin_nvm_save_paired_list(void);
void garmin_nvm_save_paired_device(BT_paired_device_info_t* device);
void garmin_nvm_save_paired_device_by_id(uint8_t* device_id);
void garmin_nvm_dump_pair_list(void);
void garmin_nvm_dump_link_key(wiced_bt_device_link_keys_t* link_key);
uint8_t garmin_nvm_find_device_id(wiced_bt_device_address_t addr);
void garmin_nvm_add_paired_device(BT_paired_device_info_t* device);
void garmin_nvm_delete_paired_device(uint8_t id);
void garmin_nvm_erase_nvm_storage(void);
void garmin_update_pairlist_to_mcu(void);
int bdcmp(const BD_ADDR a, const BD_ADDR b);
#endif /* _GARMINSPP_NVM_STORAGE_H_ */
