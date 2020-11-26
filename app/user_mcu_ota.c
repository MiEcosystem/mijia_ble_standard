/*
 * user_mcu_ota.c
 *
 *  Created on: 2020Äê10ÔÂ26ÈÕ
 *      Author: mi
 */
#include "mible_mcu.h"

#include <string.h>
#include <stdio.h>
#include "bg_types.h"
#include "mible_log.h"

#if (defined(MCU_OTA_DEMO) && MCU_OTA_DEMO)
#include "gatt_dfu/mible_dfu_main.h"
#include "usart_api.h"
#include "xmodem.h"
#include "sleep.h"
miio_xmodem_t xmodem;
uint8_t recv[64]={0};
size_t read_cnt = 0;

mible_status_t mible_mcu_cmd_send(mible_mcu_cmd_t cmd, void* arg)
{
    /* add your own code here*/
    mible_status_t ret = MI_SUCCESS;
    unsigned int last_index = 0;
    unsigned int crc32 = 0;
    unsigned int recv_bytes = 0;
    int send_len = 0;
    SLEEP_SleepBlockBegin(sleepEM2);
    mible_usart_flush(true, true);
    switch(cmd) {
    case MIBLE_MCU_GET_VERSION:
        mible_usart_send_buffer("down get_ver\r", strlen("down get_ver\r"), true);
        /* Wait until recv 4 byte version or delay 100ms */
        mible_usart_receive_delay_init(4, 100);
        break;
    case MIBLE_MCU_READ_DFUINFO:
        mible_usart_send_buffer("down rd_info\r", strlen("down dfu_rdinfo\r"), true);
        /* Wait until recv 26byte or delay 100ms */
        mible_usart_receive_delay_init(26, 100);
        break;
    case MIBLE_MCU_WRITE_DFUINFO:
        last_index = ((mible_dfu_info_t *)arg)->last_index;
        crc32 = ((mible_dfu_info_t *)arg)->crc32;
        recv_bytes = ((mible_dfu_info_t *)arg)->recv_bytes;
        sprintf((char *)recv,"down wr_info %d %08x %d\r", last_index, crc32, recv_bytes);
        mible_usart_send_buffer((const char *)recv, strlen((const char *)recv), true);
        /* Wait until recv "ok\r" or delay 100ms */
        mible_usart_receive_delay_init(3, 100);
        break;
    case MIBLE_MCU_WRITE_FIRMWARE:
        sprintf((char *)recv,"down nvm_write %08x\r", (unsigned int)(((mible_mcu_nvminfo_t*)arg)->address));
        mible_usart_send_buffer((const char *)recv, strlen((const char *)recv), true);
        /* Wait until recv "ok\r" or delay 100ms */
        mible_usart_receive_delay_init(3, 100);
        break;
    case MIBLE_MCU_VERIFY_FIRMWARE:
        mible_usart_send_buffer("down dfu_verify\r", strlen("down dfu_verify\r"), true);
        /* Wait until recv "ok\r" or delay 2000ms */
        mible_usart_receive_delay_init(3, 1000);
        while(MI_ERR_BUSY == mible_usart_receive_delay_expired());
        mible_usart_receive_delay_init(3, 1000);
        break;
    case MIBLE_MCU_UPGRADE_FIRMWARE:
        mible_usart_send_buffer("down dfu_active\r", strlen("down dfu_active\r"), true);
        /* Wait until recv "ok\r" or delay 100ms */
        mible_usart_receive_delay_init(3, 100);
        break;
    case MIBLE_MCU_TRANSFER:
        if(MI_SUCCESS != miio_xmodem_create_instance(&xmodem)) {
            MI_LOG_ERROR("miio_xmodem_create_instance FAIL!!\n");
            ret = MI_ERR_NO_MEM;
            break;
        }
        MI_LOG_DEBUG("mible_mcu_nvm_write addr %08x, len %d\n",
                ((mible_mcu_nvminfo_t*)arg)->address, ((mible_mcu_nvminfo_t*)arg)->length);
        send_len = xmodem_transfer_data(&xmodem, ((mible_mcu_nvminfo_t*)arg)->p_data,
                                        ((mible_mcu_nvminfo_t*)arg)->length);
        MI_LOG_INFO("xmodem_transfer_data send_len %d\n", send_len);
        if(send_len < 0){
            MI_LOG_ERROR("send_len %d, length %d\n", send_len, ((mible_mcu_nvminfo_t*)arg)->length);
            ret = MI_ERR_DATA_SIZE;
        }
        break;
    default:
        break;
    }
    if(MI_SUCCESS != ret)
        SLEEP_SleepBlockEnd(sleepEM2);
    return ret;
}

mible_status_t mible_mcu_cmd_wait(mible_mcu_cmd_t cmd, void* arg)
{
    /* add your own code here*/
    mible_status_t ret = MI_ERR_BUSY;
    unsigned int last_index = 0;
    unsigned int crc32 = 0;
    unsigned int recv_bytes = 0;
    switch(cmd) {
    case MIBLE_MCU_GET_VERSION:
        ret = mible_usart_receive_delay_expired();
        if(MI_SUCCESS == ret){
            mible_usart_receive_buffer(recv, 4, &read_cnt, false, 0);
            mible_usart_flush(false, true);
            //MI_LOG_INFO("mible_mcu_get_info %d byte:\n",read_cnt);
            //MI_LOG_HEXDUMP(recv, read_cnt);
            if(read_cnt == 4) {
                memcpy((uint8_t*)arg, recv, 4);
            } else {
                ret = MI_ERR_DATA_SIZE;
            }
        }
        break;
    case MIBLE_MCU_READ_DFUINFO:
        ret = mible_usart_receive_delay_expired();
        if(MI_SUCCESS == ret){
            mible_usart_receive_buffer(recv, 26, &read_cnt, false, 100);
            mible_usart_flush(false, true);
            //MI_LOG_INFO("mible_mcu_read_dfuinfo %d byte:\n",read_cnt);
            //MI_LOG_HEXDUMP(recv, read_cnt);
            if(read_cnt && 0 == strncmp((const char *)recv,"ok",strlen("ok"))){
                read_cnt = sscanf((const char *)recv, "%*s%d%08x%d", &last_index, &crc32, &recv_bytes);
                if(read_cnt == 3){
                    MI_LOG_INFO("mible_mcu_read_dfuinfo succ index %d, cec32 %08x, recv %d!!!\n",
                            last_index, crc32, recv_bytes);
                    ((mible_dfu_info_t *)arg)->last_index = last_index;
                    ((mible_dfu_info_t *)arg)->crc32 = crc32;
                    ((mible_dfu_info_t *)arg)->recv_bytes = recv_bytes;
                }else{
                    MI_LOG_ERROR("mible_mcu_read_dfuinfo fail : param %d!!!\n", read_cnt);
                    ret = MI_ERR_INVALID_PARAM;
                }
            }else{
                MI_LOG_ERROR("mible_mcu_read_dfuinfo fail !!!\n");
                ret = MI_ERR_NOT_FOUND;
            }
        }
        break;
    case MIBLE_MCU_WRITE_DFUINFO:
        ret = mible_usart_receive_delay_expired();
        if(MI_SUCCESS == ret){
            mible_usart_receive_buffer(recv, 3, &read_cnt, false, 100);
            mible_usart_flush(false, true);
            //MI_LOG_INFO("mible_mcu_write_dfuinfo %d byte:\n",read_cnt);
            //MI_LOG_HEXDUMP(recv, read_cnt);
            if(read_cnt && 0 == strncmp((const char *)recv,"ok",strlen("ok"))){
                MI_LOG_INFO("mible_mcu_write_dfuinfo succ !!!\n");
            }else{
                MI_LOG_ERROR("mible_mcu_write_dfuinfo fail !!!\n");
                ret = MI_ERR_NOT_FOUND;
            }
        }
        break;
    case MIBLE_MCU_WRITE_FIRMWARE:
        ret = mible_usart_receive_delay_expired();
        if(MI_SUCCESS == ret){
            mible_usart_receive_buffer(recv, 3, &read_cnt, false, 100);
            mible_usart_flush(false, true);
            //MI_LOG_INFO("mible_mcu_nvm_write %d byte:\n",read_cnt);
            //MI_LOG_HEXDUMP(recv, read_cnt);

            if(read_cnt && 0 == strncmp((const char *)recv,"ok",strlen("ok"))){
                MI_LOG_INFO("receive ok, start xmodem!!!\n");
            }else{
                MI_LOG_ERROR("mible_mcu_nvm_write fail !!!\n");
                ret = MI_ERR_NOT_FOUND;
            }
        }
        break;
    case MIBLE_MCU_VERIFY_FIRMWARE:
        ret = mible_usart_receive_delay_expired();
        if(MI_SUCCESS == ret){
            mible_usart_receive_buffer(recv, 3, &read_cnt, false, 2000);
            mible_usart_flush(false, true);
            //MI_LOG_INFO("mible_mcu_verify_firmware %d byte:\n",read_cnt);
            //MI_LOG_HEXDUMP(recv, read_cnt);

            if(read_cnt && 0 == strncmp((const char *)recv,"ok",strlen("ok"))){
                MI_LOG_INFO("mible_mcu_verify_firmware succ !!!\n");
            }else{
                MI_LOG_ERROR("mible_mcu_verify_firmware fail !!!\n");
                ret = MI_ERR_NOT_FOUND;
            }
        }
        break;
    case MIBLE_MCU_UPGRADE_FIRMWARE:
        ret = mible_usart_receive_delay_expired();
        if(MI_SUCCESS == ret){
            mible_usart_receive_buffer(recv, 3, &read_cnt, false, 100);
            mible_usart_flush(false, true);
            //MI_LOG_INFO("mible_mcu_upgrade_firmware %d byte:\n",read_cnt);
            //MI_LOG_HEXDUMP(recv, read_cnt);

            if(read_cnt && 0 == strncmp((const char *)recv,"ok",strlen("ok"))){
                MI_LOG_INFO("mible_mcu_upgrade_firmware succ !!!\n");
            }else{
                MI_LOG_ERROR("mible_mcu_upgrade_firmware fail !!!\n");
                ret = MI_ERR_NOT_FOUND;
            }
        }
        break;
    case MIBLE_MCU_TRANSFER:
        ret = MI_SUCCESS;
        break;
    default:
        break;
    }
    if(MI_ERR_BUSY != ret)
        SLEEP_SleepBlockEnd(sleepEM2);
    return ret;
}
#endif
