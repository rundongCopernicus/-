/*
 * falsh.c
 *
 *  Created on: 2026年6月21日
 *      Author: jred
 */


#include "zf_common_headfile.h"
#include "flash.h"
#include "navigation.h"  // 包含 navigation.h 以解析 MaxSize 和 Nag_End_Page

/**
 * @brief Flash导航数据写入函数
 * @details 将导航数据写入Flash存储器的指定页
 *          - 检查目标页是否有数据，如有则先擦除
 *          - 将缓冲区数据写入Flash页
 *          - 如果是最后一页，额外保存索引信息到结束页
 *          - 清空Flash缓冲区
 */
void flash_Nag_Write(){
    // 检查Flash页是否有数据，如果有则擦除该页（Flash写入前必须先擦除）
    if(flash_check(0, N.Flash_page_index))
        flash_erase_page(0, N.Flash_page_index);

    // 将缓冲区数据写入到指定的Flash页
    flash_write_page_from_buffer(0, N.Flash_page_index);

    // 如果是数据结束标志，保存当前索引到结束页
    if(N.End_f == 1)
    {
        // 将保存索引写入缓冲区的特定位置（MaxSize+2）
        flash_union_buffer[MaxSize+2].uint32_type = N.Save_index;
        // 将索引信息写入结束页，用于后续读取时确定数据范围
        flash_write_page_from_buffer(0, Nag_End_Page);
    }

    // 清空Flash缓冲区，为下次写入做准备
    flash_buffer_clear();
}

/**
 * @brief Flash导航数据读取函数
 * @details 从Flash存储器读取导航数据到缓冲区
 *          - 首次调用时读取结束页获取保存的索引信息
 *          - 检查目标页是否有数据，如有则读取到缓冲区
 *          - 使用静态变量确保索引只读取一次
 */
// 静态变量，用于标记是否已读取过索引（0=未读取，1=已读取）
static uint8 Index_R_f = 0;

// 重置Flash读取标志（用于新的记录/回放周期）
void flash_Nag_Reset() {
    Index_R_f = 0;
}

void flash_Nag_Read(){
    // 清空Flash缓冲区
//    flash_buffer_clear();

    // 首次调用时，从结束页读取保存的索引信息
    if(0 == Index_R_f)
    {
        // 读取结束页数据到缓冲区
        flash_read_page_to_buffer(0, Nag_End_Page);

        // 从缓冲区特定位置（MaxSize+2）提取保存的索引值
        N.Save_index = flash_union_buffer[MaxSize+2].uint32_type;

        // 标记索引已读取，避免重复读取
        Index_R_f = 1;

        // 清空缓冲区，为读取数据页做准备
        flash_buffer_clear();
    }

    // 检查目标Flash页是否有数据
    if(flash_check(0, N.Flash_page_index))
    {
        // 如果有数据，则读取该页到缓冲区
        flash_read_page_to_buffer(0, N.Flash_page_index);
    }
}



