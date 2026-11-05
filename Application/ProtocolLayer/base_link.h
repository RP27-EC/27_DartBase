/**
 ******************************************************************************
 * @file    base_link.h
 * @author  RobotPilots 镖架电控
 * @brief   测试基地端 2.4G 从机(DL20，USART1 115200)
 * @version V0.1
 * @date    2026-11-05
 ******************************************************************************
 * @attention
 *  1. 帧格式见 link24g_frame.h，与 RP27_Dart 共用同一份。
 *  2. 线程：Base_Link_Rx 在 USART1 空闲中断中只拷贝；Base_Link_Poll 在
 *     CommunityTask 中解析、去重、回执和上报状态；命令经队列交给 ControlTask
 *     (Base_Link_Pop_Cmd)，状态由 ControlTask 写入快照(Base_Link_Set_Status)。
 *  3. 去重：(会话号, 序号) 与上一条已接受命令相同则只回执、不再入队。
 *     命令队列满时不回执，镖架会重发。
 *  4. 收到命令后立即回一帧状态作为回执，此外每 BASE_STATUS_PERIOD_MS 周期上报。
 *  5. 命令接收时刻取解析时刻，比串口实际到达最多晚一个轮询周期(5 ms)。
 *
 * Copyright 2026 RobotPilots
 ******************************************************************************
 */
#ifndef __BASE_LINK_H
#define __BASE_LINK_H

/* Includes ------------------------------------------------------------------*/
#include "dart_config.h"
#include "link24g_frame.h"

/* Exported types ------------------------------------------------------------*/
typedef struct {
    uint32_t rx_tick;        /* 最近收到有效帧 ms */
    uint32_t cmd_rx;         /* 收到命令帧(含重发) */
    uint32_t cmd_dup;        /* 重复命令(已执行过) */
    uint32_t cmd_queue_full; /* 队列满未回执 */
    uint32_t crc_err;        /* CRC 错误帧 */
    uint32_t ring_overflow;  /* 接收缓冲溢出字节 */
    uint32_t tx_status;      /* 已发状态帧 */
    uint8_t  last_session;   /* 最近接受命令的会话号 */
    uint8_t  last_seq;       /* 最近接受命令的序号 */
    bool     has_last;       /* 已接受过命令 */
} base_link_view_t;

/* Exported variables --------------------------------------------------------*/
extern base_link_view_t g_base_link;

/* Exported functions --------------------------------------------------------*/
void Base_Link_Init(void);
void Base_Link_Rx(const uint8_t *buf, uint16_t len);
void Base_Link_Poll(uint32_t now);
bool Base_Link_Pop_Cmd(link24g_cmd_t *cmd, uint32_t *rx_tick);
void Base_Link_Set_Status(const link24g_status_t *st);
bool Base_Link_Is_Online(uint32_t now);

#endif /* __BASE_LINK_H */
