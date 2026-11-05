/**
 ******************************************************************************
 * @file    base_link.c
 * @author  RobotPilots 镖架电控
 * @brief   测试基地端 2.4G 从机实现
 * @version V0.1
 * @date    2026-11-05
 ******************************************************************************
 * @attention
 *  见 base_link.h。命令队列为单生产者(CommunityTask)/单消费者(ControlTask)，
 *  状态快照为单写者(ControlTask)/单读者(CommunityTask)，均在临界区内拷贝。
 *
 * Copyright 2026 RobotPilots
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "base_link.h"
#include "cmsis_os.h"

/* Private macro -------------------------------------------------------------*/
#define BASE_LINK_RING_SIZE   (256U)   /* 接收环形缓冲 B，2 的幂 */
#define BASE_LINK_QUEUE_LEN   (8U)     /* 命令队列长度 */

/* Private typedef -----------------------------------------------------------*/
typedef struct {
    link24g_cmd_t cmd;       /* 命令 */
    uint32_t      rx_tick;   /* 接收时刻 ms */
} base_link_item_t;

/* Exported variables --------------------------------------------------------*/
base_link_view_t g_base_link;

/* Private variables ---------------------------------------------------------*/
extern UART_HandleTypeDef huart1;

static uint8_t           s_ring[BASE_LINK_RING_SIZE];
static volatile uint16_t s_ring_head;   /* 中断写 */
static volatile uint16_t s_ring_tail;   /* Poll 读 */
static link24g_parser_t  s_parser;

static base_link_item_t  s_queue[BASE_LINK_QUEUE_LEN];
static uint8_t           s_q_head;
static uint8_t           s_q_count;

static link24g_status_t  s_status;      /* ControlTask 写入的快照 */
static uint32_t          s_last_status_tx;

/* Private functions ---------------------------------------------------------*/
static void Base_Link_Send_Status(uint32_t now)
{
    link24g_status_t st;
    uint8_t  payload[LINK24G_STAT_PAYLOAD_LEN];
    uint8_t  frame[LINK24G_FRAME_MAX];
    uint16_t len;

    taskENTER_CRITICAL();
    st = s_status;
    taskEXIT_CRITICAL();
    st.ack_session = g_base_link.last_session;
    st.ack_seq     = g_base_link.last_seq;

    Link24g_Status_Pack(&st, payload);
    len = Link24g_Frame_Encode((uint8_t)LINK24G_MSG_STATUS, 0U, payload,
                               LINK24G_STAT_PAYLOAD_LEN, frame);
    (void)HAL_UART_Transmit(&huart1, frame, len, BASE_TX_TIMEOUT_MS);
    s_last_status_tx = now;
    g_base_link.tx_status++;
}

static bool Base_Link_Push(const link24g_cmd_t *cmd, uint32_t rx_tick)
{
    bool ok = false;
    uint8_t slot;

    taskENTER_CRITICAL();
    if (s_q_count < BASE_LINK_QUEUE_LEN) {
        slot = (uint8_t)((s_q_head + s_q_count) % BASE_LINK_QUEUE_LEN);
        s_queue[slot].cmd     = *cmd;
        s_queue[slot].rx_tick = rx_tick;
        s_q_count++;
        ok = true;
    }
    taskEXIT_CRITICAL();
    return ok;
}

static void Base_Link_Handle_Cmd(const link24g_cmd_t *cmd, uint32_t now)
{
    g_base_link.cmd_rx++;
    if (g_base_link.has_last && (cmd->session == g_base_link.last_session) &&
        (cmd->seq == g_base_link.last_seq)) {
        g_base_link.cmd_dup++;      /* 重发：只回执 */
        Base_Link_Send_Status(now);
        return;
    }
    if (!Base_Link_Push(cmd, now)) {
        g_base_link.cmd_queue_full++;
        return;                     /* 不回执，等镖架重发 */
    }
    g_base_link.last_session = cmd->session;
    g_base_link.last_seq     = cmd->seq;
    g_base_link.has_last     = true;
    Base_Link_Send_Status(now);
}

/* Exported functions --------------------------------------------------------*/
void Base_Link_Init(void)
{
    memset(&g_base_link, 0, sizeof(g_base_link));
    memset(&s_status, 0, sizeof(s_status));
    Link24g_Parser_Init(&s_parser);
    s_ring_head = 0U;
    s_ring_tail = 0U;
    s_q_head    = 0U;
    s_q_count   = 0U;
    s_last_status_tx = 0U;
}

/**
 * @brief  USART1 接收入口(空闲中断上下文，只拷贝)
 */
void Base_Link_Rx(const uint8_t *buf, uint16_t len)
{
    uint16_t i;
    uint16_t next;

    for (i = 0U; i < len; i++) {
        next = (uint16_t)((s_ring_head + 1U) & (BASE_LINK_RING_SIZE - 1U));
        if (next == s_ring_tail) {
            g_base_link.ring_overflow++;
            continue;
        }
        s_ring[s_ring_head] = buf[i];
        s_ring_head = next;
    }
}

/**
 * @brief  CommunityTask 周期调用：解析、去重、回执、周期上报
 */
void Base_Link_Poll(uint32_t now)
{
    link24g_cmd_t cmd;
    uint8_t byte;

    while (s_ring_tail != s_ring_head) {
        byte = s_ring[s_ring_tail];
        s_ring_tail = (uint16_t)((s_ring_tail + 1U) & (BASE_LINK_RING_SIZE - 1U));
        if (!Link24g_Parser_Feed(&s_parser, byte)) {
            continue;
        }
        g_base_link.rx_tick = now;
        if (Link24g_Cmd_Unpack(&s_parser, &cmd)) {
            Base_Link_Handle_Cmd(&cmd, now);
        }
        /* PING 只刷新 rx_tick */
    }
    g_base_link.crc_err = s_parser.crc_err;

    if (Dart_Time_Elapsed(now, s_last_status_tx, BASE_STATUS_PERIOD_MS)) {
        Base_Link_Send_Status(now);
    }
}

/**
 * @brief  ControlTask 取一条命令
 * @retval false 队列空
 */
bool Base_Link_Pop_Cmd(link24g_cmd_t *cmd, uint32_t *rx_tick)
{
    bool ok = false;

    taskENTER_CRITICAL();
    if (s_q_count > 0U) {
        *cmd     = s_queue[s_q_head].cmd;
        *rx_tick = s_queue[s_q_head].rx_tick;
        s_q_head = (uint8_t)((s_q_head + 1U) % BASE_LINK_QUEUE_LEN);
        s_q_count--;
        ok = true;
    }
    taskEXIT_CRITICAL();
    return ok;
}

/**
 * @brief  ControlTask 写入状态快照(ack 字段由本模块填写)
 */
void Base_Link_Set_Status(const link24g_status_t *st)
{
    taskENTER_CRITICAL();
    s_status = *st;
    taskEXIT_CRITICAL();
}

/**
 * @brief  BASE_LINK_TIMEOUT_MS 内是否收到过镖架的有效帧(命令或心跳)
 */
bool Base_Link_Is_Online(uint32_t now)
{
    return (g_base_link.rx_tick != 0U) &&
           !Dart_Time_Elapsed(now, g_base_link.rx_tick, BASE_LINK_TIMEOUT_MS);
}
