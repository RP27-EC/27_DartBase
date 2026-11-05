/**
 ******************************************************************************
 * @file    config_uart.c
 * @author  RobotPilots 镖架电控
 * @brief   测试基地串口接收回调路由(覆盖 drv_uart.c 中的弱定义)
 * @version V0.1
 * @date    2026-11-05
 ******************************************************************************
 * @attention
 *  USART1 -> DL20 2.4G；USART3 -> DBUS(基地不接遥控，保留以免改动驱动层)；
 *  USART6 不使用。全部运行在串口空闲中断上下文，只做拷贝，不做控制。
 *
 * Copyright 2026 RobotPilots
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "config_uart.h"
#include "rc_sensor.h"
#include "drv_uart.h"
#include "base_link.h"

/* Exported functions --------------------------------------------------------*/
/**
 * @brief  USART1 接收：DL20
 * @param  rxBuf DMA 缓冲
 * @param  len   本次实际收到字节数
 */
void USART1_rxDataHandler(uint8_t *rxBuf, uint16_t len)
{
    Base_Link_Rx(rxBuf, len);
}

/**
 * @brief  USART3 接收：DBUS
 */
void USART3_rxDataHandler(uint8_t *rxBuf)
{
    rc_sensor.update(&rc_sensor, rxBuf);
    rc_sensor.check(&rc_sensor);
}

/**
 * @brief  USART6 接收：不使用
 */
void USART6_rxDataHandler(uint8_t *rxBuf, uint16_t len)
{
    (void)rxBuf;
    (void)len;
}
