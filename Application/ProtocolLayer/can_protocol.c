/**
 ******************************************************************************
 * @file    can_protocol.c
 * @author  RobotPilots 镖架电控
 * @brief   测试基地 CAN 接收分发
 * @version V0.1
 * @date    2026-11-05
 ******************************************************************************
 * @attention
 *  CAN1：0x201 基地平移 3508；CAN2 不使用。中断上下文，只做反馈解析。
 *
 * Copyright 2026 RobotPilots
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "can_protocol.h"
#include "dart_motor.h"

/* Exported functions --------------------------------------------------------*/
/**
 * @brief  CAN1 反馈分发(中断上下文)
 */
void CAN1_rxDataHandler(uint32_t canId, uint8_t *rxBuf)
{
    if (canId == BASE_CAN_ID_MOTOR) {
        g_dart_motor[DART_MOTOR_BASE].rx(&g_dart_motor[DART_MOTOR_BASE], rxBuf);
    }
}

/**
 * @brief  CAN2 反馈分发：基地不使用
 */
void CAN2_rxDataHandler(uint32_t canId, uint8_t *rxBuf)
{
    (void)canId;
    (void)rxBuf;
}
