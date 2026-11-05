/**
 ******************************************************************************
 * @file    dart_motor.h
 * @author  RobotPilots 镖架电控
 * @brief   测试基地单 3508 电机实例与 CAN 帧发送
 * @version V0.1
 * @date    2026-11-05
 ******************************************************************************
 * @attention
 *  电机表：CAN1 0x201 基地平移 3508(C620 电调 ID1)，控制帧 0x200 的 0~1 字节。
 *  接口与 RP27_Dart 同名，便于沿用 main.c 与驱动层；类型 _3508_Single 下
 *  tx_info->torque 直接等于电调原始电流指令(raw)。
 *
 * Copyright 2026 RobotPilots
 ******************************************************************************
 */
#ifndef __DART_MOTOR_H
#define __DART_MOTOR_H

/* Includes ------------------------------------------------------------------*/
#include "RM_motor.h"
#include "dart_config.h"

/* Exported types ------------------------------------------------------------*/
/**
 * @brief  电机编号
 */
typedef enum {
    DART_MOTOR_BASE = 0,      /* 基地平移 3508 */
    DART_MOTOR_NUM,           /* 电机总数 */
} dart_motor_id_e;

/* Exported variables --------------------------------------------------------*/
extern Motor_RM_t g_dart_motor[DART_MOTOR_NUM];

/* Exported functions --------------------------------------------------------*/
void Dart_Motor_Init(void);
void Dart_Motor_Heart_Beat(void);
bool Dart_Motor_Is_Online(dart_motor_id_e id);
int32_t Dart_Motor_Get_Encoder_Sum(dart_motor_id_e id);
float Dart_Motor_Get_Speed_Rpm(dart_motor_id_e id);
void Dart_Motor_Set_Output(dart_motor_id_e id, float out_raw);
void Dart_Motor_Zero_Output(void);
void Dart_Motor_Send(void);

#endif /* __DART_MOTOR_H */
