/**
 ******************************************************************************
 * @file    dart_motor.c
 * @author  RobotPilots 镖架电控
 * @brief   测试基地单 3508 电机实例与 CAN 帧发送
 * @version V0.1
 * @date    2026-11-05
 ******************************************************************************
 * @attention
 *  1. 只有 ControlTask 调用 Set_Output/Send，保证输出单一写者。
 *  2. 读取 encoder_sum 时关中断，避免与 CAN 中断撕裂读取。
 *
 * Copyright 2026 RobotPilots
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "dart_motor.h"
#include "cmsis_os.h"

/* Private variables ---------------------------------------------------------*/
extern CAN_HandleTypeDef hcan1;

static Motor_RM_Born_Info_t s_born[DART_MOTOR_NUM] = {
    [DART_MOTOR_BASE] = {
        .order_correction = 1,
        .rxId  = (uint8_t)(BASE_CAN_ID_MOTOR - 0x201U),
        .stdId = DART_CAN_TX_ID_GROUP_LOW,
        .type  = _3508_Single,
        .hcan  = &hcan1,
    },
};

static Motor_RM_Rx_Info_t   s_rx[DART_MOTOR_NUM];
static Motor_RM_Tx_Info_t   s_tx[DART_MOTOR_NUM];
static Motor_RM_State_t     s_state[DART_MOTOR_NUM];
static Motor_RM_Ctrl_Info_t s_ctrl[DART_MOTOR_NUM];  /* 未使用模板内置闭环 */
static Motor_RM_Group_t     s_group_can1;            /* CAN1 0x200 组 */

/* Exported variables --------------------------------------------------------*/
Motor_RM_t g_dart_motor[DART_MOTOR_NUM];

/* Exported functions --------------------------------------------------------*/
/**
 * @brief  初始化电机对象与 CAN1 发送组
 * @note   在 CAN 过滤器启动前调用，避免中断访问未初始化对象
 */
void Dart_Motor_Init(void)
{
    Motor_RM_t *motor = &g_dart_motor[DART_MOTOR_BASE];

    motor->born_info = &s_born[DART_MOTOR_BASE];
    motor->rx_info   = &s_rx[DART_MOTOR_BASE];
    motor->tx_info   = &s_tx[DART_MOTOR_BASE];
    motor->state     = &s_state[DART_MOTOR_BASE];
    motor->ctrl      = &s_ctrl[DART_MOTOR_BASE];

    s_group_can1.motor[0] = motor;
    s_group_can1.motor[1] = NULL;
    s_group_can1.motor[2] = NULL;
    s_group_can1.motor[3] = NULL;
    s_group_can1.stdId    = DART_CAN_TX_ID_GROUP_LOW;
    s_group_can1.hcan     = &hcan1;

    /* 内部会调用 RM_Motor_Init 并把离线阈值写死为 100 */
    RM_Group_Motor_Init(&s_group_can1);

    /* NOTE: 初始化后覆盖阈值，并置为离线，收到首帧才转在线 */
    s_state[DART_MOTOR_BASE].offline_cnt_max = DART_MOTOR_OFFLINE_CNT;
    s_state[DART_MOTOR_BASE].offline_cnt     = DART_MOTOR_OFFLINE_CNT + 1U;
    s_state[DART_MOTOR_BASE].status          = DEV_OFFLINE;
    s_tx[DART_MOTOR_BASE].torque             = 0.0f;
}

/**
 * @brief  电机心跳，MonitorTask 每 1ms 调用
 */
void Dart_Motor_Heart_Beat(void)
{
    s_group_can1.group_heartbeat(&s_group_can1);
}

/**
 * @brief  查询电机是否在线
 */
bool Dart_Motor_Is_Online(dart_motor_id_e id)
{
    if (id >= DART_MOTOR_NUM) {
        return false;
    }
    return (s_state[id].status == DEV_ONLINE) &&
           (s_state[id].offline_cnt <= s_state[id].offline_cnt_max);
}

/**
 * @brief  读取转子累计编码器计数 count(8192/转)
 */
int32_t Dart_Motor_Get_Encoder_Sum(dart_motor_id_e id)
{
    int32_t encoder_sum;

    if (id >= DART_MOTOR_NUM) {
        return 0;
    }
    taskENTER_CRITICAL();
    encoder_sum = s_rx[id].encoder_sum;
    taskEXIT_CRITICAL();
    return encoder_sum;
}

/**
 * @brief  读取转子转速 rpm(电调原始反馈)
 */
float Dart_Motor_Get_Speed_Rpm(dart_motor_id_e id)
{
    if (id >= DART_MOTOR_NUM) {
        return 0.0f;
    }
    return (float)s_rx[id].encoder_speed;
}

/**
 * @brief  设置输出(限幅后写入发送缓存)
 * @param  out_raw 电调原始电流指令
 */
void Dart_Motor_Set_Output(dart_motor_id_e id, float out_raw)
{
    if (id >= DART_MOTOR_NUM) {
        return;
    }
    s_tx[id].torque = Dart_Clamp_F(out_raw, -DART_3508_OUT_MAX, DART_3508_OUT_MAX);
}

/**
 * @brief  输出清零(只改缓存，仍需 Send 发出)
 */
void Dart_Motor_Zero_Output(void)
{
    s_tx[DART_MOTOR_BASE].torque = 0.0f;
}

/**
 * @brief  发送 CAN1 0x200 控制帧；成功只表示进入邮箱
 */
void Dart_Motor_Send(void)
{
    s_group_can1.group_set_torque(&s_group_can1);
}
