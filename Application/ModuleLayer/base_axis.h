/**
 ******************************************************************************
 * @file    base_axis.h
 * @author  RobotPilots 镖架电控
 * @brief   测试基地平移轴：堵转找零、梯形速度规划、到位(静止)判定
 * @version V0.1
 * @date    2026-11-05
 ******************************************************************************
 * @attention
 *  1. 位置 pos_mm = BASE_DIR_SIGN * (encoder_sum - zero_ref) / BASE_COUNT_PER_MM。
 *  2. 找零：转速环朝负端堵转，堵转处记为 BASE_ZERO_STOP_MM，随后低速回到 0 mm。
 *  3. 移动：按 vmax/amax 生成梯形速度轨迹，控制量 = 速度前馈 + 位置误差 P，
 *     再经转速 PI 输出电流。规划结束后误差与速度同时满足并持续
 *     BASE_ARRIVE_HOLD_MS 才判到位(静止)。
 *  4. 超时(规划时长 + BASE_MOVE_TIMEOUT_EXTRA_MS)、找零超时、电机离线进入 FAULT，
 *     FAULT 输出为零且失去零点，只能重新找零恢复。
 *  5. 只在 ControlTask 中调用(单一写者)。
 *
 * Copyright 2026 RobotPilots
 ******************************************************************************
 */
#ifndef __BASE_AXIS_H
#define __BASE_AXIS_H

/* Includes ------------------------------------------------------------------*/
#include "dart_config.h"
#include "dart_motor.h"
#include "pid.h"

/* Exported types ------------------------------------------------------------*/
typedef enum {
    BASE_AXIS_DISABLED = 0,  /* 无输出(未找零) */
    BASE_AXIS_ZEROING,       /* 堵转找零 */
    BASE_AXIS_HOLD,          /* 静止保持 */
    BASE_AXIS_MOVING,        /* 按轨迹运动 */
    BASE_AXIS_FAULT,         /* 故障，见 err */
} base_axis_status_e;

typedef enum {
    BASE_AXIS_ERR_NONE = 0,      /* 无 */
    BASE_AXIS_ERR_ZERO_TIMEOUT,  /* 找零超时 */
    BASE_AXIS_ERR_MOVE_TIMEOUT,  /* 运动超时 */
    BASE_AXIS_ERR_OFFLINE,       /* 电机离线 */
} base_axis_err_e;

/**
 * @brief  梯形轨迹
 */
typedef struct {
    float    p0;        /* 起点 mm */
    float    p1;        /* 终点 mm */
    float    dir;       /* 方向 ±1 */
    float    v_peak;    /* 峰值速度 mm/s */
    float    acc;       /* 加速度 mm/s² */
    float    t_acc;     /* 加速段 s */
    float    t_flat;    /* 匀速段 s */
    float    t_total;   /* 总时长 s */
} base_profile_t;

typedef struct {
    base_axis_status_e status;    /* 状态 */
    base_axis_err_e    err;       /* 故障原因 */
    bool     zeroed;              /* 零点有效 */
    bool     home_pending;        /* 找零后回中待执行 */
    int32_t  zero_ref;            /* 零点处原始计数 count */
    float    pos_mm;              /* 位置 mm */
    float    speed_mm_s;          /* 速度 mm/s */
    float    target_mm;           /* 目标 mm */
    float    ref_mm;              /* 当前轨迹位置 mm */
    float    ref_v_mm_s;          /* 当前轨迹速度 mm/s */
    float    output;              /* 逻辑输出 raw */
    base_profile_t prof;          /* 当前轨迹 */
    uint32_t phase_tick;          /* 找零/运动起始 ms */
    uint32_t timeout_ms;          /* 本次超时 ms */
    uint32_t plan_ms;             /* 本次规划时长 ms */
    uint32_t arrive_tick;         /* 最近一次静止时刻 ms */
    uint32_t settle_tick;         /* 到位条件起始 ms */
    bool     settle_active;       /* 到位条件成立中 */
    uint32_t stall_tick;          /* 堵转条件起始 ms */
    bool     stall_active;        /* 堵转条件成立中 */
    pid_ctrl_t spd_pid;           /* 位置模式速度环 */
    pid_ctrl_t zero_pid;          /* 找零速度环 */
} base_axis_t;

/* Exported functions --------------------------------------------------------*/
void Base_Axis_Init(base_axis_t *axis);
void Base_Axis_Step(base_axis_t *axis, uint32_t now);
bool Base_Axis_Request_Zero(base_axis_t *axis, uint32_t now);
bool Base_Axis_Request_Move(base_axis_t *axis, float target_mm, float vmax, uint32_t now);
void Base_Axis_Stop(base_axis_t *axis);
bool Base_Axis_Is_Busy(const base_axis_t *axis);
uint32_t Base_Axis_Plan_Ms(float dist_mm, float vmax, float amax);

#endif /* __BASE_AXIS_H */
