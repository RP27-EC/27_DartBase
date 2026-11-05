/**
 ******************************************************************************
 * @file    target_fsm.h
 * @author  RobotPilots 镖架电控
 * @brief   测试基地靶型状态机(纯逻辑，不依赖 HAL)
 * @version V0.1
 * @date    2026-11-05
 ******************************************************************************
 * @attention
 *  按 2027 规则(规则手册 V1.0.0 第 111~112 页)模拟基地检测模块：
 *   - 固定：停在固定位，出镖不动。
 *   - 随机固定：收到 RELOCATE(开闸/命中后，由人工下发)才换到新的随机位置。
 *   - 随机移动：出镖即换位，600 ms 内完成移动并静止。
 *   - 末端移动：出镖 1.2 s 后开始换位，600 ms 内完成移动并静止。
 *  新位置：±280 mm 内均匀随机，且与出镖时位置相差至少 TARGET_MIN_DELTA_MM。
 *  规则只要求"不同于原位置"，最小差值是工程取值，避免"等于没动"。
 *
 *  出镖时刻 t0 = 基地收到命令时刻 - 命令 age_ms - BASE_LINK_LATENCY_MS。
 *  镖架以发出释放命令的时刻为 t0；与裁判"出镖检测"的时刻存在差值，需实测记录。
 *
 *  调用约定(都在 ControlTask)：先 Target_On_Cmd 处理本周期收到的命令，
 *  再 Target_Step 取得对轴的请求；请求交给轴后调用 Target_On_Move_Result 回报。
 *
 * Copyright 2026 RobotPilots
 ******************************************************************************
 */
#ifndef __TARGET_FSM_H
#define __TARGET_FSM_H

/* Includes ------------------------------------------------------------------*/
#include "dart_config.h"
#include "link24g_frame.h"

/* Exported types ------------------------------------------------------------*/
/**
 * @brief  轴状态快照(由 base_axis 提供)
 */
typedef struct {
    bool  zeroed;    /* 零点有效 */
    bool  busy;      /* 找零或运动中 */
    bool  fault;     /* 轴故障 */
    float pos_mm;    /* 当前位置 mm */
} target_axis_view_t;

/**
 * @brief  对轴的请求
 */
typedef enum {
    TARGET_REQ_NONE = 0,     /* 无 */
    TARGET_REQ_MOVE_TIMED,   /* 计时换位：最大速度规划 */
    TARGET_REQ_MOVE_SETUP,   /* 设定移动：低速，不计时 */
    TARGET_REQ_STOP,         /* 原地保持 */
    TARGET_REQ_ZERO,         /* 堵转找零 */
} target_req_e;

typedef struct {
    target_req_e req;        /* 请求类型 */
    int16_t      target_mm;  /* 目标 mm，仅移动有效 */
} target_out_t;

/**
 * @brief  状态机对象
 */
typedef struct {
    link24g_target_mode_e mode;     /* 当前靶型 */
    int16_t  fixed_mm;              /* 固定靶位置 mm */
    link24g_base_state_e state;     /* 运行状态 */
    uint32_t rng;                   /* xorshift32 状态，非 0 */
    target_req_e pending_req;       /* 待交给轴的请求 */
    int16_t  pending_mm;            /* 待请求目标 mm */
    int16_t  origin_mm;             /* 出镖时位置 mm */
    int16_t  target_mm;             /* 当前/最近目标 mm */
    bool     timed;                 /* 当前移动是否计时 */
    uint32_t start_at;              /* 应开始换位时刻 ms */
    uint32_t move_start;            /* 实际开始时刻 ms */
    uint16_t last_move_ms;          /* 上次换位开始到静止 ms */
    int16_t  start_lag_ms;          /* 上次实际开始-应开始 ms */
    uint16_t moves;                 /* 计时换位次数 */
    uint8_t  last_shot;             /* 最近出镖发序 */
    bool     last_in_time;          /* 上次换位在时限内 */
    bool     plan_over;             /* 上次规划超时限 */
    bool     sync_late;             /* 上次同步晚于开始时刻 */
    uint32_t sync_ignored;          /* 忙时被忽略的出镖同步 */
    uint32_t cmd_rejected;          /* 条件不满足被拒命令 */
} target_fsm_t;

/* Exported functions --------------------------------------------------------*/
void Target_Init(target_fsm_t *t, uint32_t seed);
void Target_On_Cmd(target_fsm_t *t, const link24g_cmd_t *c,
                   const target_axis_view_t *ax, uint32_t rx_tick);
target_out_t Target_Step(target_fsm_t *t, const target_axis_view_t *ax, uint32_t now);
void Target_On_Move_Result(target_fsm_t *t, bool accepted, uint32_t plan_ms);
int16_t Target_Random_Pos(target_fsm_t *t, int16_t cur_mm);
uint8_t Target_Flags(const target_fsm_t *t, const target_axis_view_t *ax);

#endif /* __TARGET_FSM_H */
