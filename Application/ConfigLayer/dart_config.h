/**
 ******************************************************************************
 * @file    dart_config.h
 * @author  RobotPilots 镖架电控
 * @brief   测试基地(RP27_DartBase)编译开关、硬件映射与待标定参数
 * @version V0.1
 * @date    2026-11-05
 ******************************************************************************
 * @attention
 *  1. 本工程从 RP27_Dart 复制底层；文件名保持 dart_config.h，
 *     这样沿用的蜂鸣器、任务外壳和 main.c 不需要修改。内容只属于基地端。
 *  2. 硬件：C 板 + 1 个 3508(C620，CAN1 ID1=0x201) + 1 个 DL20(USART1)，无其它外设。
 *  3. 位置对外单位 mm，找零后中位为 0，范围 ±BASE_RANGE_MM；
 *     电机内部仍是转子累计计数 count(8192/转)和转子转速 rpm。
 *  4. 标注"待标定"的数值都是占位值，没有经过实物测量，上机前必须人工确认。
 *
 * Copyright 2026 RobotPilots
 ******************************************************************************
 */
#ifndef __DART_CONFIG_H
#define __DART_CONFIG_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/* 版本 ----------------------------------------------------------------------*/
#define DART_FW_VERSION              (0x00000100U)  /* 基地固件 0.1.0 */

/* 编译开关 ------------------------------------------------------------------*/
#define DART_IWDG_ENABLE             (0)    /* 看门狗，人工验证后再开 */
#define DART_BUZZER_ENABLE           (1)    /* 板载蜂鸣器提示音 */
#define BASE_AUTO_ZERO_ON_BOOT       (0)    /* 1:上电自动找零；默认等镖架下发 */

/* 任务周期 ------------------------------------------------------------------*/
#define DART_CTRL_PERIOD_MS          (1U)   /* 控制任务周期 */
#define DART_MONITOR_PERIOD_MS       (1U)   /* 监控任务周期 */
#define DART_COMMUNITY_PERIOD_MS     (5U)   /* 2.4G 轮询周期 */
#define DART_LED_PERIOD_MS           (50U)  /* 指示灯周期 */

/* CAN ------------------------------------------------------------------------*/
#define BASE_CAN_ID_MOTOR            (0x201U)  /* 3508 反馈 ID(电调 ID1) */
#define DART_CAN_TX_ID_GROUP_LOW     (0x200U)  /* ID1~4 控制帧 */
#define DART_MOTOR_OFFLINE_CNT       (100U)    /* 心跳计数上限，1ms/次 */
#define DART_3508_OUT_MAX            (16000.0f)/* 电调上限 16384 */

/* 蜂鸣器：TIM4 84MHz/(PSC+1)/(ARR+1=500) --------------------------------*/
#define BUZZER_PSC                   (83U)     /* 2kHz */
#define BUZZER_PWM                   (250U)    /* 占空比 50% */

/* 机构换算(待标定) --------------------------------------------------------*/
#define BASE_DIR_SIGN                (1)       /* 逻辑正向 ±1，待标定 */
#define BASE_GEAR_RATIO              (3591.0f / 187.0f) /* 3508 减速比 */
#define BASE_MM_PER_OUT_REV          (200.0f)  /* 输出轴一圈行程 mm，待标定 */
#define BASE_COUNT_PER_MM            (8192.0f * BASE_GEAR_RATIO / BASE_MM_PER_OUT_REV)
#define BASE_RPM_PER_MM_S            (60.0f * BASE_GEAR_RATIO / BASE_MM_PER_OUT_REV)

/* 行程 ------------------------------------------------------------------------*/
#define BASE_RANGE_MM                (280.0f)  /* 规则：相对中位 ±280 mm */
#define BASE_ZERO_STOP_MM            (-300.0f) /* 找零端机械限位位置 mm，待实测 */

/* 运动规划(梯形速度，待标定) --------------------------------------------*/
/* NOTE: 全程 560 mm 规划约 517 ms；200 mm/圈时转子约 8060 rpm，已接近 3508 上限 */
#define BASE_VMAX_MM_S               (1400.0f) /* 最大速度 mm/s */
#define BASE_AMAX_MM_S2              (12000.0f)/* 加减速度 mm/s² */
#define BASE_SETUP_VMAX_MM_S         (300.0f)  /* 非计时移动(设定/回中)限速 */
#define BASE_MOVE_TIMEOUT_EXTRA_MS   (500U)    /* 规划时间外允许的超时余量 */

/* 到位(静止)判定 ----------------------------------------------------------*/
#define BASE_ARRIVE_ERR_MM           (2.0f)    /* 位置误差 mm */
#define BASE_ARRIVE_SPEED_MM_S       (20.0f)   /* 速度 mm/s */
#define BASE_ARRIVE_HOLD_MS          (20U)     /* 持续 ms */

/* 找零(堵转) ---------------------------------------------------------------*/
#define BASE_ZERO_SPEED_RPM          (-1500.0f)/* 逻辑转子转速，朝负端 */
#define BASE_ZERO_OUT_MAX            (5000.0f) /* 找零电流限幅 raw */
#define BASE_STALL_OUT_MIN           (3000.0f) /* 堵转 |指令| 下限 raw */
#define BASE_STALL_SPEED_MAX         (30.0f)   /* 堵转 |转速| 上限 rpm */
#define BASE_STALL_HOLD_MS           (100U)    /* 堵转持续 ms */
#define BASE_ZERO_TIMEOUT_MS         (15000U)  /* 找零超时 ms */

/* 控制器(待调) ------------------------------------------------------------*/
#define BASE_POS_KP                  (15.0f)   /* 位置环 (mm/s)/mm */
#define BASE_SPD_KP                  (10.0f)   /* 速度环 raw/rpm */
#define BASE_SPD_KI                  (0.3f)    /* 速度环积分 */
#define BASE_SPD_IMAX                (20000.0f)/* 积分限幅 */
#define BASE_ZERO_SPD_KP             (1.0f)    /* 找零速度环 */
#define BASE_ZERO_SPD_KI             (0.1f)
/* NOTE: ki*IMAX 须大于 BASE_STALL_OUT_MIN，否则顶住限位时达不到堵转阈值 */
#define BASE_ZERO_SPD_IMAX           (40000.0f)

/* 靶型时序(2027 规则) ------------------------------------------------------*/
#define TARGET_END_DELAY_MS          (1200U)   /* 末端移动：出镖后开始换位 */
#define TARGET_MOVE_LIMIT_MS         (600U)    /* 换位到静止的时限 */
#define TARGET_PLAN_MARGIN_MS        (50U)     /* 规划需留出的静止判定余量 */
#define TARGET_MIN_DELTA_MM          (50)      /* 新位置与原位置最小差 mm，工程取值 */

/* 2.4G 链路 -----------------------------------------------------------------*/
#define BASE_LINK_LATENCY_MS         (20U)     /* DL20 单向延迟补偿 ms，待实测 */
#define BASE_STATUS_PERIOD_MS        (100U)    /* 周期状态上报 ms */
#define BASE_LINK_TIMEOUT_MS         (1000U)   /* 无命令/心跳判离线 ms，仅指示 */
#define BASE_TX_TIMEOUT_MS           (5U)      /* 单帧阻塞发送超时 ms */

/* Exported functions --------------------------------------------------------*/
static inline float Dart_Abs_F(float value)
{
    return (value < 0.0f) ? -value : value;
}

static inline float Dart_Clamp_F(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

/**
 * @brief  判断 now 是否已超过 start 后 span 毫秒(兼容计数回绕)
 */
static inline bool Dart_Time_Elapsed(uint32_t now, uint32_t start, uint32_t span)
{
    return (uint32_t)(now - start) >= span;
}

#endif /* __DART_CONFIG_H */
