/**
 ******************************************************************************
 * @file    dart_app.c
 * @author  RobotPilots 镖架电控
 * @brief   测试基地应用入口(沿用 RP27_Dart 的入口函数名，任务外壳不变)
 * @version V0.1
 * @date    2026-11-05
 ******************************************************************************
 * @attention
 *  控制周期顺序固定：取命令 -> 靶型状态机 -> 请求交给轴 -> 轴计算 -> CAN 输出 -> 状态快照。
 *  基地没有遥控和急停输入；停止只能由镖架下发 STOP，或直接断电。
 *  WARNING: 找零与换位速度很快(最大约 1.3 m/s)，测试时人员必须离开导轨行程范围。
 *
 * Copyright 2026 RobotPilots
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "dart_app.h"
#include "dart_motor.h"
#include "base_axis.h"
#include "target_fsm.h"
#include "base_link.h"
#include "buzzer.h"
#include "led.h"

#if DART_IWDG_ENABLE
#include "iwdg.h"
#endif

/* Exported variables --------------------------------------------------------*/
base_axis_t  g_base_axis;     /* 调试器可读 */
target_fsm_t g_target;        /* 调试器可读 */

/* Private functions ---------------------------------------------------------*/
static int16_t App_Round_Mm(float mm)
{
    return (int16_t)((mm >= 0.0f) ? (mm + 0.5f) : (mm - 0.5f));
}

static void App_Axis_View(target_axis_view_t *ax)
{
    ax->zeroed = g_base_axis.zeroed;
    ax->busy   = Base_Axis_Is_Busy(&g_base_axis);
    ax->fault  = (g_base_axis.status == BASE_AXIS_FAULT);
    ax->pos_mm = g_base_axis.pos_mm;
}

/**
 * @brief  把状态机请求交给轴
 */
static void App_Apply(const target_out_t *out, uint32_t now)
{
    bool ok;

    switch (out->req) {
        case TARGET_REQ_STOP:
            Base_Axis_Stop(&g_base_axis);
            break;
        case TARGET_REQ_ZERO:
            ok = Base_Axis_Request_Zero(&g_base_axis, now);
            if (!ok) {
                Target_On_Move_Result(&g_target, false, 0U);
            }
            break;
        case TARGET_REQ_MOVE_TIMED:
        case TARGET_REQ_MOVE_SETUP:
            ok = Base_Axis_Request_Move(&g_base_axis, (float)out->target_mm,
                                       (out->req == TARGET_REQ_MOVE_TIMED) ?
                                       BASE_VMAX_MM_S : BASE_SETUP_VMAX_MM_S, now);
            Target_On_Move_Result(&g_target, ok, g_base_axis.plan_ms);
            break;
        default:
            break;
    }
}

static void App_Update_Status(const target_axis_view_t *ax)
{
    link24g_status_t st;

    memset(&st, 0, sizeof(st));
    st.state        = (uint8_t)g_target.state;
    st.mode         = (uint8_t)g_target.mode;
    st.flags        = Target_Flags(&g_target, ax);
    st.fault        = (uint8_t)g_base_axis.err;
    st.pos_mm       = App_Round_Mm(g_base_axis.pos_mm);
    st.target_mm    = g_target.target_mm;
    st.last_move_ms = g_target.last_move_ms;
    st.start_lag_ms = g_target.start_lag_ms;
    st.moves        = g_target.moves;
    Base_Link_Set_Status(&st);
}

/* Exported functions --------------------------------------------------------*/
/**
 * @brief  模块初始化(调度器启动前)
 */
void Dart_App_Init(void)
{
    const uint32_t *uid = (const uint32_t *)UID_BASE;

    Base_Axis_Init(&g_base_axis);
    Target_Init(&g_target, uid[0] ^ uid[1] ^ uid[2] ^ SysTick->VAL);
    Base_Link_Init();
}

/**
 * @brief  控制周期，ControlTask 每 1ms 调用
 */
void Dart_App_Control_Step(uint32_t now)
{
    target_axis_view_t ax;
    target_out_t out;
    link24g_cmd_t cmd;
    uint32_t rx_tick;
#if BASE_AUTO_ZERO_ON_BOOT
    static bool s_auto_zero_done = false;   /* 上电自动找零已触发 */
#endif

    App_Axis_View(&ax);
#if BASE_AUTO_ZERO_ON_BOOT
    if (!s_auto_zero_done && Dart_Motor_Is_Online(DART_MOTOR_BASE)) {
        s_auto_zero_done = true;
        memset(&cmd, 0, sizeof(cmd));
        cmd.cmd = (uint8_t)LINK24G_CMD_ZERO;
        Target_On_Cmd(&g_target, &cmd, &ax, now);
    }
#endif
    while (Base_Link_Pop_Cmd(&cmd, &rx_tick)) {
        Target_On_Cmd(&g_target, &cmd, &ax, rx_tick);
    }
    out = Target_Step(&g_target, &ax, now);
    App_Apply(&out, now);

    Base_Axis_Step(&g_base_axis, now);
    Dart_Motor_Send();

    App_Axis_View(&ax);
    App_Update_Status(&ax);
}

/**
 * @brief  监控周期，MonitorTask 每 1ms 调用
 */
void Dart_App_Monitor_Step(uint32_t now)
{
    (void)now;
    Dart_Motor_Heart_Beat();
#if DART_IWDG_ENABLE
    (void)HAL_IWDG_Refresh(&hiwdg);
#endif
}

/**
 * @brief  2.4G 轮询，CommunityTask 每 DART_COMMUNITY_PERIOD_MS 调用
 */
void Dart_App_Community_Step(uint32_t now)
{
    Base_Link_Poll(now);
}

/**
 * @brief  指示灯与提示音，LedTask 每 50ms 调用
 *         蓝：镖架链路在线；绿：常亮=已找零，快闪=找零/换位中；红：常亮=故障
 */
void Dart_App_Led_Step(uint32_t now)
{
    static bool s_was_zeroed = false;   /* 上一周期零点有效 */
    static bool s_was_fault  = false;   /* 上一周期故障 */
    bool blink_fast = ((now / 125U) % 2U) == 0U;
    bool fault = (g_base_axis.status == BASE_AXIS_FAULT);
    bool busy  = Base_Axis_Is_Busy(&g_base_axis);

    if (!s_was_zeroed && g_base_axis.zeroed) {
        Buzzer_Play(BUZZER_TONE_ZERO_DONE);
    }
    if (!s_was_fault && fault) {
        Buzzer_Play(BUZZER_TONE_FAULT);
    }
    s_was_zeroed = g_base_axis.zeroed;
    s_was_fault  = fault;
    Buzzer_Step(now);

    if (Base_Link_Is_Online(now)) {
        LED_BLUE_ON;
    } else {
        LED_BLUE_OFF;
    }
    if ((g_base_axis.zeroed || busy) && (!busy || blink_fast)) {
        LED_GREEN_ON;
    } else {
        LED_GREEN_OFF;
    }
    if (fault) {
        LED_RED_ON;
    } else {
        LED_RED_OFF;
    }
}
