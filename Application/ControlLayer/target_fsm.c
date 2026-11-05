/**
 ******************************************************************************
 * @file    target_fsm.c
 * @author  RobotPilots 镖架电控
 * @brief   测试基地靶型状态机实现
 * @version V0.1
 * @date    2026-11-05
 ******************************************************************************
 * @attention
 *  状态：UNZEROED -> ZEROING -> IDLE；IDLE 下收到出镖同步 -> WAIT_START -> MOVING -> IDLE。
 *  STOP 在任何状态都取消待执行换位并原地保持；FAULT 只能由 ZERO 恢复。
 *
 * Copyright 2026 RobotPilots
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "target_fsm.h"

/* Private functions ---------------------------------------------------------*/
static int16_t Target_Round_Mm(float mm)
{
    return (int16_t)((mm >= 0.0f) ? (mm + 0.5f) : (mm - 0.5f));
}

static int16_t Target_Clamp_Mm(int16_t mm)
{
    const int16_t range = (int16_t)BASE_RANGE_MM;

    if (mm > range) {
        return range;
    }
    if (mm < -range) {
        return (int16_t)-range;
    }
    return mm;
}

static uint32_t Target_Rand(target_fsm_t *t)
{
    uint32_t x = t->rng;

    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    t->rng = x;
    return x;
}

static void Target_Request(target_fsm_t *t, target_req_e req, int16_t mm)
{
    t->pending_req = req;
    t->pending_mm  = mm;
}

/**
 * @brief  开始一次移动：timed=true 时计入换位统计
 */
static void Target_Begin_Move(target_fsm_t *t, int16_t mm, bool timed, uint32_t now)
{
    t->target_mm  = Target_Clamp_Mm(mm);
    t->timed      = timed;
    t->move_start = now;
    t->state      = LINK24G_BASE_MOVING;
    Target_Request(t, timed ? TARGET_REQ_MOVE_TIMED : TARGET_REQ_MOVE_SETUP, t->target_mm);
}

static bool Target_Can_Move(const target_fsm_t *t, const target_axis_view_t *ax)
{
    return ax->zeroed && !ax->fault && (t->state == LINK24G_BASE_IDLE);
}

static void Target_Handle_Sync(target_fsm_t *t, const link24g_cmd_t *c,
                               const target_axis_view_t *ax, uint32_t rx_tick)
{
    uint32_t t0;
    uint32_t delay;

    t->last_shot = (uint8_t)c->arg0;
    if ((t->mode != LINK24G_MODE_RANDOM_MOVE) && (t->mode != LINK24G_MODE_END_MOVE)) {
        return;   /* 固定/随机固定：出镖不换位 */
    }
    if (!Target_Can_Move(t, ax)) {
        t->sync_ignored++;
        return;
    }
    delay = (t->mode == LINK24G_MODE_END_MOVE) ? TARGET_END_DELAY_MS : 0U;
    t0 = rx_tick - (uint32_t)c->age_ms - BASE_LINK_LATENCY_MS;
    t->start_at  = t0 + delay;
    t->origin_mm = Target_Round_Mm(ax->pos_mm);
    t->target_mm = Target_Random_Pos(t, t->origin_mm);
    t->sync_late = (int32_t)(rx_tick - t->start_at) > 0;
    t->state     = LINK24G_BASE_WAIT_START;
}

/* Exported functions --------------------------------------------------------*/
/**
 * @brief  初始化
 * @param  seed 随机种子(芯片 UID 等)，0 时替换为固定非零值
 */
void Target_Init(target_fsm_t *t, uint32_t seed)
{
    memset(t, 0, sizeof(*t));
    t->mode  = LINK24G_MODE_FIXED;
    t->state = LINK24G_BASE_UNZEROED;
    t->rng   = (seed == 0U) ? 0x6D2B79F5U : seed;
}

/**
 * @brief  在 ±BASE_RANGE_MM 内均匀随机取新位置，且与 cur_mm 相差 >= TARGET_MIN_DELTA_MM
 * @note   拒绝采样 16 次仍不满足时取离 cur_mm 较远的一端，保证一定换位。
 */
int16_t Target_Random_Pos(target_fsm_t *t, int16_t cur_mm)
{
    const int32_t range = (int32_t)BASE_RANGE_MM;
    int32_t cand;
    int32_t diff;
    uint8_t i;

    for (i = 0U; i < 16U; i++) {
        cand = (int32_t)(Target_Rand(t) % (uint32_t)(2 * range + 1)) - range;
        diff = cand - (int32_t)cur_mm;
        if ((diff >= TARGET_MIN_DELTA_MM) || (diff <= -TARGET_MIN_DELTA_MM)) {
            return (int16_t)cand;
        }
    }
    return (cur_mm >= 0) ? (int16_t)-range : (int16_t)range;
}

/**
 * @brief  处理一条已去重的命令
 * @param  rx_tick 基地收到该命令的时刻 ms
 */
void Target_On_Cmd(target_fsm_t *t, const link24g_cmd_t *c,
                   const target_axis_view_t *ax, uint32_t rx_tick)
{
    switch ((link24g_cmd_e)c->cmd) {
        case LINK24G_CMD_STOP:
            Target_Request(t, TARGET_REQ_STOP, 0);
            if ((t->state == LINK24G_BASE_WAIT_START) || (t->state == LINK24G_BASE_MOVING)) {
                t->state = LINK24G_BASE_IDLE;
            }
            break;

        case LINK24G_CMD_ZERO:
            Target_Request(t, TARGET_REQ_ZERO, 0);
            t->state = LINK24G_BASE_ZEROING;
            break;

        case LINK24G_CMD_SET_MODE:
            if ((c->arg0 < 0) || (c->arg0 >= (int16_t)LINK24G_MODE_NUM)) {
                t->cmd_rejected++;
                break;
            }
            t->mode     = (link24g_target_mode_e)c->arg0;
            t->fixed_mm = Target_Clamp_Mm(c->arg1);
            if (t->state == LINK24G_BASE_WAIT_START) {
                t->state = LINK24G_BASE_IDLE;   /* 换靶型取消待执行换位 */
            }
            if ((t->mode == LINK24G_MODE_FIXED) && Target_Can_Move(t, ax)) {
                Target_Begin_Move(t, t->fixed_mm, false, rx_tick);
            }
            break;

        case LINK24G_CMD_GOTO_MM:
            if (Target_Can_Move(t, ax)) {
                Target_Begin_Move(t, c->arg0, false, rx_tick);
            } else {
                t->cmd_rejected++;
            }
            break;

        case LINK24G_CMD_RELOCATE:
            if ((t->mode == LINK24G_MODE_RANDOM_FIXED) && Target_Can_Move(t, ax)) {
                t->origin_mm = Target_Round_Mm(ax->pos_mm);
                t->start_at  = rx_tick;
                t->sync_late = false;
                Target_Begin_Move(t, Target_Random_Pos(t, t->origin_mm), true, rx_tick);
                t->start_lag_ms = 0;
            } else {
                t->cmd_rejected++;
            }
            break;

        case LINK24G_CMD_SYNC_SHOT:
            Target_Handle_Sync(t, c, ax, rx_tick);
            break;

        default:
            t->cmd_rejected++;
            break;
    }
}

/**
 * @brief  周期推进，返回本周期对轴的请求(每个请求只返回一次)
 */
target_out_t Target_Step(target_fsm_t *t, const target_axis_view_t *ax, uint32_t now)
{
    target_out_t out;
    int32_t lag;

    if (ax->fault && (t->state != LINK24G_BASE_ZEROING || t->pending_req != TARGET_REQ_ZERO)) {
        t->state = LINK24G_BASE_FAULT;
    }

    switch (t->state) {
        case LINK24G_BASE_UNZEROED:
            if (ax->zeroed) {
                t->state = LINK24G_BASE_IDLE;
            }
            break;

        case LINK24G_BASE_ZEROING:
            if ((t->pending_req == TARGET_REQ_NONE) && !ax->busy) {
                /* 找零中被 STOP 打断时零点无效，回到 UNZEROED */
                t->state = ax->zeroed ? LINK24G_BASE_IDLE : LINK24G_BASE_UNZEROED;
            }
            break;

        case LINK24G_BASE_WAIT_START:
            if ((int32_t)(now - t->start_at) >= 0) {
                lag = (int32_t)(now - t->start_at);
                Target_Begin_Move(t, t->target_mm, true, now);
                t->start_lag_ms = (lag > 32767) ? 32767 : (int16_t)lag;
            }
            break;

        case LINK24G_BASE_MOVING:
            if ((t->pending_req == TARGET_REQ_NONE) && !ax->busy) {
                if (t->timed) {
                    uint32_t used = now - t->move_start;

                    t->last_move_ms = (used > 0xFFFFU) ? 0xFFFFU : (uint16_t)used;
                    t->last_in_time = used <= TARGET_MOVE_LIMIT_MS;
                    t->moves++;
                }
                t->state = LINK24G_BASE_IDLE;
            }
            break;

        default:
            break;
    }

    out.req       = t->pending_req;
    out.target_mm = t->pending_mm;
    t->pending_req = TARGET_REQ_NONE;
    return out;
}

/**
 * @brief  轴对请求的处理结果
 * @param  accepted 轴是否接受
 * @param  plan_ms  计时换位的规划时长 ms
 */
void Target_On_Move_Result(target_fsm_t *t, bool accepted, uint32_t plan_ms)
{
    if (!accepted) {
        t->cmd_rejected++;
        if ((t->state == LINK24G_BASE_MOVING) || (t->state == LINK24G_BASE_WAIT_START)) {
            t->state = LINK24G_BASE_IDLE;
        }
        return;
    }
    if (t->timed) {
        t->plan_over = plan_ms > (TARGET_MOVE_LIMIT_MS - TARGET_PLAN_MARGIN_MS);
    }
}

/**
 * @brief  状态标志位(link24g_flag_e)
 */
uint8_t Target_Flags(const target_fsm_t *t, const target_axis_view_t *ax)
{
    uint8_t flags = 0U;

    if (ax->zeroed) {
        flags |= (uint8_t)LINK24G_FLAG_ZEROED;
    }
    if (t->last_in_time) {
        flags |= (uint8_t)LINK24G_FLAG_LAST_IN_TIME;
    }
    if (t->plan_over) {
        flags |= (uint8_t)LINK24G_FLAG_PLAN_OVER;
    }
    if (t->sync_late) {
        flags |= (uint8_t)LINK24G_FLAG_SYNC_LATE;
    }
    return flags;
}
