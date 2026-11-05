/**
 ******************************************************************************
 * @file    base_axis.c
 * @author  RobotPilots 镖架电控
 * @brief   测试基地平移轴实现
 * @version V0.1
 * @date    2026-11-05
 ******************************************************************************
 * @attention
 *  见 base_axis.h。逻辑方向 = BASE_DIR_SIGN x 电机方向，安装方向只改配置。
 *
 * Copyright 2026 RobotPilots
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "base_axis.h"
#include <math.h>
#include <string.h>

/* Private functions ---------------------------------------------------------*/
static void Base_Axis_Reset_Pid(base_axis_t *axis)
{
    axis->spd_pid.integral  = 0.0f;
    axis->spd_pid.last_err  = 0.0f;
    axis->zero_pid.integral = 0.0f;
    axis->zero_pid.last_err = 0.0f;
}

static float Base_Axis_Speed_Loop(pid_ctrl_t *pid, float target_rpm, float meas_rpm, float out_max)
{
    pid->target  = target_rpm;
    pid->measure = meas_rpm;
    pid->err     = target_rpm - meas_rpm;
    pid->out_max = out_max;
    single_pid_ctrl(pid);
    return pid->out;
}

/**
 * @brief  按梯形速度生成轨迹参数
 */
static void Base_Axis_Plan(base_profile_t *prof, float p0, float p1, float vmax, float amax)
{
    float dist = Dart_Abs_F(p1 - p0);

    prof->p0  = p0;
    prof->p1  = p1;
    prof->dir = (p1 >= p0) ? 1.0f : -1.0f;
    prof->acc = amax;
    if (dist * amax <= vmax * vmax) {
        /* 三角形：达不到 vmax */
        prof->v_peak = sqrtf(dist * amax);
        prof->t_acc  = prof->v_peak / amax;
        prof->t_flat = 0.0f;
    } else {
        prof->v_peak = vmax;
        prof->t_acc  = vmax / amax;
        prof->t_flat = (dist - vmax * vmax / amax) / vmax;
    }
    prof->t_total = 2.0f * prof->t_acc + prof->t_flat;
}

/**
 * @brief  轨迹在 t 秒处的位置与速度
 */
static void Base_Axis_Profile_At(const base_profile_t *prof, float t, float *pos, float *vel)
{
    float dist = Dart_Abs_F(prof->p1 - prof->p0);
    float s;
    float v;
    float td;

    if (t <= 0.0f) {
        s = 0.0f;
        v = 0.0f;
    } else if (t < prof->t_acc) {
        s = 0.5f * prof->acc * t * t;
        v = prof->acc * t;
    } else if (t < prof->t_acc + prof->t_flat) {
        s = 0.5f * prof->acc * prof->t_acc * prof->t_acc + prof->v_peak * (t - prof->t_acc);
        v = prof->v_peak;
    } else if (t < prof->t_total) {
        td = prof->t_total - t;
        s = dist - 0.5f * prof->acc * td * td;
        v = prof->acc * td;
    } else {
        s = dist;
        v = 0.0f;
    }
    *pos = prof->p0 + prof->dir * s;
    *vel = prof->dir * v;
}

static void Base_Axis_Enter_Fault(base_axis_t *axis, base_axis_err_e err)
{
    axis->status       = BASE_AXIS_FAULT;
    axis->err          = err;
    axis->zeroed       = false;
    axis->home_pending = false;
    axis->output       = 0.0f;
    Base_Axis_Reset_Pid(axis);
}

static bool Base_Axis_Stall_Update(base_axis_t *axis, float speed_rpm, uint32_t now)
{
    bool cond = (Dart_Abs_F(axis->output) >= BASE_STALL_OUT_MIN) &&
                (Dart_Abs_F(speed_rpm) <= BASE_STALL_SPEED_MAX);

    if (!cond) {
        axis->stall_active = false;
        return false;
    }
    if (!axis->stall_active) {
        axis->stall_active = true;
        axis->stall_tick = now;
    }
    return Dart_Time_Elapsed(now, axis->stall_tick, BASE_STALL_HOLD_MS);
}

static bool Base_Axis_Settle_Update(base_axis_t *axis, uint32_t now)
{
    bool cond = (Dart_Abs_F(axis->target_mm - axis->pos_mm) <= BASE_ARRIVE_ERR_MM) &&
                (Dart_Abs_F(axis->speed_mm_s) <= BASE_ARRIVE_SPEED_MM_S);

    if (!cond) {
        axis->settle_active = false;
        return false;
    }
    if (!axis->settle_active) {
        axis->settle_active = true;
        axis->settle_tick = now;
    }
    return Dart_Time_Elapsed(now, axis->settle_tick, BASE_ARRIVE_HOLD_MS);
}

/**
 * @brief  位置模式：速度前馈 + 位置 P，转速 PI 输出
 */
static float Base_Axis_Track(base_axis_t *axis, float speed_rpm)
{
    float cmd_mm_s = axis->ref_v_mm_s + BASE_POS_KP * (axis->ref_mm - axis->pos_mm);

    return Base_Axis_Speed_Loop(&axis->spd_pid, cmd_mm_s * BASE_RPM_PER_MM_S,
                                speed_rpm, DART_3508_OUT_MAX);
}

/* Exported functions --------------------------------------------------------*/
void Base_Axis_Init(base_axis_t *axis)
{
    memset(axis, 0, sizeof(*axis));
    axis->status = BASE_AXIS_DISABLED;

    axis->spd_pid.kp           = BASE_SPD_KP;
    axis->spd_pid.ki           = BASE_SPD_KI;
    axis->spd_pid.integral_max = BASE_SPD_IMAX;
    axis->spd_pid.out_max      = DART_3508_OUT_MAX;

    axis->zero_pid.kp           = BASE_ZERO_SPD_KP;
    axis->zero_pid.ki           = BASE_ZERO_SPD_KI;
    axis->zero_pid.integral_max = BASE_ZERO_SPD_IMAX;
    axis->zero_pid.out_max      = BASE_ZERO_OUT_MAX;
}

/**
 * @brief  周期计算，ControlTask 每 1ms 调用；输出写入电机缓存
 */
void Base_Axis_Step(base_axis_t *axis, uint32_t now)
{
    bool online = Dart_Motor_Is_Online(DART_MOTOR_BASE);
    int32_t enc = Dart_Motor_Get_Encoder_Sum(DART_MOTOR_BASE);
    float speed_rpm = (float)BASE_DIR_SIGN * Dart_Motor_Get_Speed_Rpm(DART_MOTOR_BASE);
    float t;

    axis->pos_mm     = (float)BASE_DIR_SIGN * (float)(enc - axis->zero_ref) / BASE_COUNT_PER_MM;
    axis->speed_mm_s = speed_rpm / BASE_RPM_PER_MM_S;

    if (!online && (axis->status != BASE_AXIS_DISABLED) && (axis->status != BASE_AXIS_FAULT)) {
        Base_Axis_Enter_Fault(axis, BASE_AXIS_ERR_OFFLINE);
    }

    switch (axis->status) {
        case BASE_AXIS_ZEROING:
            axis->output = Base_Axis_Speed_Loop(&axis->zero_pid, BASE_ZERO_SPEED_RPM,
                                                speed_rpm, BASE_ZERO_OUT_MAX);
            if (Base_Axis_Stall_Update(axis, speed_rpm, now)) {
                /* 堵转处即 BASE_ZERO_STOP_MM */
                axis->zero_ref = enc - (int32_t)((float)BASE_DIR_SIGN * BASE_ZERO_STOP_MM * BASE_COUNT_PER_MM);
                axis->zeroed   = true;
                axis->pos_mm   = BASE_ZERO_STOP_MM;
                axis->status   = BASE_AXIS_HOLD;
                axis->target_mm = axis->pos_mm;
                axis->ref_mm   = axis->pos_mm;
                axis->ref_v_mm_s = 0.0f;
                axis->output   = 0.0f;
                Base_Axis_Reset_Pid(axis);
                axis->home_pending = true;
            } else if (Dart_Time_Elapsed(now, axis->phase_tick, BASE_ZERO_TIMEOUT_MS)) {
                Base_Axis_Enter_Fault(axis, BASE_AXIS_ERR_ZERO_TIMEOUT);
            }
            break;

        case BASE_AXIS_MOVING:
            t = (float)(now - axis->phase_tick) * 0.001f;
            Base_Axis_Profile_At(&axis->prof, t, &axis->ref_mm, &axis->ref_v_mm_s);
            axis->output = Base_Axis_Track(axis, speed_rpm);
            if ((t >= axis->prof.t_total) && Base_Axis_Settle_Update(axis, now)) {
                axis->status      = BASE_AXIS_HOLD;
                axis->arrive_tick = now;
            } else if (Dart_Time_Elapsed(now, axis->phase_tick, axis->timeout_ms)) {
                Base_Axis_Enter_Fault(axis, BASE_AXIS_ERR_MOVE_TIMEOUT);
            }
            break;

        case BASE_AXIS_HOLD:
            axis->ref_mm     = axis->target_mm;
            axis->ref_v_mm_s = 0.0f;
            axis->output     = Base_Axis_Track(axis, speed_rpm);
            if (axis->home_pending) {
                axis->home_pending = false;
                (void)Base_Axis_Request_Move(axis, 0.0f, BASE_SETUP_VMAX_MM_S, now);
            }
            break;

        case BASE_AXIS_DISABLED:
        case BASE_AXIS_FAULT:
        default:
            axis->output = 0.0f;
            Base_Axis_Reset_Pid(axis);
            break;
    }

    Dart_Motor_Set_Output(DART_MOTOR_BASE, (float)BASE_DIR_SIGN * axis->output);
}

/**
 * @brief  请求堵转找零；FAULT 下也可调用以恢复
 * @retval false 电机离线
 */
bool Base_Axis_Request_Zero(base_axis_t *axis, uint32_t now)
{
    if (!Dart_Motor_Is_Online(DART_MOTOR_BASE)) {
        return false;
    }
    axis->status       = BASE_AXIS_ZEROING;
    axis->err          = BASE_AXIS_ERR_NONE;
    axis->zeroed       = false;
    axis->home_pending = false;
    axis->phase_tick   = now;
    axis->stall_active = false;
    axis->zero_ref     = Dart_Motor_Get_Encoder_Sum(DART_MOTOR_BASE);
    Base_Axis_Reset_Pid(axis);
    return true;
}

/**
 * @brief  请求按梯形轨迹移动到 target_mm(限幅 ±BASE_RANGE_MM)
 * @param  vmax 本次最大速度 mm/s
 * @retval false 未找零、故障或找零中
 */
bool Base_Axis_Request_Move(base_axis_t *axis, float target_mm, float vmax, uint32_t now)
{
    if (!axis->zeroed || (axis->status == BASE_AXIS_FAULT) ||
        (axis->status == BASE_AXIS_ZEROING)) {
        return false;
    }
    axis->target_mm = Dart_Clamp_F(target_mm, -BASE_RANGE_MM, BASE_RANGE_MM);
    /* 从当前轨迹点起步，运动中改目标不会跳变 */
    Base_Axis_Plan(&axis->prof, axis->ref_mm, axis->target_mm, vmax, BASE_AMAX_MM_S2);
    axis->plan_ms       = (uint32_t)(axis->prof.t_total * 1000.0f + 0.5f);
    axis->timeout_ms    = axis->plan_ms + BASE_MOVE_TIMEOUT_EXTRA_MS;
    axis->phase_tick    = now;
    axis->settle_active = false;
    axis->status        = BASE_AXIS_MOVING;
    return true;
}

/**
 * @brief  停止：运动中原地保持；找零中放弃零点并无输出
 */
void Base_Axis_Stop(base_axis_t *axis)
{
    axis->home_pending = false;
    if (axis->status == BASE_AXIS_MOVING) {
        axis->target_mm = axis->pos_mm;
        axis->ref_mm    = axis->pos_mm;
        axis->status    = BASE_AXIS_HOLD;
    } else if (axis->status == BASE_AXIS_ZEROING) {
        axis->status = BASE_AXIS_DISABLED;
        axis->zeroed = false;
        axis->output = 0.0f;
    }
}

bool Base_Axis_Is_Busy(const base_axis_t *axis)
{
    return (axis->status == BASE_AXIS_ZEROING) || (axis->status == BASE_AXIS_MOVING) ||
           axis->home_pending;
}

/**
 * @brief  给定距离的梯形规划时长 ms(不含静止判定)
 */
uint32_t Base_Axis_Plan_Ms(float dist_mm, float vmax, float amax)
{
    base_profile_t prof;

    Base_Axis_Plan(&prof, 0.0f, Dart_Abs_F(dist_mm), vmax, amax);
    return (uint32_t)(prof.t_total * 1000.0f + 0.5f);
}
