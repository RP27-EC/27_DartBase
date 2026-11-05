/**
 * host_check：基地靶型状态机(target_fsm.c)与平移轴(base_axis.c)纯逻辑检查。
 * 电机用一阶模型代替，只用于检查流程与判定，结果不代表实物性能或实机通过。
 */
#include <stdio.h>
#include <math.h>
#include "target_fsm.h"
#include "base_axis.h"

static int s_fail = 0;

#define CHECK(cond, msg)                                              \
    do {                                                              \
        if (cond) {                                                   \
            printf("  [OK]   %s\n", msg);                             \
        } else {                                                      \
            printf("  [FAIL] %s (line %d)\n", msg, __LINE__);         \
            s_fail++;                                                 \
        }                                                             \
    } while (0)

/* ---------- 电机与 PID 假实现 ---------- */
static bool  g_online = true;
static float g_x_mm;          /* 真实位置 mm(相对上电位置) */
static float g_rpm;           /* 转子转速(电机方向) */
static float g_out;           /* 最近输出 raw */
static float g_wall_mm = -300.0f; /* 负端机械限位(相对上电位置) */

bool Dart_Motor_Is_Online(dart_motor_id_e id) { (void)id; return g_online; }
int32_t Dart_Motor_Get_Encoder_Sum(dart_motor_id_e id) { (void)id; return (int32_t)lroundf(g_x_mm * BASE_COUNT_PER_MM * (float)BASE_DIR_SIGN); }
float Dart_Motor_Get_Speed_Rpm(dart_motor_id_e id) { (void)id; return g_rpm; }
void Dart_Motor_Set_Output(dart_motor_id_e id, float out_raw) { (void)id; g_out = out_raw; }

void single_pid_ctrl(pid_ctrl_t *pid)
{
    pid->integral += pid->err;
    if (pid->integral > pid->integral_max) pid->integral = pid->integral_max;
    if (pid->integral < -pid->integral_max) pid->integral = -pid->integral_max;
    pid->out = pid->kp * pid->err + pid->ki * pid->integral;
    if (pid->out > pid->out_max) pid->out = pid->out_max;
    if (pid->out < -pid->out_max) pid->out = -pid->out_max;
    pid->last_err = pid->err;
}

/* 一阶模型：稳态转速≈输出 raw，时间常数约 20 ms；碰到限位速度归零 */
static void Plant_Step(void)
{
    g_rpm += (g_out - g_rpm) * 0.05f;
    g_x_mm += (float)BASE_DIR_SIGN * g_rpm / BASE_RPM_PER_MM_S * 0.001f;
    if ((g_x_mm <= g_wall_mm) && ((float)BASE_DIR_SIGN * g_rpm < 0.0f)) {
        g_x_mm = g_wall_mm;
        g_rpm = 0.0f;
    }
}

/* ---------- 与 dart_app.c 相同的胶合逻辑 ---------- */
static base_axis_t  s_axis;
static target_fsm_t s_t;
static uint32_t     s_now;

static void View(target_axis_view_t *ax)
{
    ax->zeroed = s_axis.zeroed;
    ax->busy   = Base_Axis_Is_Busy(&s_axis);
    ax->fault  = (s_axis.status == BASE_AXIS_FAULT);
    ax->pos_mm = s_axis.pos_mm;
}

static target_req_e Cycle(const link24g_cmd_t *cmd, uint32_t rx_tick)
{
    target_axis_view_t ax;
    target_out_t out;
    bool ok;

    View(&ax);
    if (cmd != NULL) {
        Target_On_Cmd(&s_t, cmd, &ax, rx_tick);
    }
    out = Target_Step(&s_t, &ax, s_now);
    switch (out.req) {
        case TARGET_REQ_STOP: Base_Axis_Stop(&s_axis); break;
        case TARGET_REQ_ZERO:
            if (!Base_Axis_Request_Zero(&s_axis, s_now)) Target_On_Move_Result(&s_t, false, 0U);
            break;
        case TARGET_REQ_MOVE_TIMED:
        case TARGET_REQ_MOVE_SETUP:
            ok = Base_Axis_Request_Move(&s_axis, (float)out.target_mm,
                                       (out.req == TARGET_REQ_MOVE_TIMED) ? BASE_VMAX_MM_S : BASE_SETUP_VMAX_MM_S, s_now);
            Target_On_Move_Result(&s_t, ok, s_axis.plan_ms);
            break;
        default: break;
    }
    Base_Axis_Step(&s_axis, s_now);
    Plant_Step();
    s_now++;
    return out.req;
}

static void Run(uint32_t ms) { while (ms-- > 0U) (void)Cycle(NULL, 0U); }

static link24g_cmd_t Cmd(link24g_cmd_e cmd, int16_t a0, int16_t a1, uint16_t age)
{
    link24g_cmd_t c;
    memset(&c, 0, sizeof(c));
    c.cmd = (uint8_t)cmd; c.arg0 = a0; c.arg1 = a1; c.age_ms = age;
    return c;
}

static void Reset_All(void)
{
    g_online = true; g_x_mm = 0.0f; g_rpm = 0.0f; g_out = 0.0f; g_wall_mm = -300.0f;
    s_now = 1000U;
    Base_Axis_Init(&s_axis);
    Target_Init(&s_t, 12345U);
}

static void Zero_And_Home(void)
{
    link24g_cmd_t c = Cmd(LINK24G_CMD_ZERO, 0, 0, 0U);
    uint32_t guard;

    (void)Cycle(&c, s_now);
    for (guard = 0U; (guard < 30000U) && (s_t.state != LINK24G_BASE_IDLE); guard++) {
        (void)Cycle(NULL, 0U);
    }
}

/* ---------- 检查项 ---------- */
static void Check_Random(void)
{
    target_fsm_t t;
    int16_t cur, r, lo = 0, hi = 0;
    int i, bad = 0;

    printf("1. 随机新位置：范围、最小差值、覆盖两端\n");
    Target_Init(&t, 1U);
    for (i = 0; i < 20000; i++) {
        cur = (int16_t)((i % 561) - 280);
        r = Target_Random_Pos(&t, cur);
        if ((r < -280) || (r > 280) || (abs(r - cur) < TARGET_MIN_DELTA_MM)) bad++;
        if (r < lo) lo = r;
        if (r > hi) hi = r;
    }
    CHECK(bad == 0, "2 万次采样全部在 ±280 内且与原位置相差 >= 最小差值");
    CHECK((lo <= -275) && (hi >= 275), "采样能覆盖两端附近");
}

static void Check_Plan(void)
{
    uint32_t full = Base_Axis_Plan_Ms(560.0f, BASE_VMAX_MM_S, BASE_AMAX_MM_S2);
    uint32_t half = Base_Axis_Plan_Ms(280.0f, BASE_VMAX_MM_S, BASE_AMAX_MM_S2);

    printf("2. 梯形规划时长(不含静止判定)\n");
    printf("     全程 560 mm = %u ms，半程 280 mm = %u ms\n", (unsigned)full, (unsigned)half);
    CHECK(full <= TARGET_MOVE_LIMIT_MS - TARGET_PLAN_MARGIN_MS, "全程规划留出静止判定余量后仍在 600 ms 内");
    CHECK(Base_Axis_Plan_Ms(0.0f, BASE_VMAX_MM_S, BASE_AMAX_MM_S2) == 0U, "零距离规划为 0");
}

static void Check_Zero(void)
{
    link24g_cmd_t c;

    printf("3. 未找零拒绝移动、堵转找零后回中\n");
    Reset_All();
    c = Cmd(LINK24G_CMD_GOTO_MM, 100, 0, 0U);
    (void)Cycle(&c, s_now);
    CHECK((s_t.cmd_rejected == 1U) && (s_t.state == LINK24G_BASE_UNZEROED), "未找零时 GOTO 被拒");
    Zero_And_Home();
    CHECK(s_axis.zeroed && (s_t.state == LINK24G_BASE_IDLE), "找零完成进入 IDLE");
    CHECK(fabsf(s_axis.pos_mm) <= BASE_ARRIVE_ERR_MM, "找零后回到 0 mm");
    CHECK(fabsf((g_x_mm - g_wall_mm) - (0.0f - BASE_ZERO_STOP_MM)) <= 3.0f, "0 mm 距限位为 |BASE_ZERO_STOP_MM|");
}

static void Check_End_Move(void)
{
    link24g_cmd_t c;
    uint32_t rx, start_at, guard;
    target_req_e req = TARGET_REQ_NONE;

    printf("4. 末端移动：出镖 1.2 s 后开始，换位并静止\n");
    Reset_All();
    Zero_And_Home();
    c = Cmd(LINK24G_CMD_SET_MODE, LINK24G_MODE_END_MOVE, 0, 0U);
    (void)Cycle(&c, s_now);
    rx = s_now;
    c = Cmd(LINK24G_CMD_SYNC_SHOT, 2, 0, 40U);
    (void)Cycle(&c, rx);
    start_at = rx - 40U - BASE_LINK_LATENCY_MS + TARGET_END_DELAY_MS;
    CHECK((s_t.state == LINK24G_BASE_WAIT_START) && (s_t.start_at == start_at),
          "开始时刻 = 收到 - age - 延迟补偿 + 1200");
    CHECK(abs(s_t.target_mm - s_t.origin_mm) >= TARGET_MIN_DELTA_MM, "目标与出镖时位置不同");
    while (s_now < start_at) {
        req = Cycle(NULL, 0U);
        if (req != TARGET_REQ_NONE) break;
    }
    CHECK(req == TARGET_REQ_NONE, "开始时刻之前不动");
    req = Cycle(NULL, 0U);
    CHECK((req == TARGET_REQ_MOVE_TIMED) && (s_t.start_lag_ms == 0), "到点发出计时换位，无滞后");
    for (guard = 0U; (guard < 3000U) && (s_t.state != LINK24G_BASE_IDLE); guard++) {
        (void)Cycle(NULL, 0U);
    }
    printf("     目标 %d mm，规划 %u ms，开始到静止 %u ms(一阶模型)\n",
           s_t.target_mm, (unsigned)s_axis.plan_ms, (unsigned)s_t.last_move_ms);
    CHECK((s_t.state == LINK24G_BASE_IDLE) && (s_t.moves == 1U) &&
          (fabsf(s_axis.pos_mm - (float)s_t.target_mm) <= BASE_ARRIVE_ERR_MM), "到达目标并静止，计数 +1");
    CHECK(s_t.last_in_time == (s_t.last_move_ms <= TARGET_MOVE_LIMIT_MS), "时限标志与实测时长一致");
}

static void Check_Random_Move_And_Late(void)
{
    link24g_cmd_t c;
    target_req_e req;

    printf("5. 随机移动：出镖即换位；同步晚到时立即开始并记录滞后\n");
    Reset_All();
    Zero_And_Home();
    c = Cmd(LINK24G_CMD_SET_MODE, LINK24G_MODE_RANDOM_MOVE, 0, 0U);
    (void)Cycle(&c, s_now);
    c = Cmd(LINK24G_CMD_SYNC_SHOT, 1, 0, 100U);
    req = Cycle(&c, s_now);
    CHECK(s_t.sync_late, "age 100 ms 时开始时刻已过，标记 SYNC_LATE");
    CHECK((req == TARGET_REQ_MOVE_TIMED) && (s_t.start_lag_ms >= 100 + (int16_t)BASE_LINK_LATENCY_MS),
          "收到同步的同一周期立即开始，滞后 >= age + 延迟补偿");
    c = Cmd(LINK24G_CMD_SYNC_SHOT, 2, 0, 0U);
    (void)Cycle(&c, s_now);
    CHECK(s_t.sync_ignored == 1U, "换位中再次出镖被忽略并计数");
}

static void Check_Fixed_And_Random_Fixed(void)
{
    link24g_cmd_t c;
    uint32_t guard;
    target_req_e req;

    printf("6. 固定靶 / 随机固定靶\n");
    Reset_All();
    Zero_And_Home();
    c = Cmd(LINK24G_CMD_SET_MODE, LINK24G_MODE_FIXED, 150, 0U);
    req = Cycle(&c, s_now);
    CHECK(req == TARGET_REQ_MOVE_SETUP, "设为固定靶后低速移到固定位");
    for (guard = 0U; (guard < 5000U) && (s_t.state != LINK24G_BASE_IDLE); guard++) (void)Cycle(NULL, 0U);
    CHECK(fabsf(s_axis.pos_mm - 150.0f) <= BASE_ARRIVE_ERR_MM, "停在 150 mm");
    c = Cmd(LINK24G_CMD_SYNC_SHOT, 1, 0, 0U);
    (void)Cycle(&c, s_now);
    Run(2000U);
    CHECK((s_t.state == LINK24G_BASE_IDLE) && (s_t.moves == 0U), "固定靶出镖不换位");

    c = Cmd(LINK24G_CMD_SET_MODE, LINK24G_MODE_RANDOM_FIXED, 0, 0U);
    (void)Cycle(&c, s_now);
    c = Cmd(LINK24G_CMD_SYNC_SHOT, 2, 0, 0U);
    (void)Cycle(&c, s_now);
    Run(10U);
    CHECK(s_t.state == LINK24G_BASE_IDLE, "随机固定靶出镖不换位");
    c = Cmd(LINK24G_CMD_RELOCATE, 0, 0, 0U);
    req = Cycle(&c, s_now);
    CHECK(req == TARGET_REQ_MOVE_TIMED, "RELOCATE 立即随机换位");
    c = Cmd(LINK24G_CMD_SET_MODE, LINK24G_MODE_END_MOVE, 0, 0U);
    for (guard = 0U; (guard < 3000U) && (s_t.state != LINK24G_BASE_IDLE); guard++) (void)Cycle(NULL, 0U);
    (void)Cycle(&c, s_now);
    c = Cmd(LINK24G_CMD_RELOCATE, 0, 0, 0U);
    (void)Cycle(&c, s_now);
    CHECK(s_t.cmd_rejected == 1U, "非随机固定靶下 RELOCATE 被拒");
}

static void Check_Stop_And_Fault(void)
{
    link24g_cmd_t c;
    target_req_e req;

    printf("7. STOP 取消待执行换位；掉线进入故障，找零恢复\n");
    Reset_All();
    Zero_And_Home();
    c = Cmd(LINK24G_CMD_SET_MODE, LINK24G_MODE_END_MOVE, 0, 0U);
    (void)Cycle(&c, s_now);
    c = Cmd(LINK24G_CMD_SYNC_SHOT, 1, 0, 0U);
    (void)Cycle(&c, s_now);
    c = Cmd(LINK24G_CMD_STOP, 0, 0, 0U);
    req = Cycle(&c, s_now);
    CHECK((req == TARGET_REQ_STOP) && (s_t.state == LINK24G_BASE_IDLE), "STOP 后回到 IDLE");
    Run(1500U);
    CHECK(s_t.moves == 0U, "STOP 后到时刻也不换位");

    g_online = false;
    Run(5U);
    CHECK((s_t.state == LINK24G_BASE_FAULT) && !s_axis.zeroed && (s_axis.err == BASE_AXIS_ERR_OFFLINE),
          "电机离线进入 FAULT 并失去零点");
    c = Cmd(LINK24G_CMD_GOTO_MM, 0, 0, 0U);
    (void)Cycle(&c, s_now);
    CHECK(s_t.state == LINK24G_BASE_FAULT, "FAULT 下 GOTO 无效");
    g_online = true;
    Zero_And_Home();
    CHECK((s_t.state == LINK24G_BASE_IDLE) && s_axis.zeroed, "重新找零后恢复");
}

int main(void)
{
    Check_Random();
    Check_Plan();
    Check_Zero();
    Check_End_Move();
    Check_Random_Move_And_Late();
    Check_Fixed_And_Random_Fixed();
    Check_Stop_And_Fault();
    printf("\n%s：%d 项失败\n", (s_fail == 0) ? "全部通过" : "存在失败", s_fail);
    return (s_fail == 0) ? 0 : 1;
}
