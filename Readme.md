# RP27_DartBase：测试基地（移动靶）电控（C 板）

- 用途：队内测试时模拟 2027 规则中的基地检测模块，由镖架通过 2.4G 控制。**比赛不用。**
- 来源：底层（HAL、FreeRTOS、`RM_motor`、`PID`、`drv_uart`、蜂鸣器）复制自 `code\RP27_Dart`，镖架机构代码已删除。入口函数和 `dart_config.h` 沿用原文件名，内容只属于基地端。
- 状态：**纯编译通过，host_check 通过（只能说明逻辑符合预期）。尚未烧录，没有任何实机验证。** 所有标“待标定”的参数都要人工确认后才能上机。

## 1. 硬件

| 资源 | 引脚 / 参数 | 用途 |
|---|---|---|
| CAN1 1 Mbps | PD0 RX / PD1 TX | 3508 + C620，电调 ID1（反馈 0x201，控制帧 0x200 的 0~1 字节） |
| USART1 115200 8N1 | PA9 TX / PB7 RX（板上丝印 UART2，待对照 C 板手册确认） | DL20 无线串口：DL20 TXD→PB7，RXD→PA9 |
| LED | 板载 | 蓝：镖架链路在线；绿：常亮=已找零，快闪=找零或换位中；红：故障 |
| 蜂鸣器 | PD14 | 1 声：找零完成；3 声：故障 |

没有遥控、限位开关或急停输入。**能让基地停下的只有镖架下发 STOP，或者直接断电。**

DL20 配置：两只模块都设 115200、同一信道；一只 A 端、一只 B 端，哪只接镖架都可以。

## 2. Quick Start

需要：Keil MDK 5、ARMCC 5.06 update 7（Target `My_C`，芯片 STM32F407IGHx）。

```powershell
& 'G:\Keil 5\Core\UV4\UV4.exe' -b 'G:\STM32_ALL\RM\code\RP27_DartBase\MDK-ARM\My_C.uvprojx' -t My_C -j0 -o build.log
```

判断是否成功要看日志末尾的 `0 Error(s)`。烧录和镖架工程相同，用 uVision 按 F8（调试器配置为 J-Link）。

## 3. 代码结构

```
Application/
  ConfigLayer/   dart_config.h（基地全部参数）、config_uart.c（USART1→2.4G）
  ControlLayer/  dart_app（任务入口与胶合）、target_fsm（四种靶型，纯逻辑）
  ModuleLayer/   base_axis（堵转找零、梯形速度规划、静止判定）
  DeviceLayer/   dart_motor（单 3508）、buzzer、led、device
  ProtocolLayer/ base_link（2.4G 从机）、link24g_frame.h（与 RP27_Dart 共用，改动时两边同步）、can_protocol
host_check/      上位机纯逻辑检查，不进 Keil 工程
```

| 任务 | 周期 | 内容 |
|---|---|---|
| ControlTask | 1 ms | 取命令 → 靶型状态机 → 轴 → CAN 输出 → 写状态快照 |
| MonitorTask | 1 ms | 电机心跳 |
| CommunityTask | 5 ms | 2.4G 解析、去重、回执；每 100 ms 上报一次状态 |
| LedTask | 50 ms | 指示灯、提示音 |

## 4. 靶型行为（依据 2027 规则手册 V1.0.0 第 111~112 页）

| 靶型（`mode`） | 行为 |
|---|---|
| 0 固定 | 移到 `fixed_mm` 后保持不动，出镖也不动 |
| 1 随机固定 | 出镖不动。收到 RELOCATE（模拟开闸，或命中后由人工下发）才换到新的随机位置 |
| 2 随机移动 | 出镖即换位，要求 600 ms 内完成移动并静止 |
| 3 末端移动 | 出镖 1.2 s 后开始换位，要求 600 ms 内完成移动并静止 |

- **新位置**：在 ±280 mm 内均匀随机，且与出镖时的位置相差至少 `TARGET_MIN_DELTA_MM`（50 mm）。规则只要求“不同于原位置”，50 mm 是工程取值。
- **出镖时刻**：t0 = 基地收到命令时刻 − 命令 `age_ms` − `BASE_LINK_LATENCY_MS`。镖架以发出释放命令的时刻为 t0，与裁判“出镖检测”的时刻存在差值，需要实测。
- 同步到达时已经过了应开始的时刻：立即开始换位，并置 `SYNC_LATE` 标志；滞后时间记在 `start_lag_ms`。
- 换位进行中又收到出镖同步：忽略，计入 `sync_ignored`。
- 每次计时换位记录“开始到静止”的时长 `last_move_ms`，并标记是否在 600 ms 内完成。“静止”指误差 ≤ 2 mm、速度 ≤ 20 mm/s，并持续 20 ms。

## 5. 待标定参数（`dart_config.h`）

| 宏 | 占位值 | 说明 |
|---|---|---|
| `BASE_DIR_SIGN` | 1 | 让正方向与导轨坐标一致 |
| `BASE_MM_PER_OUT_REV` | 200 mm | 3508 输出轴每圈对应的平移量，由同步轮或丝杆决定 |
| `BASE_ZERO_STOP_MM` | −300 mm | 找零端机械限位相对中位的位置；必须比 −280 更靠外 |
| `BASE_VMAX_MM_S` / `BASE_AMAX_MM_S2` | 1400 / 12000 | 全程 560 mm 的规划时长约 517 ms。按 200 mm/圈换算，转子约 8060 rpm，已接近 3508 上限 |
| `BASE_POS_KP`、`BASE_SPD_KP/KI` | 15 / 10 / 0.3 | 需要上机调 |
| `BASE_ZERO_SPEED_RPM`、`BASE_ZERO_OUT_MAX`、`BASE_STALL_*` | −1500 rpm / 5000 / 3000 | 找零力度。须满足 `ki × IMAX > STALL_OUT_MIN`，否则撞到限位后达不到堵转判定 |
| `BASE_LINK_LATENCY_MS` | 20 ms | DL20 单向延迟补偿，需要实测 |

> [!warning]
> 0.6 s 内走完 560 mm 需要约 1.4 m/s 的速度和 1.2 g 的加速度，能否达到取决于机械（传动比、负载质量、导轨）。规划时长超过 550 ms 时，状态会置 `PLAN_OVER` 标志；实际是否达标看 `last_move_ms`。

## 6. 上机步骤（人工）

1. 先**不装同步带**（让电机空转）上电，确认红灯不亮（电机在线、无故障）、蓝灯亮（收到镖架心跳）。
2. 在镖架端用调试器写 `g_link24g_ctrl.req = 2`（ZERO）。基地朝负端低速堵转找零，听到“嘀”一声后回到 0 mm。
3. `req = 3`（GOTO），把 `goto_mm` 依次设为 ±100、±280，核对实际位置，标定 `BASE_MM_PER_OUT_REV`。
4. 设 `mode = 2`（随机移动），`req = 1`；然后 `req = 6`（SYNC_TEST）模拟出镖，记录 `last_move_ms`。从低速开始，逐步把 `VMAX/AMAX` 提高到目标值。
5. 设 `mode = 3`（末端移动），重复第 4 步，并核对开始时刻是否约为同步后 1.2 s。
6. 联调发射：镖架正常发射时会自动发出同步。记录每一发的释放时刻、基地开始换位的时刻和静止时刻。

## 7. host_check

在 `code\RP27_DartBase` 目录下执行：

```powershell
gcc -std=c99 -Wall -I host_check/stubs -I Application/ConfigLayer -I Application/ControlLayer -I Application/ModuleLayer -I Application/ProtocolLayer host_check/check_target.c Application/ControlLayer/target_fsm.c Application/ModuleLayer/base_axis.c -lm -o host_check/check_target.exe
```

```powershell
.\host_check\check_target.exe
```

检查内容：随机位置的范围和最小差值、梯形规划时长、找零与回中、四种靶型的时序、STOP、离线故障与恢复。电机用一阶模型代替，**结果不代表实物性能**。生成的 exe 是临时文件，用完后删除。
