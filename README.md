# MiyaFoc

MiyaFoc 是一份面向小串腿机器人轮毂电机驱动板的 STM32F103C8 FOC 固件。项目目标是让单块 FOC 板通过 CAN 接收主控板命令，完成轮毂电机的速度控制、电流/力矩测试、位置保持和实时状态反馈；同时保留 USART2 作为桌面调试入口，便于在 VOFA+ 中观察控制过程。

本工程当前服务于小串腿机器人底盘：主控板通过两路 CAN 分别连接左右轮毂 FOC 板，每块 FOC 板使用相同节点 ID，因为它们位于独立 CAN 总线上。

## 功能概览

- 三相无刷电机 FOC 控制，支持 SVPWM 输出。
- MT6701 磁编码器角度采集，维护连续轮端位置 `position_rad`。
- 电流环、速度环、位置保持控制。
- 串口调试命令：速度、位置、电流/力矩、停止。
- CAN 主控协议：停止、电流、速度、位置四种模式。
- CAN 超时保护：主控命令丢失后自动停机。
- CAN 周期反馈：速度和连续位置，供主控闭环和状态监控使用。
- VOFA+ 20 Hz 遥测输出，支持 CSV 曲线模式和带变量名文本模式。

## 工程信息

- MCU：STM32F103C8
- IDE：Keil MDK，打开 `FOC.uvprojx` 编译目标 `FOC`
- CubeMX 工程：`FOC.ioc`
- 主要代码：
  - `Core/Src/main.c`：初始化与主循环
  - `Core/Src/adc.c`：ADC 注入回调中运行电流环/控制步进
  - `Hardware/PID.c`：速度环、位置规划与位置保持
  - `Hardware/foc_can.c`：CAN 主控通信协议
  - `Core/Src/usart.c`：USART2 调试命令和遥测输出

## 硬件接口

| 接口 | 引脚 | 用途 |
| --- | --- | --- |
| USART2_TX | PA2 | VOFA+ 调试输出 |
| USART2_RX | PA3 | VOFA+ 调试命令输入 |
| CAN1_RX | PA11 | 主控 CAN 输入 |
| CAN1_TX | PA12 | 主控 CAN 输出 |
| SPI1 | PA5/PA6/PA7 | MT6701 编码器 |
| TIM1 PWM | 工程 `.ioc` 为准 | 三相桥 PWM |
| ADC1/ADC2 | 工程 `.ioc` 为准 | 两相电流采样 |

CAN 配置为 classic CAN 1 Mbps。USART2 配置为 `115200 8N1`。

## CAN 主控协议

### 控制帧

- 标准帧 ID：`0x211`
- DLC：8
- 周期建议：5 ms 至 20 ms。若主控连续控制，建议 10 ms 发送一次。
- 超时保护：100 ms 内没有收到有效控制帧时，FOC 板自动清空控制模式并关闭 PWM 输出。

| Byte | 含义 |
| --- | --- |
| 0 | 控制模式 |
| 1 | 节点 ID，当前固定为 `1` |
| 2..5 | `float` 小端数据 |
| 6 | 主控侧轮序号/诊断标记，当前仅参与校验 |
| 7 | 校验和，等于 `~sum(byte0..byte6)` 的低 8 位 |

控制模式：

| Mode | 名称 | value 单位 | 用途 |
| --- | --- | --- | --- |
| 0 | STOP | 忽略 | 立即停机，清 PID，PWM 置零 |
| 1 | CURRENT | A | 电流/力矩测试，内部按 `MAX_CURRENT` 限幅并归一化 |
| 2 | SPEED | rps | 速度闭环，正值前进，负值后退 |
| 3 | POSITION | rad | 位置保持/位置跳转，内部转换为角度进入位置环 |

### 反馈帧

- 标准帧 ID：`0x291`
- DLC：8
- 周期：10 ms

| Byte | 含义 |
| --- | --- |
| 0..3 | 当前轮速，`float` 小端，单位 rps |
| 4..7 | 连续轮端位置，`float` 小端，单位 rad |

### 主控功能映射建议

- 前进：左右 FOC 板均发送 `SPEED` 正值。
- 后退：左右 FOC 板均发送 `SPEED` 负值。
- 加速/减速：主控侧对目标 rps 做斜坡限制，再周期发送 `SPEED`。FOC 板会执行速度环，但整车加速度规划应放在主控。
- 转向：左右轮发送不同速度，例如左低右高实现左转，左高右低实现右转。
- 固定/刹停保持：先读取反馈 `position_rad`，再周期发送 `POSITION`，value 填当前或目标位置的弧度值。由于 FOC 板有 100 ms CAN 超时保护，保持状态也需要主控持续刷新。
- 急停：发送 `STOP`，或停止发送 CAN 控制帧等待 100 ms 超时保护。

### 控制帧示例

以节点 ID `1` 为例：

- 停止：`mode=0, value=0`
- 前进：`mode=2, value=5.0`，表示目标轮速 `+5 rps`
- 后退：`mode=2, value=-5.0`，表示目标轮速 `-5 rps`
- 位置固定：周期发送 `mode=3, value=当前 position_rad`
- 跳到目标位置：周期发送 `mode=3, value=目标 position_rad`

校验和计算：

```c
uint8_t checksum8(const uint8_t data[7])
{
    uint8_t sum = 0;
    for (int i = 0; i < 7; ++i) {
        sum += data[i];
    }
    return (uint8_t)(~sum);
}
```

## USART2 调试命令

命令以换行结束，大小写不敏感，`:` 和 `=` 都可以使用。

```text
SPEED:10
POSITION:90
TORQUE:0.1
STOP
LOG:CSV
LOG:TEXT
```

串口命令说明：

| 命令 | 单位 | 说明 |
| --- | --- | --- |
| `SPEED:x` | rad/s | 串口速度测试，沿用原工程内部单位 |
| `POSITION:x` | deg | 串口位置测试，角度单位 |
| `TORQUE:x` | 归一化电流 | 电流/力矩测试，建议小值开始 |
| `STOP` | 无 | 停机 |
| `LOG:CSV` | 无 | VOFA+ 数值曲线模式，默认模式 |
| `LOG:TEXT` | 无 | 带变量名文本输出，便于人工读数 |

默认 CSV 遥测变量顺序：

```text
speed_rad_s,target,enc_rad,motor_deg,elec_rad,pos_rad,iq,id,cmd
```

## 调试流程

1. 空载上电，确认无异常发热、三相桥无直通。
2. 使用 VOFA+ 连接 USART2，参数为 `115200 8N1`。
3. 发送 `SPEED:1`，确认电机低速平稳转动。
4. 发送 `SPEED:0` 或 `STOP`，确认能停机。
5. 发送小角度 `POSITION:30`、`POSITION:60`，确认位置环可保持。
6. 接入主控 CAN，先发送 `STOP`，再发送小速度 `SPEED` 帧。
7. 主控读取 `0x291` 反馈，确认速度和位置方向与整车定义一致。
8. 最后再测试前进、后退、加速、减速、转向和固定位置。

## 注意事项

- `CURRENT/TORQUE` 模式会直接给电流目标，务必从很小值开始测试。
- 主控侧应做速度斜坡，避免轮腿结构出现突变冲击。
- 两块 FOC 板在机器人上分别接 CAN1/CAN2，因此可以都保持节点 ID `1`。
- 如果两块 FOC 板挂在同一条 CAN 总线上，需要修改 `FOC_CAN_NODE_ID` 和对应 ID 分配。
- 位置模式使用连续位置 `position_rad`，主控应避免无意发送过大的位置跳变；如需长时间固定，请按 10 ms 左右周期持续发送位置帧。
