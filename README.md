# MiyaFOC

MiyaFOC 是用于小串腿机器人轮毂电机的 STM32F103C8 FOC 驱动板固件。当前版本保留原串口控制时丝滑稳定的参数和控制行为，并加入 CAN 通信，供主控板通过 CAN 控制左右轮毂电机。

主控板通过两路 CAN 分别连接左右轮毂 FOC 板。因为两块 FOC 板位于独立 CAN 总线上，所以它们可以使用相同节点 ID。

## 硬件接口

- MCU：STM32F103C8T6
- 编码器：MT6701，SPI1
- 调试串口：USART2，PA2 TX / PA3 RX，115200 8N1
- CAN：CAN1，PA12 TX / PA11 RX，经典 CAN，1 Mbps
- 控制模式：电流/力矩、速度、位置、停机

## 功能概览

- 三相无刷电机 FOC 控制，支持 SVPWM 输出。
- MT6701 磁编码器角度采集，维护连续轮端位置。
- 电流环、速度环、位置保持控制。
- 串口调试命令：速度、位置、电流/力矩、停止。
- CAN 主控协议：停止、电流/力矩、速度、位置四种模式。
- CAN 周期反馈：速度和连续位置，供主控闭环和状态监控使用。
- VOFA+ 遥测输出，支持 CSV 曲线模式和带变量名文本模式。

## 工程说明

- IDE：Keil MDK，打开 `FOC.uvprojx` 编译目标 `FOC`
- CubeMX 工程：`FOC.ioc`
- 当前测试通过的主工程：`D:\github_prj\MiyaFoc`
- 当前备份工程：`D:\github_prj\MiyaFoc\MiyaFoc`
- 主要代码：
  - `Core/Src/main.c`：初始化与主循环
  - `Core/Src/adc.c`：ADC 注入回调中运行电流环/控制步进
  - `Hardware/PID.c`：速度环、位置规划与位置保持
  - `Hardware/foc_can.c`：CAN 主控通信协议
  - `Core/Src/usart.c`：USART2 调试命令和遥测输出

## 串口命令

USART2 用于直接调试和控制。命令以换行结尾。
命令大小写不敏感，`:` 和 `=` 都可以使用。

```text
SPEED:10
POSITION:90
TORQUE:0.01
STOP
CANSTAT
CANPROFILE:0
CANLOOP:1
LOG:CSV
LOG:TEXT
```

说明：

- `SPEED:x` 使用原固件速度命令单位。
- `POSITION:x` 使用角度制，单位为度。`POSITION:0` 表示转到绝对零位，不是停机。
- `TORQUE:x` 使用原固件力矩/电流命令值，测试时从很小的值开始。
- `STOP` 是安全停机命令。

默认 CSV 遥测变量顺序：

```text
speed_rad_s,target,enc_rad,motor_deg,elec_rad,pos_rad,iq,id,cmd
```

## CAN 协议

CAN 用于主控板控制 FOC 板。当前实测配置为每条 CAN 总线连接一块 FOC 板，因此左右 FOC 板均可保持节点 ID `1`。

- CAN 波特率：1 Mbps
- 命令标准帧 ID：`0x211`
- 反馈标准帧 ID：`0x291`
- 反馈周期：10 ms
- 上电测试帧：默认关闭
- CAN 指令超时停机：默认关闭，因此 CAN 指令会像串口指令一样保持目标值

命令帧，DLC 8：

| 字节 | 含义 |
| --- | --- |
| 0 | 模式：`0` 停机，`1` 力矩/电流，`2` 速度，`3` 位置 |
| 1 | 节点 ID，默认 `1` |
| 2..5 | 小端 float 命令值 |
| 6 | 序号/诊断字节 |
| 7 | 校验和，对字节 0..6 求和后按位取反 |

CAN 命令值的单位刻意与串口命令保持一致：

- 模式 `1`：与 `TORQUE:x` 相同
- 模式 `2`：与 `SPEED:x` 相同
- 模式 `3`：与 `POSITION:x` 相同
- 模式 `0`：停机，忽略命令值

反馈帧 `0x291`，DLC 8：

| 字节 | 含义 |
| --- | --- |
| 0..3 | 小端 float 速度反馈 |
| 4..7 | 小端 float 连续位置反馈 |

## 主控测试命令

从小串腿主控板电脑调试串口发送：

```text
foc speed 1 3
foc speed 1 10
foc pos 1 30
foc torque 1 0.01
foc stop 1
stop
```

## 硬件调试流程

1. 空载上电，确认无异常发热，确认心跳/通信状态正常。
2. 使用 VOFA+ 连接 USART2，参数为 `115200 8N1`。
3. 发送 `SPEED:1`，确认电机低速平稳转动。
4. 发送 `SPEED:0` 或 `STOP`，确认能停机。
5. 发送小角度 `POSITION:30`、`POSITION:60`，确认位置环可保持。
6. 使用 USB2CAN 确认反馈帧 `0x291` 每 10 ms 出现。
7. 接入主控 CAN，先发送 `foc stop 1`，再发送小速度 `foc speed 1 3`。
8. 最后再测试前进、后退、加速、减速、转向和固定位置。

## 注意事项

- 使用 `STOP` 或 `foc stop 1` 停机，不要把 `POSITION:0` 当作停机命令。
- 第一次 CAN 测试建议空载。
- 力矩/电流命令必须从很小的值开始。
- 如果多块 FOC 板接在同一条 CAN 总线上，需要修改节点 ID 以及对应的命令/反馈 ID。
