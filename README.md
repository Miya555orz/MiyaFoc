# MiyaFoc

STM32F103C8 FOC firmware with USART2 debug control and classic CAN control.

## Build

Open `FOC.uvprojx` with Keil MDK and build target `FOC`.

The project is self-contained. Source code is under `Core`, `Hardware`,
`Drivers`, and `Middlewares`. `FOC.ioc` is the matching STM32CubeMX project.

## Interfaces

- USART2: PA2 TX, PA3 RX, 115200 8N1
- CAN1: PA12 TX, PA11 RX, classic CAN at 1 Mbps
- Encoder: MT6701 on SPI1

## CAN protocol

Both FOC boards use node ID `1` because the left and right boards are connected
to separate buses on the robot controller.

### Command, standard ID 0x211, DLC 8

| Byte | Meaning |
| --- | --- |
| 0 | Mode: 0 stop, 1 current, 2 speed |
| 1 | Node ID, must be 1 |
| 2..5 | IEEE754 float, little-endian |
| 6 | Source wheel index, accepted for diagnostics |
| 7 | Bitwise NOT of the sum of bytes 0..6 |

Units:

- Current mode: amperes. The command is clamped to `MAX_CURRENT`.
- Speed mode: wheel revolutions per second.
- Stop mode: value is ignored and PWM is set to zero.

A valid CAN command owns the controller. If no further valid frame arrives for
100 ms, the firmware clears the control mode, resets the PID state, and sets all
PWM compares to zero. A later USART2 command can take ownership for bench tests.

### Feedback, standard ID 0x291, DLC 8

| Byte | Meaning |
| --- | --- |
| 0..3 | Wheel speed in revolutions per second, float little-endian |
| 4..7 | Continuous wheel position in radians, float little-endian |

Feedback is transmitted every 10 ms.

## USART2 commands

Commands end with `\n`:

```text
SPEED:10
POSITION:90
TORQUE:0.1
STOP
```

USART telemetry is limited to 20 Hz to avoid flooding low-cost USB-UART
adapters. USART speed and torque units retain the original firmware behavior:
speed is radians per second and torque is normalized current.
