# MiyaFOC

MiyaFOC is a compact STM32F103C8 based FOC firmware for the mini wheel-leg robot hub motor driver. This version keeps the original smooth UART control behavior and adds a CAN interface for the main controller.

## Hardware

- MCU: STM32F103C8T6
- Encoder: MT6701 over SPI1
- PC debug UART: USART2, PA2 TX / PA3 RX, 115200 8N1
- CAN: CAN1, PA12 TX / PA11 RX, 1 Mbps classic CAN
- Motor control: current, speed, position and torque-style commands

## Build

Open `FOC.uvprojx` with Keil MDK and build target `FOC`.

The tested project copy is:

- Main working copy: `D:\github_prj\MiyaFoc`
- Backup copy: `D:\github_prj\MiyaFoc\MiyaFoc`

## UART Commands

USART2 is the direct debug/control port. Commands end with newline.

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

Notes:

- `SPEED:x` uses the original firmware speed command unit.
- `POSITION:x` uses degrees. `POSITION:0` means move to absolute zero, not stop.
- `TORQUE:x` uses the original firmware torque/current command value. Start from a very small value.
- `STOP` is the safe stop command.

## CAN Protocol

CAN is used by the main controller to command the FOC board. The current tested setup uses one FOC board per CAN bus, so both boards can keep node ID `1`.

- CAN bitrate: 1 Mbps
- Command standard ID: `0x211`
- Feedback standard ID: `0x291`
- Feedback period: 10 ms
- Boot test frames: disabled
- CAN command timeout: disabled by default, so commands latch like UART commands

Command frame, DLC 8:

| Byte | Meaning |
| --- | --- |
| 0 | mode: `0` stop, `1` torque/current, `2` speed, `3` position |
| 1 | node ID, default `1` |
| 2..5 | little-endian float command value |
| 6 | sequence / diagnostic byte |
| 7 | checksum, bitwise NOT of sum of bytes 0..6 |

CAN command value units are intentionally the same as UART:

- mode `1`: same value as `TORQUE:x`
- mode `2`: same value as `SPEED:x`
- mode `3`: same value as `POSITION:x`
- mode `0`: stop, value ignored

Feedback frame `0x291`, DLC 8:

| Byte | Meaning |
| --- | --- |
| 0..3 | little-endian float speed feedback |
| 4..7 | little-endian float continuous position feedback |

## Main Controller Test Commands

From the mini wheel-leg main controller PC port:

```text
foc speed 1 3
foc speed 1 10
foc pos 1 30
foc torque 1 0.01
foc stop 1
stop
```

Recommended first test sequence:

1. Power the FOC board and confirm UART telemetry is normal.
2. Use a USB2CAN tool to confirm feedback frame `0x291` appears every 10 ms.
3. Connect main controller CAN1 or CAN2 to the FOC board.
4. Send `foc speed 1 3`, then `foc speed 1 0`, then `foc stop 1`.
5. Test small position steps before large movements.

## Safety Notes

- Use `STOP` or `foc stop 1` to stop the motor. Do not use `POSITION:0` as a stop command.
- Keep the wheel unloaded for the first CAN test.
- Start torque/current commands from a very small value.
- If multiple FOC boards are placed on the same CAN bus, change node IDs and command/feedback IDs accordingly.
