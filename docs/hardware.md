# Hardware Reference

## Board: B-G431B-ESC1

- **MCU**: STM32G431CB (Cortex-M4F with hardware FPU)
- **Debug Interface**: ST-Link (integrated)
- **Communication**: RTT (Real-Time Transfer) via probe-rs

## B-G431B-ESC1 Pinout (Oxifoc)

| Pin           | Signal               |
|--------------:|----------------------|
| VBAT          | 3V3                  |
| PC13/TAMP/RTC | TIM1_CH1N            |
| PC14          | CAN_TERM             |
| PC15          | N.C.                 |
| PF0/OSC-IN    | OSC 8MHz             |
| PF1/OSC-OUT   | OSC 8MHz             |
| PG10/NRST     | RESET                |
| PA0           | VBUS                 |
| PA1           | Curr_fdbk1_OPAmp+    |
| PA2           | OP1_OUT              |
| PA3           | Curr_fdbk1_OPAmp-    |
| PA4           | BEMF1                |
| PA5           | Curr_fdbk2_OPAmp-    |
| PA6           | OP2_OUT              |
| PA7           | Curr_fdbk2_OPAmp+    |
| PC4           | BEMF2                |
| PB0           | Curr_fdbk3_OPAmp+    |
| PB1           | TP3                  |
| PB2           | Curr_fdbk3_OPAmp-    |
| VREF+         | 3V3                  |
| VDDA          | 3V3                  |
| PB10          | N.C.                 |
| VDD4          | 3V3                  |
| PB11          | BEMF3                |
| PB12          | POTENTIOMETER        |
| PB13          | N.C.                 |
| PB14          | Temperature feedback |
| PB15          | TIM1_CH3N            |
| PC6           | STATUS               |
| PA8           | TIM1_CH1             |
| PA9           | TIM1_CH2             |
| PA10          | TIM1_CH3             |
| PA11          | CAN_RX               |
| PA12          | TIM1_CH2N            |
| VDD6          | 3V3                  |
| PA13          | SWDIO                |
| PA14          | SWCLK                |
| PA15          | PWM                  |
| PC10          | BUTTON               |
| PC11          | CAN_SHDN, TP2        |
| PB3           | USART2_TX            |
| PB4           | USART2_RX            |
| PB5           | GPIO_BEMF            |
| PB6           | A+/H1                |
| PB7           | B+/H2                |
| PB8           | Z+/H3                |
| PB9           | CAN_TX               |
| VDD8          | 3V3                  |

## Functional Groups

### PWM / Motor Drive
- **TIM1_CH1/CH1N** (PA8/PC13): Phase A high/low
- **TIM1_CH2/CH2N** (PA9/PA12): Phase B high/low
- **TIM1_CH3/CH3N** (PA10/PB15): Phase C high/low

### Current Sensing (via OPAMPs)
- **Phase A**: PA1 (+), PA3 (-), PA2 (output) → OPAMP1
- **Phase B**: PA7 (+), PA5 (-), PA6 (output) → OPAMP2
- **Phase C**: PB0 (+), PB2 (-) → OPAMP3

### Analog Inputs
- **PA0**: VBUS voltage sensing
- **PB14**: NTC temperature feedback
- **PB12**: Potentiometer input

### BEMF Sensing
- **PA4**: BEMF1
- **PC4**: BEMF2
- **PB11**: BEMF3
- **PB5**: GPIO_BEMF enable

### Hall / Encoder
- **PB6**: H1 / A+
- **PB7**: H2 / B+
- **PB8**: H3 / Z+

### Communication
- **PB3/PB4**: USART2 TX/RX (VCP via ST-Link)
- **PA11/PB9**: CAN RX/TX
- **PC14**: CAN termination
- **PC11**: CAN shutdown

### Debug / Status
- **PA13/PA14**: SWDIO/SWCLK
- **PC6**: Status LED
- **PC10**: User button
- **PB1**: Test point 3
- **PC11**: Test point 2

## Schematic

See [mb1419-g431cbu6-b01_schematic.pdf](mb1419-g431cbu6-b01_schematic.pdf) for the full board schematic.
