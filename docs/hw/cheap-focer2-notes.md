# Cheap FOCer 2 (STM32F405) bring-up notes

Field notes from the v0.9 schematic to drive the F405 target (`oxifoc-f405`).

## Pin map (confirmed visually)

- PWM (DRV8301 6-PWM):
  - High: PA8 (INH_A / TIM1_CH1), PA9 (INH_B / TIM1_CH2), PA10 (INH_C / TIM1_CH3)
  - Low: PB13 (INL_A / TIM1_CH1N), PB14 (INL_B / TIM1_CH2N), PB15 (INL_C / TIM1_CH3N)
- EN_GATE: PB5 (active high)
- Fault: PB7 (DRV8301 nFAULT). OCTW not routed separately.
- DRV8301 SPI (SPI3): CS PC9, SCK PC10, MISO PC11, MOSI PC12
- Currents (ADC):
  - BR_SO1 PC0 (ADC123_IN10)
  - BR_SO2 PC1 (ADC123_IN11)
  - BR_SO3 PC2 (ADC123_IN12)
- VBUS: PC3 (ADC123_IN13) via 39k/2.2k
- Temps: PA3 (board NTC 10k/10k), PC4 (motor NTC on hall connector, 2.2k pull-up)
- Halls: PC6/PC7/PC8 (RC filter + 10k pull-ups)
- USB FS: PA11/PA12 (22 Ω series, ESD array); vbus_detection should remain disabled (PA9 is PWM).
- CAN: PB8 (RX), PB9 (TX)
- LEDs: PB0 (green), PB1 (red). Servo: PB6.

## Analog scaling (for firmware constants)

- Shunts: two 1 mΩ in parallel per phase ⇒ ~0.5 mΩ effective.
- DRV8301 internal amp: set gain to 20 V/V (matches op-amp stage on one phase).
- External TP2604 op-amp (one channel): gain ≈ 20 V/V (20k feedback / 1k input).
- VBUS divider (PC3): 39k / 2.2k ⇒ scale ≈ 18.7:1 (Vbus ≈ adc_volts * 18.7).
- Hall filters: 2.2 k series + 100 nF to ground, 10k pull-up → ~220 µs time constant (≈720 Hz).
- Board temp (PA3): 10k NTC + 10k to 3.3 V, C=2.2 µF (≈1.65 V at 25 °C).
- Motor temp (PC4): hall connector NTC with 2.2k pull-up (different curve vs board temp).

## Connector pinouts

### Hall/Encoder (P4)

1: 5 V  
2: TEMP_IN (motor NTC)  
3: HALL1  
4: HALL2  
5: HALL3  
6: GND  

Protection: SMF05 ESD at the connector, 100 Ω series at the header, then 2.2 k + 100 nF RC and 10 k pull-ups before MCU.

### CAN (J1)

2-pin: CANH/CANL via TJA1051.

### USB

Micro-B; 22 Ω series on D+/D-, SMF05 ESD. VBUS not routed to a dedicated pin (leave vbus_detection off).
