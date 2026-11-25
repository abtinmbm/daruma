# Wiring and pin map

**DRAFT.** To be confirmed against the board in hand during bring-up. This file is the single source of truth for pins; firmware must match it.

## ESP32-S3-DevKitC-1 pin map

| GPIO | Signal | Connected to | Notes |
|---|---|---|---|
| 12 | SPI SCK | IMU SCL/SCK | SPI2 via GPIO matrix |
| 11 | SPI MOSI | IMU SDA/SDI | |
| 13 | SPI MISO | IMU DO/SDO | |
| 10 | SPI CS | IMU CS | |
| 9 | IMU INT1 | IMU INT1 | data-ready interrupt, wakes the control task |
| 4 | Left encoder A | left motor encoder A | PCNT unit 0, 3.3 V |
| 5 | Left encoder B | left motor encoder B | |
| 6 | Right encoder A | right motor encoder A | PCNT unit 1, 3.3 V |
| 7 | Right encoder B | right motor encoder B | |
| 15 | M1A | left DRV8871 IN1 | left motor PWM |
| 16 | M1B | left DRV8871 IN2 | |
| 17 | M2A | right DRV8871 IN1 | right motor PWM |
| 18 | M2B | right DRV8871 IN2 | |
| 1 | Battery sense | divider midpoint | ADC1 channel 0 |
| 21 | Timing | logic analyzer channel 0 | toggled every control loop |
| 14 | Arm button | button to GND | internal pull-up |
| 48 or 38 | Status RGB LED | on-board | GPIO48 on board v1.0, GPIO38 on v1.1: check the silkscreen |

**Never use:** strapping pins 0, 3, 45, 46; 35–37 (octal PSRAM on N8R8); 19/20 (USB); 43/44 (UART console).

## Power

```
8×AA NiMH (9.6 V nominal, ~11.6 V full)
  └─ 5 A fuse ─ rocker switch ─┬─ left DRV8871 VM / GND     + 100 µF across it
                                ├─ right DRV8871 VM / GND    + 100 µF across it
                                ├─ R-78E5.0-0.5 in ─ 5 V out ─ DevKitC 5V pin   (10 µF on in and out)
                                └─ 100 kΩ ─┬─ 30 kΩ (3×10 kΩ) ─ GND
                                           └─ GPIO1 (+100 nF to GND)
All grounds common.
```

Battery sense: 11.6 V × 30 / 130 = 2.68 V at full charge, inside the ADC's range at 12 dB attenuation.

**Before connecting the board:** power the regulator alone and confirm 4.9–5.2 V on its output with the multimeter. The R-78E needs at least 7 V in; the low-battery latch trips well above that.

## Motors and encoders

- DRV8871 logic: IN1/IN2 straight from the ESP32 (3.3 V); driver GND common with the ESP32 GND. Each board's OUT1/OUT2 goes to its motor. Coast, brake and sleep behaviour (both inputs low) to be confirmed from the TI datasheet before the PWM driver is written.

- Encoder VCC from the board's **3.3 V** pin (the ESP32-S3 inputs are not 5 V tolerant).
- A 100 nF ceramic capacitor across each motor's terminals; twist each motor's two wires together.
- Wire colours on 37D encoder-motor leads vary by seller: identify them from the label or the encoder board before connecting, and measure the encoder output voltage before wiring it to the ESP32.
