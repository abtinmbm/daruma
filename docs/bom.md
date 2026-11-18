# Bill of materials

Prices are approximate CAD before tax, checked November 2026; marketplace listings change, so check the details below before buying. Links are product pages where known and searches otherwise.

## To buy

| # | Part | Qty | What to check before buying | Where | ~CAD |
|---|---|---|---|---|---|
| 1 | ESP32-S3-DevKitC-1-N8R8 | 1 | Exact model N8R8 (8 MB flash, 8 MB PSRAM) | [Digi-Key](https://www.digikey.ca/en/products/detail/espressif-systems/ESP32-S3-DEVKITC-1-N8R8/15295894) | 23 |
| 2 | Adafruit LSM6DSOX 6-DoF IMU breakout (#4438) | 1 | Comes with header pins to solder | [Digi-Key](https://www.digikey.ca/en/products/detail/adafruit-industries-llc/4438/11497501) or PiShop.ca | 18 |
| 3 | JGB37-520 gearmotor with Hall encoder, **12 V, ~330 rpm (30:1)** | 2 | Encoder version (6 wires: motor ±, GND, VCC, A, B); encoder works at 3.3 V; ideally bundled with L-bracket and 6 mm coupling | [Amazon.ca search](https://www.amazon.ca/s?k=JGB37-520+encoder+12V+330RPM) | 55–70 |
| 4 | Rubber wheels 65–85 mm with 6 mm shaft couplings | 2 | Hub fits a 6 mm D-shaft; skip if item 3 includes wheels | [Amazon.ca search](https://www.amazon.ca/s?k=65mm+wheel+6mm+coupling+JGB37) | 15–20 |
| 5 | Cytron MDD3A dual motor driver | 1 | 4–16 V, 3 A per channel | [RobotShop CA search](https://ca.robotshop.com/search?q=cytron%20mdd3a) | 15 |
| 6 | AA NiMH rechargeable cells (e.g. IKEA LADDA 2450 mAh) | 8 | NiMH, not alkaline | IKEA / Amazon.ca | 25 |
| 7 | NiMH AA charger | 1 | Skip if you already have one | IKEA / Amazon.ca | 15–25 |
| 8 | 8×AA battery holder with leads (or 2× 4×AA in series) | 1 | Wire leads, ideally with a cover | [Amazon.ca search](https://www.amazon.ca/s?k=8+AA+battery+holder+leads) | 8 |
| 9 | Inline blade-fuse holder + 5 A fuses | 1 | ATC/ATO size | Amazon.ca / Canadian Tire | 8 |
| 10 | Rocker switch, ≥ 6 A at 12 V DC | 1 | | Amazon.ca | 6 |
| 11 | 5 V buck converter, fixed 5 V output (e.g. Mini560 5 V) | 1–2 | Input range covers 12 V; fixed-output version | [Amazon.ca search](https://www.amazon.ca/s?k=Mini560+5V+buck) | 6–10 |
| 12 | M3 brass standoff + screw kit | 1 | Assorted 10–40 mm | [Amazon.ca search](https://www.amazon.ca/s?k=M3+brass+standoff+kit) | 15 |
| 13 | Female–female Dupont jumper wires | 1 pack | | Amazon.ca | 6 |
| 14 | 22 AWG silicone hookup wire + heat-shrink | 1 | For motor and battery wiring | Amazon.ca | 15 |
| 15 | 24 MHz 8-channel USB logic analyzer | 1 | Works with PulseView (sigrok) | [Amazon.ca search](https://www.amazon.ca/s?k=24MHz+8+channel+logic+analyzer) or RobotShop | 15–30 |
| 16 | Laser-cut 1/8" acrylic plates (2–3) | — | Cut from `cad/` DXF at the university prototyping centre | — | ~1.30 each |

**Estimated total:** about CA$190–240 before tax.

## Already owned and used

Multimeter, solder kit, half-size breadboard (mounted on the robot for v1 wiring), jumper wires, 40-pin headers, 100 µF and 10 µF electrolytic capacitors (bulk capacitance at the motor driver), 100 nF ceramic capacitors (across motor terminals and the battery-sense divider), resistors (100 kΩ + 3× 10 kΩ battery divider), buttons (arm/reset), LEDs, buzzer.

## Why these parts

| Choice | Instead of | Reason |
|---|---|---|
| LSM6DSOX | MPU-6050 | The MPU-6050 is not recommended for new designs and is widely cloned; the LSM6DSOX has lower noise, SPI and a data-ready interrupt. |
| LSM6DSOX (raw data) | BNO085 (on-chip fusion) | Writing the sensor fusion is part of the project. |
| JGB37-520 30:1 with encoder | N20, steppers, gimbal BLDC | Enough torque, ~1320 counts per wheel turn, low cost; steppers lose the motor-model work, BLDC needs field-oriented control. |
| Cytron MDD3A | TB6612FNG, DRV8874 | TB6612 is marginal against ~2.4–3.5 A stall current; current sensing (DRV8874) isn't needed. |
| 8× AA NiMH | 3S LiPo | Safe to charge anywhere, no balance charger; lower top speed is fine for balancing. |
| Custom acrylic plates | Balancing-robot kit | Kits weren't cheaper and their electronics would be discarded. |
