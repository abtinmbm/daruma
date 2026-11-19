# Bill of materials

Prices are CAD before tax, and stock was checked on the store pages on November 18, 2026. Online stores: Canada Robotix, PiShop.ca and RobotShop CA. Cells, charger, fuse, switch and screws can be bought locally.

## Required

| # | Part | Qty | Notes | Where | CAD |
|---|---|---|---|---|---|
| 1 | ESP32-S3 DevKitC-1-compatible board, ESP32-S3-WROOM-1-N16R8 | 1 | 16 MB flash, 8 MB octal PSRAM, two USB-C, DevKitC-1 pin layout (checked from the photo). Same pin rules as N8R8 | [Canada Robotix](https://www.canadarobotix.com/products/5218) | 17.99 |
| 2 | 8-AA battery holder, back-to-back, with leads | 1 | No built-in switch, so motor current goes through item 9 | [Canada Robotix](https://www.canadarobotix.com/products/179) | 1.99 |
| 3 | Adafruit LSM6DSOX 6-DoF IMU breakout (#4438) | 1 | Genuine Adafruit (a generic LSM6DS3/LSM6DSO board has a different pinout); header pins to solder | [PiShop.ca](https://www.pishop.ca/product/adafruit-lsm6dsox-6-dof-accelerometer-and-gyroscope-stemma-qt-qwiic/) | 18.95 |
| 4 | 5 V DC-DC buck converter (Waveshare 29035 mini module) | 1 | Powers the ESP32 from the pack. 6–36 V input in the default 5 V mode, up to 4 A out. The MDD3A's own 5 V output (200 mA) is too marginal | [PiShop.ca](https://www.pishop.ca/product/3-3v-or-5v-dc-dc-buck-converter/) | 4.95 |
| 5 | 12 V 37D gearmotor kit with 11 PPR magnetic encoder, **320 rpm**: motor, 80 × 35 mm wheel, metal bracket, coupling, screws | 2 | Sold and shipped by E-S Motor (RobotShop marketplace seller): check the delivery estimate at checkout. Gear ratio isn't stated (measure it on arrival). Encoders of this type run on 3.3 or 5 V (same-type listings; E-S Motor's own sheet not readable): power the encoder from 3.3 V first; if it needs 5 V, divide each output down with two owned resistors before the ESP32 pin | [RobotShop CA](https://ca.robotshop.com/products/e-s-motor-12v-37d-metal-gearmotor-w-11ppr-encoder-80d-wheel-kits-320rpm) | 26.88 each |
| 6 | Cytron MDD3A dual motor driver | 1 | 4–16 V, 3 A per channel | [RobotShop CA](https://ca.robotshop.com/products/cytron-3a-4-16v-dual-channel-dc-motor-driver) | 12.63 |
| 7 | AA NiMH cells | 8 | NiMH, not alkaline. Skip if owned | Local store or Amazon.ca (WENJOOP 8-pack) | ~14 |
| 8 | NiMH AA charger | 1 | Skip if owned | Local store or Amazon.ca (WENJOOP 8-slot) | ~12 |
| 9 | Toggle or rocker switch, 2-pin, ≥ 6 A at 12 V DC | 1 | Hard power cut | Local hardware/auto store | ~3–5 |
| 10 | Inline ATC/ATO blade-fuse holder + one 5 A fuse | 1 | Standard size, not "mini" | Local auto store (e.g. Canadian Tire) | ~5 |
| 11 | M3 × 40 mm screws + nuts (as plate spacers) | ~8 + 16 nuts | Nuts on both sides of each plate set the spacing | Local hardware store | ~5 |
| 12 | Laser-cut 1/8" acrylic plates (2–3) | — | Cut from `cad/` DXF at the university prototyping centre | — | ~1.30 each |

**Total:** about CA$150 before tax and shipping.

## Optional / later

| Part | When to buy | Where | CAD |
|---|---|---|---|
| 22 AWG silicone stranded wire | Only if a motor or battery lead turns out too short (the motor kits, battery holder and fuse holder come with leads) | [Amazon.ca, Fermerry 6 × 10 ft](https://www.amazon.ca/Fermerry-Stranded-Electric-Tinned-Copper/dp/B089CQHRDT) | 20.49 |
| 24 MHz 8-channel USB logic analyzer | Only if SPI or encoder debugging stalls in M2; loop timing is measured in firmware (timestamps in the serial log) | [Amazon.ca, HiLetgo](https://www.amazon.ca/HiLetgo-Analyzer-Device-Channel-Arduino/dp/B077LSG5P2) | 21.99 |

## Already owned and used

Multimeter, solder kit, half-size breadboard (mounted on the robot for v1 wiring), jumper wires (male and female), 40-pin headers, 100 µF and 10 µF electrolytic capacitors (bulk capacitance at the motor driver), 100 nF ceramic capacitors (across motor terminals and the battery-sense divider), resistors (100 kΩ + 3× 10 kΩ battery divider), buttons (arm/reset), LEDs, buzzer.

## Why these parts

| Choice | Instead of | Reason |
|---|---|---|
| LSM6DSOX | MPU-6050 | The MPU-6050 is not recommended for new designs and is widely cloned; the LSM6DSOX has lower noise, SPI and a data-ready interrupt. |
| LSM6DSOX (raw data) | BNO085 (on-chip fusion) | Writing the sensor fusion is part of the project. |
| 37D 12 V ~30:1 gearmotor kit with encoder (JGB37-520 class) | N20, steppers, gimbal BLDC, DFRobot FIT0186, Pololu 37D | Enough torque, ~1320 counts per wheel turn if the ratio is 30:1, low cost, wheel and bracket included; steppers lose the motor-model work, BLDC needs field-oriented control; the FIT0186 (Digi-Key) is slower (251 rpm, 43.8:1) and needs separate wheels and hubs; Pololu 37D motors are only sold through Digi-Key's marketplace (separate shipping) and cost about twice as much. |
| Cytron MDD3A | TB6612FNG, DRV8874 | TB6612 is marginal against ~2.4–3.5 A stall current; current sensing (DRV8874) isn't needed. |
| 8× AA NiMH | 3S LiPo | Safe to charge anywhere, no balance charger; lower top speed is fine for balancing. |
| Custom acrylic plates | Balancing-robot kit | Kits weren't cheaper and their electronics would be discarded. |
