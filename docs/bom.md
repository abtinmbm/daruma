# Bill of materials

Prices are CAD before tax. Stock, price and delivery date were checked on the store pages on November 24, 2026, for delivery to Richmond Hill, ON. Rule for every line: in stock, delivered by December 12, and a part commonly used in two-wheeled balancing robots. Two online stores (Amazon.ca, Digi-Key) plus a local hardware or auto store.

## Required

### Amazon.ca (free delivery on these listings)

| # | Part | Qty | Checked Nov 24 | Notes | CAD |
|---|---|---|---|---|---|
| 1 | [DIYmall ESP32-S3-DevKitC-1 N16R8, 2-pack](https://www.amazon.ca/dp/B0BSCXHB5S) | 1 pack | "Only 1 left in stock"; ships from Amazon; delivery Dec 2 | 16 MB flash, 8 MB octal PSRAM, DevKitC-1 layout. The second board is a spare. If it sells out: [Canada Robotix #5218](https://www.canadarobotix.com/products/5218), N16R8, CA$17.99, "Only 23 left" (adds a store and its shipping) | 32.99 |
| 2 | [JGB37-520 12 V encoder gearmotor set, **333 rpm**](https://www.amazon.ca/dp/B0H9GJ3C8Z) (Xixian): motor, wheel, bracket, coupling, cable, screws | 2 | "In Stock"; ships from the seller (Fragriant); delivery Dec 2–9; returnable until Jan 31, 2027 | Select size **333RPM** (30:1). Listing: 11 PPR Hall encoder (1320 counts per wheel turn with quadrature), stall 2.3 A at 12 V, max torque 5 kg·cm. Wheel diameter isn't stated: measure on arrival. No ratings yet; if delivery slips past Dec 12: [RobotShop E-S Motor 320 rpm kit](https://ca.robotshop.com/products/e-s-motor-12v-37d-metal-gearmotor-w-11ppr-encoder-80d-wheel-kits-320rpm), CA$26.88 each | 39.98 |
| 3 | [POWEROWL 8× AA NiMH 2800 mAh + 8-bay charger](https://www.amazon.ca/dp/B07TW9T8JW) | 1 | "In Stock"; ships from Amazon; delivery Nov 28; 4.4★ (14,701) | NiMH (1.2 V), not the "1.5 V lithium" AA cells, whose built-in regulators can't supply motor current | 29.59 |
| 4 | [8× AA battery holder with lead, 2-set](https://www.amazon.ca/dp/B07WP1CYYW) | 1 | Delivery Nov 28; 4.4★ (942) | Snap lead: cut it off and solder the pack straight to the fuse holder. Second holder is a spare | 10.99 |

### Digi-Key (digikey.ca; free shipping over CA$100, otherwise the fee is shown at checkout)

| # | Part | Qty | Checked Nov 24 | Notes | CAD |
|---|---|---|---|---|---|
| 5 | [Adafruit LSM6DSOX 6-DoF IMU (#4438)](https://www.digikey.ca/en/products/detail/adafruit-industries-llc/4438/11497501) | 1 | In-Stock: 143 | Genuine Adafruit (generic LSM6DS3/LSM6DSO boards have a different pinout); headers to solder | 17.75 |
| 6 | [Adafruit DRV8871 motor driver breakout (#3190)](https://www.digikey.ca/en/products/detail/adafruit-industries-llc/3190/6138098) | 2 | In-Stock: 1,873 | One per motor. 6.5–45 V, two PWM inputs per motor (same scheme as the MDD3A). Current regulated to ~2 A by the fitted 30 kΩ R_lim, under the motor's 2.3 A listed stall | 22.28 |
| 7 | [Recom R-78E5.0-0.5 5 V switching regulator](https://www.digikey.ca/en/products/detail/recom-power/R-78E5-0-1-0/2834904) | 1 | In-Stock: 30,835 | Powers the ESP32 from the pack: 7–28 V in, 5 V 0.5 A out, 3-pin drop-in. Ample with Wi-Fi and Bluetooth off; 10 µF (owned) on input and output | 5.58 |

### Local (Canadian Tire / Home Depot)

| # | Part | Qty | Notes | CAD |
|---|---|---|---|---|
| 8 | Toggle or rocker switch, 2-pin, ≥ 6 A at 12 V DC | 1 | Hard power cut | ~3–5 |
| 9 | Inline ATC/ATO blade-fuse holder + 5 A fuse | 1 | Standard size, not "mini" | ~5 |
| 10 | M3 × 40 mm screws + nuts (plate spacers) | ~8 + 16 nuts | Nuts on both sides of each plate set the spacing | ~5 |
| 11 | Laser-cut 1/8" acrylic plates (2–3) | — | Cut from the `cad/` file at the university prototyping centre before Dec 19: CA$0.55 per 10 × 10 cm of 1/8" acrylic (price list checked Nov 24), so about 1–2 per plate | ~4 total |

**Total:** Amazon.ca 113.55 + Digi-Key 45.61 + local about 17 + plates about 4 = about CA$180, plus the Digi-Key shipping fee shown at checkout. Budget cap: CA$225 including shipping.

## Optional / later

| Part | When to buy | Where | CAD |
|---|---|---|---|
| 22 AWG silicone stranded wire | Only if a motor or battery lead turns out too short | [Amazon.ca, Fermerry 6 × 10 ft](https://www.amazon.ca/Fermerry-Stranded-Electric-Tinned-Copper/dp/B089CQHRDT) | 20.49 |
| 24 MHz 8-channel USB logic analyzer | Only if SPI or encoder debugging stalls; loop timing is measured in firmware (timestamps in the serial log) | [Amazon.ca, HiLetgo](https://www.amazon.ca/HiLetgo-Analyzer-Device-Channel-Arduino/dp/B077LSG5P2) | 21.99 |

## Already owned and used

Multimeter, solder kit, half-size breadboard (mounted on the robot for v1 wiring), jumper wires (male and female), 40-pin headers, 100 µF and 10 µF electrolytic capacitors (bulk capacitance at each motor driver and at the regulator), 100 nF ceramic capacitors (across motor terminals and the battery-sense divider), resistors (100 kΩ + 3× 10 kΩ battery divider), buttons (arm/reset), LEDs, buzzer.

## Why these parts

| Choice | Instead of | Reason |
|---|---|---|
| LSM6DSOX | MPU-6050 | The MPU-6050 is not recommended for new designs and is widely cloned; the LSM6DSOX has lower noise, SPI and a data-ready interrupt. |
| LSM6DSOX (raw data) | BNO085 (on-chip fusion) | Writing the sensor fusion is part of the project. |
| JGB37-520 12 V 30:1 Hall-encoder kit | N20, steppers, gimbal BLDC, DFRobot FIT0186, Pololu 37D | The usual motor in balance-car kits: enough torque, 1320 counts per wheel turn, wheel and bracket included. Steppers lose the motor-model work; BLDC needs field-oriented control; the FIT0186 (Digi-Key) stalls at 7 A (beyond either driver), needs a 5 V encoder supply and separate wheels; Pololu 37D motors cost about twice as much. |
| 2× Adafruit DRV8871 | Cytron MDD3A, TB6612FNG, L298N | The MDD3A was on backorder at RobotShop and not carried by Canada Robotix or PiShop on Nov 24. The DRV8871 keeps the same two-PWM-per-motor wiring, and its ~2 A regulation protects gearboxes and cells; in simulation the limit left every recorded result unchanged. TB6612 (1.2 A) is too small; the L298N drops ~2 V. |
| Recom R-78E5.0-0.5 | Waveshare buck module (PiShop) | Same job from the same order as the IMU and drivers (one store fewer); genuine part with a datasheet. |
| 8× AA NiMH | 3S LiPo | Safe to charge anywhere, no balance charger; lower top speed is fine for balancing. |
| Custom acrylic plates | Balancing-robot kit | Kits weren't cheaper and their electronics would be discarded. |
