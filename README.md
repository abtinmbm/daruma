# Daruma

[![ci](https://github.com/abtinmbm/daruma/actions/workflows/ci.yml/badge.svg)](https://github.com/abtinmbm/daruma/actions/workflows/ci.yml)
[![Licence: MIT](https://img.shields.io/badge/licence-MIT-blue.svg)](LICENSE)

**A two-wheeled robot that balances itself and gets back up when you push it.** Named after the Daruma doll: fall seven times, stand up eight.

> **Status:** early development. The physics simulation and the first controller run in CI; the hardware is being assembled. Nothing has been measured on the real robot yet, so this page makes no performance claims. Every number that appears here later will come from [`results/results.csv`](results/) with its raw data.

## How it works

An ESP32-S3 reads a 6-axis IMU (inertial measurement unit) over SPI about 400 times a second, fuses the accelerometer and gyro into a tilt angle, and drives two encoder gearmotors to keep the wheels under the centre of mass. A cascaded controller does the balancing: an outer loop turns a wheel-speed error into a lean angle, and an inner loop turns the lean error into motor voltage. A linear-quadratic regulator (LQR) designed from an identified model of the real robot will be compared against it on identical tests.

The controller is designed and tested in simulation first. The C control code that runs on the robot is compiled on the host in CI and must match the Python reference bit for bit.

## Repository

| Folder | What's in it |
| --- | --- |
| [`sim/`](sim/) | Physics model, controllers, simulation, analysis tools (Python) |
| [`firmware/`](firmware/) | ESP-IDF firmware (C, FreeRTOS); `components/` hold host-testable control code |
| [`dashboard/`](dashboard/) | Web Bluetooth page for live telemetry and tuning |
| [`cad/`](cad/) | Chassis plates as code, exported to DXF for laser cutting |
| [`docs/`](docs/) | Bill of materials, wiring, design notes |
| [`results/`](results/) | Every measured number, with its raw evidence |

## Run the checks

```
cd sim
uv sync
uv run pytest -q
```

The C test needs `gcc` on the PATH. The firmware builds with ESP-IDF v6.1 (`idf.py build` in `firmware/`).

## Roadmap

1. Simulation, control library and hardware bring-up
2. First balance with the cascaded controller, system identification, live tuning over Bluetooth
3. Measurements: loop timing, push recovery, balance quality
4. LQR from the identified model, compared with the cascaded controller on the same tests
5. A custom PCB to replace the wiring
