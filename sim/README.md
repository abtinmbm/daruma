# Daruma simulation

Physics model, controller design and analysis tools for the robot. Everything is checked in CI.

```
uv sync
uv run pytest -q
```

- `src/daruma/model.py`: equations of motion (wheeled inverted pendulum + DC gearmotors) and the sign convention
- `src/daruma/params.py`: physical parameters, each marked as an estimate or a measurement
- `src/daruma/controllers.py`: the PID controllers, mirroring the firmware's C code in float32
- `src/daruma/simulate.py`: closed loop with a discrete controller, sensor noise, encoder quantisation, actuation delay and pushes
- `src/daruma/results.py`: the only writer of `results/results.csv`
