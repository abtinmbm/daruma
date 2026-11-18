"""Tuning the cascade PID (and the tilt filter's time constant) against a fixed set of simulated scenarios.

Every candidate is scored on the same scenarios with the same noise seeds, so the cost is deterministic
and two candidates are compared on identical disturbances. The scenarios cover the nominal robot, a push,
+-20 % error on every physical parameter the controller depends on, a deadband at half and twice the
estimate, and a nearly flat (8.0 V) and fresh (11.2 V) battery.

Usage (from the repo root):
    uv run --project sim python -m daruma.tuning search      # Nelder-Mead from the current defaults
    uv run --project sim python -m daruma.tuning report      # metrics + evidence CSV + figure for the defaults
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys
import time
from collections.abc import Callable
from concurrent.futures import Executor, ProcessPoolExecutor
from dataclasses import asdict, dataclass, field, replace
from pathlib import Path

import numpy as np
from scipy.optimize import minimize

from daruma.controllers import CascadeGains
from daruma.fusion import DEFAULT_TAU_S
from daruma.params import RobotParams
from daruma.simulate import SimResult, simulate

REPO = Path(__file__).resolve().parents[3]
SCRATCH = REPO / "sim" / "scratch"

FALL_COST = 100.0
UNCERTAIN = ("m_body", "l_com", "j_body", "motor_kt", "motor_ke", "motor_r")


@dataclass(frozen=True)
class Scenario:
    name: str
    params: RobotParams
    theta0_deg: float = 5.0
    duration_s: float = 4.0
    push_impulse_ns: float = 0.0
    seed: int = 0


def scenarios(p: RobotParams | None = None) -> list[Scenario]:
    """The tuning set: nominal starts and push, parameter errors, deadband and battery extremes."""
    p = p or RobotParams()
    out = [
        Scenario("nominal_10deg_s0", p, theta0_deg=10.0, seed=0),
        Scenario("nominal_10deg_s1", p, theta0_deg=10.0, seed=1),
        Scenario("push_0p5", p, theta0_deg=0.0, push_impulse_ns=0.5, seed=2),
    ]
    for name in UNCERTAIN:
        for f in (0.8, 1.2):
            out.append(Scenario(f"{name}_x{f}", replace(p, **{name: getattr(p, name) * f})))
    for f in (0.5, 2.0):
        out.append(Scenario(f"deadband_x{f}", replace(p, motor_deadband_v=p.motor_deadband_v * f)))
    for v in (8.0, 11.2):
        out.append(Scenario(f"battery_{v}V", replace(p, v_battery=v)))
    for deg in (-2.0, 2.0):  # a level error the speed loop must trim out, else the robot rolls away
        out.append(Scenario(f"mount_{deg:+.0f}deg", replace(p, imu_mount_offset_rad=math.radians(deg)), duration_s=8.0))
    return out


@dataclass(frozen=True)
class Candidate:
    """Everything the search may change: the five cascade gains and the tilt filter's time constant."""

    gains: CascadeGains = field(default_factory=CascadeGains)
    tau_s: float = DEFAULT_TAU_S

    def to_vector(self) -> np.ndarray:
        g = self.gains
        return np.log([g.angle_kp, g.angle_ki, g.angle_kd, g.speed_kp, g.speed_ki, self.tau_s])

    @classmethod
    def from_vector(cls, v: np.ndarray, max_lean: float) -> Candidate:
        kp, ki, kd, skp, ski, tau = np.exp(v)
        gains = CascadeGains(float(kp), float(ki), float(kd), float(skp), float(ski), max_lean)
        return cls(gains, float(tau))


def run(sc: Scenario, cand: Candidate) -> SimResult:
    return simulate(
        sc.params,
        cand.gains,
        theta0_deg=sc.theta0_deg,
        duration_s=sc.duration_s,
        seed=sc.seed,
        push_impulse_ns=sc.push_impulse_ns,
        fusion_tau_s=cand.tau_s,
    )


@dataclass(frozen=True)
class Metrics:
    fell: bool
    settling_s: float  # |theta| inside 1 deg for the rest of the run (inf if never); reported, not in the cost
    mean_abs_tilt_deg: float  # over the whole run: how fast and how well it recovers
    tilt_rms_deg: float  # over the last 2 s
    max_drift_m: float  # largest |x| over the run
    voltage_rms: float  # control effort over the run
    voltage_step_rms: float  # RMS change of the command between periods: chatter (buzz, heat, wasted battery)
    final_speed_m_s: float  # mean |speed| over the last second: still rolling away?


def metrics(r: SimResult) -> Metrics:
    tail = r.t >= r.t[-1] - 2.0
    return Metrics(
        fell=r.fell,
        settling_s=r.settling_time(band_deg=1.0),
        mean_abs_tilt_deg=float(np.mean(np.abs(r.theta_deg))),
        tilt_rms_deg=float(np.sqrt(np.mean(r.theta_deg[tail] ** 2))),
        max_drift_m=float(np.max(np.abs(r.state[:, 0]))),
        voltage_rms=float(np.sqrt(np.mean(r.voltage**2))),
        voltage_step_rms=float(np.sqrt(np.mean(np.diff(r.voltage) ** 2))),
        final_speed_m_s=float(np.mean(np.abs(r.state[r.t >= r.t[-1] - 1.0, 1]))),
    )


def scenario_cost(m: Metrics) -> float:
    """Lower is better. A fall dominates; then recovery (mean |tilt|), standing tilt, drift, effort, chatter and
    still rolling at the end (which max drift alone misses when a level error is trimmed out too slowly).

    Settling time is left out on purpose: with a 1 deg band and ~0.5 deg of estimate noise it jumps between
    near 0 and several seconds for tiny gain changes, which breaks a simplex search.
    """
    if m.fell:
        return FALL_COST
    cost = m.mean_abs_tilt_deg + 2.0 * m.tilt_rms_deg + 2.0 * m.max_drift_m + 0.05 * m.voltage_rms + m.voltage_step_rms
    cost += 10.0 * m.final_speed_m_s
    return cost


def _scenario_cost(args: tuple[Scenario, Candidate]) -> float:
    return scenario_cost(metrics(run(*args)))


def cost(cand: Candidate, set_: list[Scenario], pool: Executor | None = None) -> float:
    """Mean scenario cost; scenarios run in parallel if a process pool is given (results are identical)."""
    jobs = [(sc, cand) for sc in set_]
    costs = list(pool.map(_scenario_cost, jobs)) if pool else [_scenario_cost(j) for j in jobs]
    return float(np.mean(costs))


def max_recovered_push(p: RobotParams, cand: Candidate, hi: float = 3.0, tol: float = 0.01) -> float:
    """Largest impulse (N s) at the COM, applied while standing, that the robot recovers from (bisection).

    Recovered = doesn't fall and is back inside 2 deg for the last second of a 5 s run.
    """

    def ok(j: float) -> bool:
        r = simulate(p, cand.gains, theta0_deg=0.0, duration_s=5.0, push_impulse_ns=j, push_time_s=1.0, seed=3,
                     fusion_tau_s=cand.tau_s)  # fmt: skip
        return not r.fell and float(np.abs(r.theta_deg[-int(p.loop_hz) :]).max()) < 2.0

    lo = 0.0
    if ok(hi):
        return hi
    while hi - lo > tol:
        mid = 0.5 * (lo + hi)
        lo, hi = (mid, hi) if ok(mid) else (lo, mid)
    return lo


def make_pool(n_jobs: int) -> ProcessPoolExecutor:
    """Process pool for scenario runs. Each worker gets one BLAS thread: the sim's 2x2 algebra gains nothing from
    threads, and 28 workers x 28 OpenBLAS threads each exhausted memory. Workers inherit the environment at spawn.
    """
    for var in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
        os.environ[var] = "1"
    return ProcessPoolExecutor(max_workers=min(n_jobs, os.cpu_count() or 1))


def search(
    start: Candidate, set_: list[Scenario], max_evals: int, pool: Executor | None = None, log: Callable[[str], None] = print
) -> Candidate:
    """Nelder-Mead in log-gain space (keeps every gain positive and scales steps relative to each gain)."""
    t0, n = time.perf_counter(), 0
    best = (math.inf, start)

    def f(v: np.ndarray) -> float:
        nonlocal n, best
        n += 1
        cand = Candidate.from_vector(v, start.gains.max_lean)
        c = cost(cand, set_, pool)
        if c < best[0]:
            best = (c, cand)
            log(f"[{n:4d} {time.perf_counter() - t0:6.0f}s] cost {c:.4f}  {asdict(cand.gains)} tau={cand.tau_s:.3f}")
        return c

    minimize(f, start.to_vector(), method="Nelder-Mead",
             options={"maxfev": max_evals, "xatol": 0.02, "fatol": 1e-3, "initial_simplex": None})  # fmt: skip
    return best[1]


def write_report(cand: Candidate, out_csv: Path, out_png: Path | None) -> dict[str, float]:
    """Runs the tuning set and the push bisection; writes one CSV row per scenario; returns headline numbers."""
    p = RobotParams()
    set_ = scenarios(p)
    rows = []
    for sc in set_:
        m = metrics(run(sc, cand))
        rows.append({"scenario": sc.name, **asdict(m), "cost": scenario_cost(m)})
    standing = simulate(p, cand.gains, theta0_deg=0.0, duration_s=10.0, seed=4, fusion_tau_s=cand.tau_s)
    headline = {
        "settling_10deg_s": max(r["settling_s"] for r in rows if r["scenario"].startswith("nominal_10deg")),
        "standing_tilt_rms_deg": metrics(standing).tilt_rms_deg,
        "standing_estimate_error_rms_deg": standing.estimate_error_rms_deg,
        "max_recovered_push_ns": max_recovered_push(p, cand),
        "scenarios_fallen": float(sum(r["fell"] for r in rows)),
        "tuning_cost": float(np.mean([r["cost"] for r in rows])),
    }
    out_csv.parent.mkdir(parents=True, exist_ok=True)
    with out_csv.open("w", newline="", encoding="utf-8") as f:
        f.write(f"# candidate: {json.dumps({**asdict(cand.gains), 'tau_s': cand.tau_s})}\n")
        f.write(f"# headline: {json.dumps(headline)}\n")
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    if out_png is not None:
        plot_response(p, cand, out_png)
    return headline


def plot_response(p: RobotParams, cand: Candidate, out_png: Path) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    start = simulate(p, cand.gains, theta0_deg=10.0, duration_s=4.0, seed=0, fusion_tau_s=cand.tau_s)
    push = simulate(p, cand.gains, theta0_deg=0.0, duration_s=4.0, push_impulse_ns=0.5, seed=2, fusion_tau_s=cand.tau_s)
    fig, axes = plt.subplots(3, 2, figsize=(10, 7), sharex="col", constrained_layout=True)
    for col, (r, title) in enumerate([(start, "Start at 10 deg"), (push, "0.5 N s push at t = 1 s")]):
        axes[0, col].plot(r.t, r.theta_deg, label="true tilt")
        axes[0, col].plot(r.t, np.degrees(r.theta_est), label="filter estimate", alpha=0.7)
        axes[0, col].set_ylabel("tilt (deg)")
        axes[0, col].set_title(f"{title} (simulation)")
        axes[0, col].legend(loc="upper right")
        axes[1, col].plot(r.t, r.state[:, 0] * 100)
        axes[1, col].set_ylabel("position (cm)")
        axes[2, col].plot(r.t, r.voltage)
        axes[2, col].set_ylabel("motor voltage (V)")
        axes[2, col].set_xlabel("time (s)")
        for ax in axes[:, col]:
            ax.grid(alpha=0.3)
    out_png.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_png, dpi=110)
    plt.close(fig)


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(prog="daruma.tuning")
    sub = parser.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("search")
    s.add_argument("--max-evals", type=int, default=400)
    r = sub.add_parser("report")
    r.add_argument("--csv", type=Path, default=SCRATCH / "tuning_report.csv")
    r.add_argument("--png", type=Path, default=SCRATCH / "tuning_response.png")
    args = parser.parse_args(argv)

    if args.cmd == "search":
        start = Candidate()
        set_ = scenarios()
        with make_pool(len(set_)) as pool:
            print(f"start cost {cost(start, set_, pool):.4f} over {len(set_)} scenarios", flush=True)
            best = search(start, set_, args.max_evals, pool, log=lambda m: print(m, flush=True))
        SCRATCH.mkdir(parents=True, exist_ok=True)
        out = SCRATCH / "tuning_best.json"
        out.write_text(json.dumps({**asdict(best.gains), "tau_s": best.tau_s}, indent=2), encoding="utf-8")
        print(f"best written to {out}")
    else:
        print(json.dumps(write_report(Candidate(), args.csv, args.png), indent=2))


if __name__ == "__main__":
    main(sys.argv[1:])
