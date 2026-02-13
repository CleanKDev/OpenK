from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Iterable

import numpy as np


@dataclass(frozen=True)
class ScurveDurations:
    distance: float
    t_j: float
    t_a: float
    t_v: float
    j: float
    T: float
    v_peak: float
    a_peak: float


def build_time_samples(T: float, dt: float) -> np.ndarray:
    if T <= 0.0 or dt <= 0.0:
        raise ValueError("T and dt must be positive.")
    t = np.arange(0.0, T, dt, dtype=float)
    if t.size == 0 or not math.isclose(t[-1], T):
        t = np.append(t, T)
    return t


def _solve_ta(distance: float, T: float, j: float, t_j: float) -> tuple[float, float] | None:
    if t_j <= 0.0:
        return None
    K = distance / (j * t_j)
    disc = (T - 3.0 * t_j) ** 2 + 4.0 * (t_j * T - 2.0 * t_j**2 - K)
    if disc < -1e-12:
        return None
    disc = max(disc, 0.0)
    sqrt_disc = math.sqrt(disc)
    candidates = (
        0.5 * ((T - 3.0 * t_j) + sqrt_disc),
        0.5 * ((T - 3.0 * t_j) - sqrt_disc),
    )
    for t_a in candidates:
        t_v = T - 4.0 * t_j - 2.0 * t_a
        if t_a >= -1e-9 and t_v >= -1e-9:
            return max(t_a, 0.0), max(t_v, 0.0)
    return None


def solve_scurve_durations(
    distance: float,
    T: float,
    j_lim: float,
    a_lim: float,
    *,
    search_steps: int = 200,
) -> ScurveDurations | None:
    distance = abs(float(distance))
    if distance == 0.0:
        return ScurveDurations(
            distance=0.0,
            t_j=0.0,
            t_a=0.0,
            t_v=T,
            j=j_lim,
            T=T,
            v_peak=0.0,
            a_peak=0.0,
        )
    if T <= 0.0 or j_lim <= 0.0 or a_lim <= 0.0:
        return None

    t_j_max = min(a_lim / j_lim, T / 4.0)
    if t_j_max <= 0.0:
        return None

    def build_params(t_j: float, t_a: float, t_v: float) -> ScurveDurations:
        v_peak = j_lim * t_j**2 + j_lim * t_j * t_a
        a_peak = j_lim * t_j
        return ScurveDurations(
            distance=distance,
            t_j=t_j,
            t_a=t_a,
            t_v=t_v,
            j=j_lim,
            T=T,
            v_peak=v_peak,
            a_peak=a_peak,
        )

    candidate = _solve_ta(distance, T, j_lim, t_j_max)
    if candidate is not None:
        t_a, t_v = candidate
        return build_params(t_j_max, t_a, t_v)

    if search_steps < 2:
        return None

    for step in range(search_steps):
        frac = (search_steps - 1 - step) / (search_steps - 1)
        t_j = t_j_max * frac
        if t_j <= 0.0:
            continue
        candidate = _solve_ta(distance, T, j_lim, t_j)
        if candidate is None:
            continue
        t_a, t_v = candidate
        return build_params(t_j, t_a, t_v)

    return None


def _state_first_half(t: float, params: ScurveDurations) -> tuple[float, float]:
    t_j = params.t_j
    t_a = params.t_a
    t_v = params.t_v
    j = params.j

    t1 = t_j
    t2 = t1 + t_a
    t3 = t2 + t_j
    t4 = t3 + t_v

    v1 = 0.5 * j * t_j**2
    q1 = (1.0 / 6.0) * j * t_j**3
    a1 = j * t_j
    v2 = v1 + a1 * t_a
    q2 = q1 + v1 * t_a + 0.5 * a1 * t_a**2
    v_peak = params.v_peak
    q3 = j * t_j**3 + 1.5 * j * t_j**2 * t_a + 0.5 * j * t_j * t_a**2

    if t <= t1:
        q = (1.0 / 6.0) * j * t**3
        v = 0.5 * j * t**2
        return q, v
    if t <= t2:
        tau = t - t1
        q = q1 + v1 * tau + 0.5 * a1 * tau**2
        v = v1 + a1 * tau
        return q, v
    if t <= t3:
        tau = t - t2
        q = q2 + v2 * tau + 0.5 * a1 * tau**2 - (1.0 / 6.0) * j * tau**3
        v = v2 + a1 * tau - 0.5 * j * tau**2
        return q, v
    if t <= t4:
        tau = t - t3
        q = q3 + v_peak * tau
        v = v_peak
        return q, v
    return params.distance, 0.0


def scurve_state_at(t: float, params: ScurveDurations) -> tuple[float, float]:
    if t <= 0.0:
        return 0.0, 0.0
    if t >= params.T:
        return params.distance, 0.0

    t_j = params.t_j
    t_a = params.t_a
    t_v = params.t_v
    t3 = 2.0 * t_j + t_a
    t4 = t3 + t_v

    if t <= t4:
        return _state_first_half(t, params)

    t_mirror = params.T - t
    q_mirror, v_mirror = _state_first_half(t_mirror, params)
    return params.distance - q_mirror, v_mirror


def generate_scurve_profile(
    q0: float,
    q1: float,
    T: float,
    dt: float,
    j_lim: float,
    a_lim: float,
    *,
    t_samples: Iterable[float] | None = None,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, ScurveDurations] | None:
    t_samples_arr = np.asarray(t_samples, dtype=float) if t_samples is not None else build_time_samples(T, dt)
    distance = float(abs(q1 - q0))
    params = solve_scurve_durations(distance, T, j_lim, a_lim)
    if params is None:
        return None
    if distance == 0.0:
        q = np.full_like(t_samples_arr, q0, dtype=float)
        dq = np.zeros_like(t_samples_arr, dtype=float)
        return t_samples_arr, q, dq, params

    sign = 1.0 if q1 >= q0 else -1.0
    q = np.empty_like(t_samples_arr)
    dq = np.empty_like(t_samples_arr)
    for idx, t in enumerate(t_samples_arr):
        pos, vel = scurve_state_at(float(t), params)
        q[idx] = q0 + sign * pos
        dq[idx] = sign * vel
    return t_samples_arr, q, dq, params


def run_scurve_self_test(
    *,
    distance: float = 50.0,
    T: float = 2.5,
    dt: float = 0.005,
    v_lim: float = 40.0,
    a_lim: float = 120.0,
    j_lim: float = 300.0,
    save_path: str | Path | None = None,
) -> dict[str, float]:
    result = generate_scurve_profile(0.0, distance, T, dt, j_lim, a_lim)
    if result is None:
        raise AssertionError("Profile generation failed in self-test.")
    t, q, dq, params = result

    if np.any(~np.isfinite(q)) or np.any(~np.isfinite(dq)):
        raise AssertionError("Non-finite values in profile.")

    max_v = float(np.max(np.abs(dq)))
    if max_v > v_lim + 1e-6:
        raise AssertionError(f"Velocity limit exceeded: {max_v} > {v_lim}")

    dt_samples = np.diff(t)
    dq_est = np.diff(q) / dt_samples
    max_dq_err = float(np.max(np.abs(dq_est - dq[:-1])))
    if max_dq_err > max(1e-3, 0.1 * max_v):
        raise AssertionError(f"Discontinuity detected in dq (max error {max_dq_err}).")

    if save_path is not None:
        path = Path(save_path)
        path.parent.mkdir(parents=True, exist_ok=True)
        np.savez(path, t=t, q=q, dq=dq)

    return {
        "max_velocity": max_v,
        "a_peak": params.a_peak,
        "j_peak": params.j,
    }
