#!/usr/bin/env python

# Copyright 2025 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Check how often randomly sampled S-curve segments are feasible.
This is an offline check (no robot IO). Units: rad, rad/s, N.m (centered).

Example:
```shell
python scripts/check_segment_sampling.py \
  --config_path=config/generate_torque_data.yaml \
  --num_segments=200 \
  --plot=true
```
"""

from dataclasses import asdict, dataclass
import importlib
import json
import logging
import math
from pathlib import Path

import numpy as np

from lerobot.configs import parser
from lerobot.robots import RobotConfig
from lerobot.utils.constants import HF_LEROBOT_CALIBRATION, TELEOPERATORS
from lerobot.utils.import_utils import register_third_party_plugins
from lerobot.utils.utils import init_logging

from utils.scurve_profile import build_time_samples, generate_scurve_profile


def _register_openk() -> None:
    for module in (
        "openk.openk-1-alpha_follower2",
    ):
        importlib.import_module(module)


@dataclass
class SegmentSamplingCheckConfig:
    robot: RobotConfig | None = None
    joint_names: list[str] | None = None
    # Accepted for config compatibility; not used in this check.
    always_calibrate: bool = False

    q_min: list[float] | float | None = None
    q_max: list[float] | float | None = None
    v_min: list[float] | float = 0.2
    v_max: list[float] | float = 1.0
    a_lim: list[float] | float = 2.0
    j_lim: list[float] | float = 10.0

    segment_time_min: float = 1.0
    segment_time_max: float = 3.0
    time_sampling: str = "lognormal_frequency"
    time_log_sigma: float | None = None
    time_sampling_max_tries: int = 200
    dt: float = 1.0 / 30.0
    num_segments: int = 200
    max_segment_attempts: int = 50
    hold_sec: float = 0.0

    q_start: list[float] | float | None = None

    calibration_file: str | Path | None = None
    calibration_dir: str | Path | None = None

    output_dir: str | Path = "outputs/segment_sampling_check"
    output_csv: str | Path | None = None
    save_npz: bool = True
    plot: bool = False
    num_plot_segments: int = 3

    seed: int | None = None
    log_file: str | Path | None = "logs/check_segment_sampling.log"


def _coerce_array(value: list[float] | float | str | None, n_dof: int, name: str) -> np.ndarray | None:
    if value is None:
        return None
    if isinstance(value, str):
        parts = [p.strip() for p in value.split(",") if p.strip()]
        value = [float(p) for p in parts]
    if isinstance(value, (int, float)):
        return np.full(n_dof, float(value), dtype=float)
    arr = np.array(list(value), dtype=float)
    if arr.size == 1:
        return np.full(n_dof, float(arr[0]), dtype=float)
    if arr.size != n_dof:
        raise ValueError(f"{name} has length {arr.size}, expected {n_dof}.")
    return arr


def _safe_limits(min_val: float, max_val: float, margin: float = 0.05) -> tuple[float, float]:
    if max_val < min_val:
        min_val, max_val = max_val, min_val
    span = max_val - min_val
    return min_val + margin * span, max_val - margin * span


def _resolve_calibration_path(cfg: SegmentSamplingCheckConfig) -> Path | None:
    if cfg.calibration_file is not None:
        return Path(cfg.calibration_file)
    if cfg.calibration_dir is not None and cfg.robot is not None:
        robot_id = getattr(cfg.robot, "id", None)
        if robot_id:
            return Path(cfg.calibration_dir) / f"{robot_id}.json"
    if cfg.robot is not None:
        robot_id = getattr(cfg.robot, "id", None)
        robot_calib_dir = getattr(cfg.robot, "calibration_dir", None)
        if robot_id and robot_calib_dir is not None:
            return Path(robot_calib_dir) / f"{robot_id}.json"
    if cfg.robot is not None:
        robot_id = getattr(cfg.robot, "id", None)
        robot_type = getattr(cfg.robot, "type", None)
        if robot_id and robot_type == "openk-1-alpha_follower2":
            return HF_LEROBOT_CALIBRATION / TELEOPERATORS / "openk_follower2" / f"{robot_id}.json"
    return None


def _load_calibration_limits(
    cfg: SegmentSamplingCheckConfig,
) -> tuple[list[str], dict[str, tuple[float, float]]]:
    calibration_path = _resolve_calibration_path(cfg)
    if calibration_path is None:
        raise FileNotFoundError("Calibration path not provided. Set calibration_dir or calibration_file.")
    if not calibration_path.is_file():
        raise FileNotFoundError(f"Calibration file not found: {calibration_path}")

    with calibration_path.open("r", encoding="utf-8") as f:
        data = json.load(f)

    damiao = data.get("damiao", {})
    if not damiao:
        raise RuntimeError("Damiao calibration data is missing.")
    joint_names = list(damiao.keys())
    limits: dict[str, tuple[float, float]] = {}

    for name, calib in damiao.items():
        min_raw = float(calib["range_min"])
        max_raw = float(calib["range_max"])
        limits[name] = (min_raw, max_raw)

    return joint_names, limits


def _derive_safe_joint_ranges_raw(
    joint_names: list[str],
    limits_raw: dict[str, tuple[float, float]],
) -> tuple[np.ndarray, np.ndarray]:
    q_min = []
    q_max = []
    for name in joint_names:
        min_raw, max_raw = limits_raw[name]
        safe_min, safe_max = _safe_limits(min_raw, max_raw, margin=0.05)
        q_min.append(safe_min)
        q_max.append(safe_max)
    return np.array(q_min, dtype=float), np.array(q_max, dtype=float)


def _compute_centers(joint_names: list[str], limits_raw: dict[str, tuple[float, float]]) -> np.ndarray:
    centers = []
    for name in joint_names:
        min_raw, max_raw = limits_raw[name]
        centers.append(0.5 * (min_raw + max_raw))
    return np.array(centers, dtype=float)


def _sample_segment_time(
    rng: np.random.Generator,
    t_min: float,
    t_max: float,
    *,
    method: str,
    log_sigma: float | None,
    max_tries: int,
) -> float:
    if method == "uniform":
        return float(rng.uniform(t_min, t_max))
    if method != "lognormal_frequency":
        raise ValueError(f"Unknown time_sampling method: {method}")

    # Sample log-normal in frequency space f in [1/T_max, 1/T_min].
    f_min = 1.0 / t_max
    f_max = 1.0 / t_min
    log_f_min = math.log(f_min)
    log_f_max = math.log(f_max)
    if log_sigma is None:
        log_sigma = max(1e-6, (log_f_max - log_f_min) / 6.0)
    log_mu = 0.5 * (log_f_min + log_f_max)

    freq = None
    for _ in range(max_tries):
        candidate = float(rng.lognormal(log_mu, log_sigma))
        if f_min <= candidate <= f_max:
            freq = candidate
            break
        freq = candidate
    if freq is None:
        freq = float(rng.uniform(f_min, f_max))
    freq = min(max(freq, f_min), f_max)
    return 1.0 / freq


def _maybe_plot_segments(
    output_dir: Path,
    samples: list[dict[str, np.ndarray]],
) -> None:
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        logging.warning("matplotlib is not available; skipping plots.")
        return

    for idx, sample in enumerate(samples):
        t = sample["t"]
        q = sample["q"]
        fig, axes = plt.subplots(q.shape[0], 1, sharex=True, figsize=(7.0, 1.8 * q.shape[0]))
        if q.shape[0] == 1:
            axes = [axes]
        for j, ax in enumerate(axes):
            ax.plot(t, q[j], linewidth=1.2)
            ax.set_ylabel(f"q[{j}]")
        axes[-1].set_xlabel("t [s]")
        fig.tight_layout()
        fig.savefig(output_dir / f"segment_{idx:03d}.png")
        plt.close(fig)


@parser.wrap()
def check_segment_sampling(cfg: SegmentSamplingCheckConfig) -> None:
    if cfg.log_file is not None:
        Path(cfg.log_file).parent.mkdir(parents=True, exist_ok=True)
    init_logging(log_file=cfg.log_file)
    logging.info("Config: %s", asdict(cfg))

    if cfg.segment_time_max < cfg.segment_time_min:
        raise ValueError("segment_time_max must be >= segment_time_min.")
    if cfg.dt <= 0.0:
        raise ValueError("dt must be positive.")

    q_min_raw = cfg.q_min
    q_max_raw = cfg.q_max
    if (q_min_raw is None) != (q_max_raw is None):
        raise ValueError("q_min and q_max must be provided together or omitted together.")

    if q_min_raw is not None and q_max_raw is not None:
        if isinstance(q_min_raw, list) and isinstance(q_max_raw, list):
            n_dof = len(q_min_raw)
            if len(q_max_raw) != n_dof:
                raise ValueError("q_min and q_max must have the same length.")
            q_min = np.array(q_min_raw, dtype=float)
            q_max = np.array(q_max_raw, dtype=float)
            if cfg.joint_names is None:
                joint_names = [f"joint_{i}" for i in range(n_dof)]
            else:
                if len(cfg.joint_names) != n_dof:
                    raise ValueError("joint_names length must match q_min/q_max length.")
                joint_names = cfg.joint_names
        elif isinstance(q_min_raw, (int, float)) and isinstance(q_max_raw, (int, float)):
            if cfg.joint_names is None:
                raise ValueError("joint_names is required when q_min/q_max are scalars.")
            n_dof = len(cfg.joint_names)
            joint_names = cfg.joint_names
            q_min = np.full(n_dof, float(q_min_raw), dtype=float)
            q_max = np.full(n_dof, float(q_max_raw), dtype=float)
        else:
            raise ValueError("q_min/q_max must both be lists or both be scalars.")
    else:
        if cfg.robot is None:
            raise ValueError("robot config is required when q_min/q_max are omitted.")
        calib_joint_names, limits_raw = _load_calibration_limits(cfg)
        if cfg.joint_names is None:
            joint_names = calib_joint_names
        else:
            if set(cfg.joint_names) != set(calib_joint_names):
                raise ValueError("joint_names must include all Damiao motors.")
            joint_names = cfg.joint_names
        n_dof = len(joint_names)
        if q_min_raw is None and q_max_raw is None:
            q_min_raw_arr, q_max_raw_arr = _derive_safe_joint_ranges_raw(joint_names, limits_raw)
            centers = _compute_centers(joint_names, limits_raw)
            q_min = q_min_raw_arr - centers
            q_max = q_max_raw_arr - centers
        else:
            q_min = _coerce_array(q_min_raw, n_dof, "q_min")
            q_max = _coerce_array(q_max_raw, n_dof, "q_max")

    v_min = _coerce_array(cfg.v_min, n_dof, "v_min")
    v_max = _coerce_array(cfg.v_max, n_dof, "v_max")
    a_lim = _coerce_array(cfg.a_lim, n_dof, "a_lim")
    j_lim = _coerce_array(cfg.j_lim, n_dof, "j_lim")
    q_start = _coerce_array(cfg.q_start, n_dof, "q_start")

    if v_min is None or v_max is None or a_lim is None or j_lim is None:
        raise ValueError("v_min/v_max/a_lim/j_lim must be provided.")
    if np.any(q_min >= q_max):
        raise ValueError("q_min must be < q_max for all joints.")
    if np.any(v_min > v_max):
        raise ValueError("v_min must be <= v_max for all joints.")

    output_dir = Path(cfg.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    seed = cfg.seed if cfg.seed is not None else 0
    rng = np.random.default_rng(seed)

    if q_start is None:
        q_current = 0.5 * (q_min + q_max)
    else:
        q_current = q_start.astype(float)

    attempts = np.zeros(cfg.num_segments, dtype=int)
    success = np.zeros(cfg.num_segments, dtype=bool)
    reject_profile = 0
    reject_dq = 0
    samples_for_plot: list[dict[str, np.ndarray]] = []

    for segment_id in range(cfg.num_segments):
        for attempt in range(cfg.max_segment_attempts):
            attempts[segment_id] += 1

            q_target = rng.uniform(q_min, q_max)
            dq_max_sampled = rng.uniform(v_min, v_max)
            T_segment = _sample_segment_time(
                rng,
                cfg.segment_time_min,
                cfg.segment_time_max,
                method=cfg.time_sampling,
                log_sigma=cfg.time_log_sigma,
                max_tries=cfg.time_sampling_max_tries,
            )
            t_samples = build_time_samples(T_segment, cfg.dt)

            q_cmd_list = []
            feasible = True
            for i in range(n_dof):
                result = generate_scurve_profile(
                    q_current[i],
                    q_target[i],
                    T_segment,
                    cfg.dt,
                    j_lim[i],
                    a_lim[i],
                    t_samples=t_samples,
                )
                if result is None:
                    feasible = False
                    reject_profile += 1
                    break
                _, q_prof, dq_prof, _ = result
                if np.max(np.abs(dq_prof)) > dq_max_sampled[i] + 1e-6:
                    feasible = False
                    reject_dq += 1
                    break
                q_cmd_list.append(q_prof)

            if feasible:
                q_cmds = np.stack(q_cmd_list, axis=0)
                success[segment_id] = True
                q_current = q_cmds[:, -1]
                if len(samples_for_plot) < cfg.num_plot_segments:
                    samples_for_plot.append({"t": t_samples, "q": q_cmds})
                break

    success_rate = float(np.mean(success))
    avg_attempts = float(np.mean(attempts[success])) if np.any(success) else float("nan")
    logging.info("Success rate: %.3f", success_rate)
    logging.info("Average attempts (success only): %.2f", avg_attempts)
    logging.info("Reject counts: profile=%d dq=%d", reject_profile, reject_dq)

    if cfg.save_npz:
        np.savez(
            output_dir / "sampling_stats.npz",
            attempts=attempts,
            success=success.astype(np.int32),
            success_rate=success_rate,
            reject_profile=reject_profile,
            reject_dq=reject_dq,
            q_min=q_min,
            q_max=q_max,
            v_min=v_min,
            v_max=v_max,
            a_lim=a_lim,
            j_lim=j_lim,
        )

    if cfg.plot and samples_for_plot:
        _maybe_plot_segments(output_dir, samples_for_plot)


def main() -> None:
    register_third_party_plugins()
    _register_openk()
    check_segment_sampling()


if __name__ == "__main__":
    main()
