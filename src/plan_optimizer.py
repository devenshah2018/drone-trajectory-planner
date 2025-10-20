"""
Cross-entropy Method (CEM) optimizer for drone photo camera/mission parameters.
"""

import numpy as np
from typing import Tuple, Dict, Any, List
from src.data_model import Camera, DatasetSpec, Waypoint
from src.plan_computation import (
    generate_photo_plan_on_grid,
    compute_plan_time,
    compute_speed_during_photo_capture,
)
from src.camera_utils import compute_ground_sampling_distance

def waypoints_to_positions(waypoints: List[Waypoint]) -> List[np.ndarray]:
    """
    Convert Waypoint list to list of 3D numpy positions for compute_plan_time
    
    Args:
        waypoints: List of Waypoint objects.

    Returns:
        List of np.ndarray positions [X, Y, Z].
    """
    pts = []
    for w in waypoints:
        pts.append(np.array([float(w.x), float(w.y), float(w.z)]))
    return pts

def evaluate_candidate(
    camera: Camera,
    base_dataset: DatasetSpec,
    candidate_params: Dict[str, float],
) -> Tuple[float, Dict[str, Any]]:
    """
    Evaluate a candidate parameter vector.
    
    Args:
        camera: the camera model
        base_dataset: the base dataset spec to copy other params from
        candidate_params: dict with keys: 'height', 'overlap', 'sidelap', 'exposure_time_ms'
    
    Returns:
        (objective_score, info_dict) where higher objective is better.
    """
    ds = DatasetSpec(
        overlap=float(candidate_params['overlap']),
        sidelap=float(candidate_params['sidelap']),
        height=float(candidate_params['height']),
        scan_dimension_x=float(base_dataset.scan_dimension_x),
        scan_dimension_y=float(base_dataset.scan_dimension_y),
        exposure_time_ms=float(candidate_params['exposure_time_ms']),
    )

    # Generate flight plan
    try:
        plan = generate_photo_plan_on_grid(camera, ds)
    except Exception as e:
        return -1e6, {"error": f"plan_generation_failed: {e}"}

    pts = waypoints_to_positions(plan)

    # Compute max blur-free speed
    try:
        v_photo = compute_speed_during_photo_capture(camera, ds)
    except Exception as e:
        return -1e6, {"error": f"compute_speed_failed: {e}"}

    # 3) Compute mission time
    try:
        total_time, segments = compute_plan_time(pts, v_photo, exposure_time_s=ds.exposure_time_ms/1000.0)
    except Exception as e:
        return -1e6, {"error": f"compute_plan_time_failed: {e}"}

    # 4) Compute GSD at this height
    try:
        gsd = float(compute_ground_sampling_distance(camera, ds.height))
    except Exception as e:
        return -1e6, {"error": f"compute_gsd_failed: {e}"}

    info = {
        "plan": plan,
        "num_photos": len(plan),
        "v_photo": float(v_photo),
        "mission_time_s": float(total_time),
        "gsd_m": float(gsd),
        "segments": segments,
        "candidate_params": candidate_params,
    }

    # Objective will be computed relative to baseline externally
    return 0.0, info 


def compute_baseline_metrics(camera: Camera, dataset: DatasetSpec) -> Dict[str, Any]:
    """
    Compute baseline metrics for given camera and dataset.

    Args:
        camera: the camera model.
        dataset: the dataset specification.

    Returns:
        A dict with baseline metrics:
        {
            "plan": ...,
            "num_photos": ...,
            "v_photo": ...,
            "mission_time_s": ...,
            "gsd_m": ...,
            "segments": ...,
        }
    """
    plan_base = generate_photo_plan_on_grid(camera, dataset)
    pts_base = waypoints_to_positions(plan_base)
    v_photo_base = compute_speed_during_photo_capture(camera, dataset)
    time_base, segments_base = compute_plan_time(pts_base, v_photo_base, exposure_time_s=dataset.exposure_time_ms/1000.0)
    gsd_base = float(compute_ground_sampling_distance(camera, dataset.height))
    return {
        "plan": plan_base,
        "num_photos": len(plan_base),
        "v_photo": float(v_photo_base),
        "mission_time_s": float(time_base),
        "gsd_m": float(gsd_base),
        "segments": segments_base,
    }


def combined_objective(baseline: Dict[str, Any], candidate_info: Dict[str, Any], quality_weight: float = 0.5) -> float:
    """
    Combine quality and speed equally and normalize both by clipping to reasonable ranges and average them.

    Args:
        baseline: dict with baseline metrics.
        candidate_info: dict with candidate metrics.
        quality_weight: weight for quality in objective (between 0 and 1). Speed weight is (1 - quality_weight).
    
    Returns:
        The combined objective score as a float.
    """
    base_gsd = baseline["gsd_m"]
    cand_gsd = candidate_info["gsd_m"]
    base_time = baseline["mission_time_s"]
    cand_time = candidate_info["mission_time_s"]

    # Validate time and gsd are positive
    if cand_time <= 0 or cand_gsd <= 0:
        return -1e6

    # Compute ratios
    quality_ratio = base_gsd / cand_gsd
    speed_ratio = base_time / cand_time

    # Clamp extreme values to avoid runaway influence
    quality_norm = float(np.clip(quality_ratio, 0.1, 10.0))
    speed_norm = float(np.clip(speed_ratio, 0.1, 10.0))

    # Equal weight
    obj = quality_weight * quality_norm + (1 - quality_weight) * speed_norm
    return float(obj)


def optimize_parameters_cem(
    camera: Camera,
    base_dataset: DatasetSpec,
    baseline_metrics: Dict[str, Any],
    n_iter: int = 40,
    population_size: int = 128,
    topk: int = 16,
    seed: int = 0,
    quality_weight: float = 0.5,
) -> Dict[str, Any]:
    """
    Optimize [height, overlap, sidelap, exposure_time_ms] with CEM.
    
    Args:
        camera: the camera model.
        base_dataset: the base dataset spec to copy other params from.
        baseline_metrics: dict with baseline metrics to compare against.
        n_iter: number of CEM iterations.
        population_size: number of samples per iteration.
        topk: number of top samples to use for updating distribution.
        seed: random seed for reproducibility.
        quality_weight: weight for quality in objective (between 0 and 1). Speed weight is (1 - quality_weight).
    
    Returns:
        A dict with best candidate parameters and info:
        {
            "best_params": {...},
            "best_info": {...},
            "quality_ratio": ...,
            "speed_ratio": ...,
            "objective_score": ...,
        }
    """
    rng = np.random.RandomState(seed)

    # Initialize mean from baseline
    mean = np.array([
        float(base_dataset.height),
        float(base_dataset.overlap),
        float(base_dataset.sidelap),
        float(base_dataset.exposure_time_ms),
    ], dtype=float)

    # Reasonable std devs for sampling relative to parameter positions in array
    std = np.array([10.0, 0.1, 0.1, 1.0], dtype=float)

    # Bounds
    bounds = {
        "height": (5.0, 120.0),            # meters
        "overlap": (0.3, 0.95),
        "sidelap": (0.3, 0.95),
        "exposure_time_ms": (0.2, 10.0),
    }

    best_overall = None
    best_score = -np.inf

    # CEM iterations
    for it in range(n_iter):
        # Sample population
        samples = rng.normal(loc=mean[None, :], scale=std[None, :], size=(population_size, 4))

        # Clip to bounds
        samples[:, 0] = np.clip(samples[:, 0], bounds["height"][0], bounds["height"][1])
        samples[:, 1] = np.clip(samples[:, 1], bounds["overlap"][0], bounds["overlap"][1])
        samples[:, 2] = np.clip(samples[:, 2], bounds["sidelap"][0], bounds["sidelap"][1])
        samples[:, 3] = np.clip(samples[:, 3], bounds["exposure_time_ms"][0], bounds["exposure_time_ms"][1])

        results = []
        infos = []
        for s in samples:
            cand = {
                "height": float(s[0]),
                "overlap": float(s[1]),
                "sidelap": float(s[2]),
                "exposure_time_ms": float(s[3]),
            }
            _, info = evaluate_candidate(camera, base_dataset, cand)
            # Compute objective only if info has valid keys
            if "error" in info:
                score = -1e6
            else:
                score = combined_objective(baseline_metrics, info, quality_weight)
            results.append(score)
            infos.append(info)

        results = np.array(results)
        # Pick topk
        top_idx = results.argsort()[-topk:]
        top_samples = samples[top_idx]
        top_scores = results[top_idx]

        # Update mean & std as weighted
        weights = (top_scores - top_scores.min()) + 1e-6
        weights = weights / weights.sum()
        new_mean = (weights[:, None] * top_samples).sum(axis=0)
       
        # Update std as weighted variance
        diff = top_samples - new_mean[None, :]
        new_var = (weights[:, None] * (diff ** 2)).sum(axis=0)
        new_std = np.sqrt(new_var + 1e-6)

        # Smoothing update
        alpha = 0.7
        mean = alpha * new_mean + (1 - alpha) * mean
        std = alpha * new_std + (1 - alpha) * std

        # Track best
        iter_best_idx = int(np.argmax(results))
        iter_best_score = float(results[iter_best_idx])
        iter_best_info = infos[iter_best_idx] if iter_best_idx < len(infos) else None
        if iter_best_score > best_score and iter_best_info and "error" not in iter_best_info:
            best_score = iter_best_score
            best_overall = iter_best_info

    # After iterations, return best candidate's info augmented with baseline comparisons
    if best_overall is None:
        raise RuntimeError("CEM failed to find valid candidate")
    quality_ratio = baseline_metrics["gsd_m"] / best_overall["gsd_m"]
    speed_ratio = baseline_metrics["mission_time_s"] / best_overall["mission_time_s"]
    result = {
        "best_params": best_overall["candidate_params"],
        "best_info": best_overall,
        "quality_ratio": float(quality_ratio),
        "speed_ratio": float(speed_ratio),
        "objective_score": float(best_score),
    }

    return result
