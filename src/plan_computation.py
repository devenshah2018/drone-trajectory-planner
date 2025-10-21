import typing as T
import math
import numpy as np
from src.data_model import Camera, DatasetSpec, Waypoint, SegmentProfileType
from src.camera_utils import (
    compute_image_footprint_on_surface,
    compute_ground_sampling_distance,
)
from typing import List, Tuple, Dict, Any


def compute_distance_between_images(
    camera: Camera, dataset_spec: DatasetSpec
) -> np.ndarray:
    """Compute the distance between images in the horizontal and vertical directions for specified overlap and sidelap.

    Args:
        camera: Camera model used for image capture.
        dataset_spec: user specification for the dataset.

    Returns:
        The horizontal and vertical distance between images (as a 2-element array).
    """

    # Validate overlap and sidelap
    overlap = float(dataset_spec.overlap)
    sidelap = float(dataset_spec.sidelap)

    if not (0.0 <= overlap < 1.0):
        raise ValueError(f"overlap must be in [0, 1), got {overlap}")
    if not (0.0 <= sidelap < 1.0):
        raise ValueError(f"sidelap must be in [0, 1), got {sidelap}")

    # Compute the footprint of a single image on the surface at the height
    footprint = compute_image_footprint_on_surface(camera, dataset_spec.height)
    footprint_x, footprint_y = float(footprint[0]), float(footprint[1])

    # Distance between image centers
    distance_x = footprint_x * (1.0 - overlap)
    distance_y = footprint_y * (1.0 - sidelap)

    return np.array([distance_x, distance_y], dtype=float)

def compute_non_nadir_footprint(camera, height, camera_angle_rad) -> np.ndarray:
    """Compute the footprint of the image captured by the camera at a given distance from the surface for a non-nadir camera angle.

    Args:
        camera: the camera model.
        height: distance from the surface (in m).
        camera_angle_rad: angle of the camera from nadir (in radians).

    Returns:    
        [footprint_x, footprint_y] in meters as a 2-element array.
    """

    # Standard nadir footprint
    footprint = compute_image_footprint_on_surface(camera, height)
    
    # Stretch the footprint in the direction of tilt
    footprint_tilted = footprint.copy()

    # For non-nadir, the footprint on the ground increases by 1/cos(angle) in the direction of tilt
    footprint_tilted[1] /= np.cos(camera_angle_rad)
    return footprint_tilted

def compute_distance_between_images_non_nadir(camera, dataset_spec, camera_angle_rad):
    """Compute the distance between images in the horizontal and vertical directions for specified overlap and sidelap for a non-nadir camera angle.

    Args:
        camera: Camera model used for image capture.
        dataset_spec: user specification for the dataset.
        camera_angle_rad: angle of the camera from nadir (in radians).

    Returns:
        The horizontal and vertical distance between images (as a 2-element array).
    """

    # Compute the footprint of a single image on the surface at the height
    footprint = compute_non_nadir_footprint(camera, dataset_spec.height, camera_angle_rad)

    # Distance between image centers
    dx = footprint[0] * (1 - dataset_spec.overlap)
    dy = footprint[1] * (1 - dataset_spec.sidelap)
    return np.array([dx, dy])

def compute_speed_during_photo_capture(
    camera: Camera, dataset_spec: DatasetSpec, allowed_movement_px: float = 1, camera_angle_rad: float = 0.0
) -> float:
    """Compute the speed of drone during an active photo capture to prevent more than 1px of motion blur.

    Args:
        camera: Camera model used for image capture.
        dataset_spec: user specification for the dataset.
        allowed_movement_px: The maximum allowed movement in pixels. Defaults to 1 px.
        camera_angle_rad: angle of the camera from nadir (in radians). Default is 0 (nadir).

    Returns:
        The speed at which the drone should move during photo capture.
    """
    # Compute the ground sampling distance (GSD) at the flight height
    # GSD varies with gimbal angle due to footprint changes
    gsd = compute_ground_sampling_distance(camera, dataset_spec.height, camera_angle_rad)
    
    # Maximum allowed ground movement during exposure
    max_ground_movement = allowed_movement_px * gsd
    
    # Convert exposure time from milliseconds to seconds
    exposure_time_s = dataset_spec.exposure_time_ms / 1000.0
    
    # Speed = distance / time
    max_speed = max_ground_movement / exposure_time_s
    
    return float(max_speed)


def generate_photo_plan_on_grid(
    camera: Camera, dataset_spec: DatasetSpec
) -> T.List[Waypoint]:
    """Generate the complete photo plan as a list of waypoints in a lawn-mower pattern.

    Args:
        camera: Camera model used for image capture.
        dataset_spec: user specification for the dataset.

    Returns:
        Scan plan as a list of waypoints.

    """

    # Compute the distance between consecutive images
    distances = compute_distance_between_images(camera, dataset_spec)
    dx, dy = distances[0], distances[1]
    
    # Compute the maximum speed for blur-free photos
    max_speed = compute_speed_during_photo_capture(camera, dataset_spec)
    
    # Calculate the number of images needed in each direction. To ensure complete coverage, round up.
    num_images_x = math.ceil(dataset_spec.scan_dimension_x / dx) + 1
    num_images_y = math.ceil(dataset_spec.scan_dimension_y / dy) + 1
    
    # Calculate starting positions to center the grid
    start_x = -dataset_spec.scan_dimension_x / 2
    start_y = -dataset_spec.scan_dimension_y / 2
    
    waypoints = []
    
    # Generate waypoints in a lawn-mower pattern
    for row in range(num_images_y):
        y = start_y + row * dy
        
        # Alternate direction for lawn-mower pattern
        if row % 2 == 0:
            # Left to right
            x_range = range(num_images_x)
        else:
            # Right to left
            x_range = range(num_images_x - 1, -1, -1)
        
        for col in x_range:
            x = start_x + col * dx
            z = dataset_spec.height
            
            # Create waypoint with nadir scanning (looking straight down)
            waypoint = Waypoint(
                x=x,
                y=y, 
                z=z,
                speed=max_speed
            )
            waypoints.append(waypoint)
    
    return waypoints

def compute_segment_travel_time(
    distance: float,
    v_photo: float,
    v_max: float = 16.0,
    a_max: float = 3.5,
) -> Tuple[float, Dict[str, Any]]:
    """
    Compute minimum time to travel `distance` starting & ending at v_photo,
    given max speed v_max and max accel a_max.

    Returns (time_s, profile_dict).
    profile_dict includes 'type' (SegmentProfileType) and relevant params.
    """
    if distance <= 0.0:
        return 0.0, {"type": SegmentProfileType.DEGENERATE, "distance": distance}

    # candidate peak if no v_max limit (triangular)
    v_peak = np.sqrt(a_max * distance + v_photo * v_photo)

    if v_peak <= v_max:
        # triangular profile (no cruise)
        t_acc = (v_peak - v_photo) / a_max
        total_time = 2.0 * t_acc
        return total_time, {"type": SegmentProfileType.TRIANGULAR, "v_peak": v_peak, "t_acc": t_acc}

    # trapezoidal: reach v_max, cruise, then decel
    v_cruise = v_max
    # distance used accelerating from v_photo to v_cruise and back
    d_accdec = (v_cruise * v_cruise - v_photo * v_photo) / a_max
    if d_accdec >= distance:
        # numerical fallback: compute achievable peak (shouldn't normally happen because v_peak>v_max checked)
        v_peak = np.sqrt(a_max * distance + v_photo * v_photo)
        t_acc = (v_peak - v_photo) / a_max
        total_time = 2.0 * t_acc
        return total_time, {"type": SegmentProfileType.TRIANGULAR_FALLBACK, "v_peak": v_peak, "t_acc": t_acc}

    d_cruise = distance - d_accdec
    t_acc = (v_cruise - v_photo) / a_max
    t_cruise = d_cruise / v_cruise
    total_time = 2.0 * t_acc + t_cruise
    return total_time, {
        "type": SegmentProfileType.TRAPEZOIDAL,
        "v_cruise": v_cruise,
        "t_acc": t_acc,
        "t_cruise": t_cruise,
    }


def compute_plan_time(
    positions: List[Any],
    v_photo: float,
    exposure_time_s: float = 0.0,
    v_max: float = 16.0,
    a_max: float = 3.5,
) -> Tuple[float, List[Dict[str, Any]]]:
    """
    Compute total mission time for a sequence of positions (iterable of 2D/3D numpy-like points).
    - positions: list of coordinates (np.array, tuple, or Waypoint-like with .position)
    - v_photo: required speed at photo points (m/s)
    - exposure_time_s: per-photo dwell time (s) to account for capture
    Returns (total_time_s, segment_details)
    """
    # extract numpy arrays
    pts = []
    for p in positions:
        if hasattr(p, "pos"):
            pts.append(np.asarray(p.pos))
        else:
            pts.append(np.asarray(p))

    total_time = 0.0
    segments = []
    n = len(pts)
    if n <= 1:
        return total_time, segments

    # include initial photo dwell
    total_time += exposure_time_s
    for i in range(n - 1):
        p0, p1 = pts[i], pts[i + 1]
        dist = float(np.linalg.norm(p1 - p0))
        seg_time, profile = compute_segment_travel_time(dist, v_photo, v_max, a_max)
        dist = float(dist)
        seg_time = float(seg_time)
        capture_time = float(exposure_time_s)

        profile_clean = {
            "type": profile.get("type")
        }
        if "v_peak" in profile:
            profile_clean["v_peak"] = float(profile["v_peak"])
        if "t_acc" in profile:
            profile_clean["t_acc"] = float(profile["t_acc"])
        if "t_cruise" in profile:
            profile_clean["t_cruise"] = float(profile["t_cruise"])

        seg_info = {
            "index": int(i),
            "distance": dist,
            "travel_time_s": seg_time,
            "profile": profile_clean,
            "capture_time_s": capture_time,
        }
        total_time += seg_time + exposure_time_s
        segments.append(seg_info)

    return total_time, segments