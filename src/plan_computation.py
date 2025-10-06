import typing as T
import math

import numpy as np

from src.data_model import Camera, DatasetSpec, Waypoint
from src.camera_utils import (
    compute_image_footprint_on_surface,
    compute_ground_sampling_distance,
)


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

def compute_non_nadir_footprint(camera, height, camera_angle_rad):
    # Standard nadir footprint
    footprint = compute_image_footprint_on_surface(camera, height)
    
    # Stretch the footprint in the direction of tilt
    footprint_tilted = footprint.copy()

    # For non-nadir, the footprint on the ground increases by 1/cos(angle) in the direction of tilt
    footprint_tilted[1] /= np.cos(camera_angle_rad)
    return footprint_tilted

def compute_distance_between_images_non_nadir(camera, dataset_spec, camera_angle_rad):
    footprint = compute_non_nadir_footprint(camera, dataset_spec.height, camera_angle_rad)
    dx = footprint[0] * (1 - dataset_spec.overlap)
    dy = footprint[1] * (1 - dataset_spec.sidelap)
    return np.array([dx, dy])

def compute_speed_during_photo_capture(
    camera: Camera, dataset_spec: DatasetSpec, allowed_movement_px: float = 1
) -> float:
    """Compute the speed of drone during an active photo capture to prevent more than 1px of motion blur.

    Args:
        camera: Camera model used for image capture.
        dataset_spec: user specification for the dataset.
        allowed_movement_px: The maximum allowed movement in pixels. Defaults to 1 px.

    Returns:
        The speed at which the drone should move during photo capture.
    """
    raise NotImplementedError()


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
    raise NotImplementedError()