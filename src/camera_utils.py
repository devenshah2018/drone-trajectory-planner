"""Utility functions for the camera model.
"""

import numpy as np

from src.data_model import Camera


def compute_focal_length_in_mm(camera: Camera) -> np.ndarray:
    """Computes the focal length in mm for the given camera

    Args:
        camera: the camera model.

    Returns:
        [fx, fy] in mm as a 2-element array.
    """
    pixel_to_mm_x = camera.sensor_size_x_mm / camera.image_size_x
    pixel_to_mm_y = camera.sensor_size_y_mm / camera.image_size_y

    return np.array([camera.fx * pixel_to_mm_x, camera.fy * pixel_to_mm_y])


def project_world_point_to_image(camera: Camera, world_point: np.ndarray) -> np.ndarray:
    """Project a 3D world point into the image coordinates.

    Args:
        camera: the camera model
        world_point: the 3D world point

    Returns:
        [u, v] pixel coordinates corresponding to the 3D world point.
    """
    X, Y, Z = world_point
    
    # Project to image coordinates using pinhole camera model
    # x = fx * (X/Z), y = fy * (Y/Z)
    x = camera.fx * (X / Z)
    y = camera.fy * (Y / Z)
    
    # Convert to pixel coordinates
    u = x + camera.cx
    v = y + camera.cy
    
    return np.array([u, v])


def compute_image_footprint_on_surface(
    camera: Camera, distance_from_surface: float
) -> np.ndarray:
    """Compute the footprint of the image captured by the camera at a given distance from the surface.

    Args:
        camera: the camera model.
        distance_from_surface: distance from the surface (in m).

    Returns:
        [footprint_x, footprint_y] in meters as a 2-element array.
    """
    # Get image corner pixel coordinates
    corner1 = np.array([0, 0])  # Top-left
    corner2 = np.array([camera.image_size_x, camera.image_size_y])  # Bottom-right
    
    # Convert pixel coordinates to normalized image coordinates
    x1 = (corner1[0] - camera.cx) / camera.fx
    y1 = (corner1[1] - camera.cy) / camera.fy
    
    x2 = (corner2[0] - camera.cx) / camera.fx
    y2 = (corner2[1] - camera.cy) / camera.fy
    
    # Project to world coordinates at given distance
    X1 = x1 * distance_from_surface
    Y1 = y1 * distance_from_surface
    
    X2 = x2 * distance_from_surface
    Y2 = y2 * distance_from_surface
    
    # Calculate footprint dimensions
    footprint_x = abs(X2 - X1)
    footprint_y = abs(Y2 - Y1)
    
    return np.array([footprint_x, footprint_y])


def compute_ground_sampling_distance(
    camera: Camera, distance_from_surface: float
) -> float:
    """Compute the ground sampling distance (GSD) at a given distance from the surface.

    Args:
        camera: the camera model.
        distance_from_surface: distance from the surface (in m).

    Returns:
        The GSD in meters (smaller among x and y directions). You should return a float and not a numpy data type.
    """
    # Get the image footprint
    footprint = compute_image_footprint_on_surface(camera, distance_from_surface)
    
    # Calculate GSD in both directions
    gsd_x = footprint[0] / camera.image_size_x
    gsd_y = footprint[1] / camera.image_size_y
    
    # Return the smaller GSD (higher resolution)
    return float(min(gsd_x, gsd_y))
