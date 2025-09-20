"""Data models for the camera and user specification."""


class DatasetSpec:
    """
    Data model for specifications of an image dataset.
    """

    def __init__(self, overlap: float, sidelap: float, height: float, scan_dimension_x: float, scan_dimension_y: float, exposure_time_ms: float):
        self.overlap = overlap
        self.sidelap = sidelap
        self.height = height
        self.scan_dimension_x = scan_dimension_x
        self.scan_dimension_y = scan_dimension_y
        self.exposure_time_ms = exposure_time_ms
    
    def __str__(self) -> str:
        return (f"DatasetSpec(overlap={self.overlap}, sidelap={self.sidelap}, "f"height={self.height}m, scan_area={self.scan_dimension_x}x{self.scan_dimension_y}m, "f"exposure={self.exposure_time_ms}ms)")

    pass

class Camera:
    """
    Data model for a simple pinhole camera.

    References:
    - https://github.com/colmap/colmap/blob/3f75f71310fdec803ab06be84a16cee5032d8e0d/src/colmap/sensor/models.h#L220
    - https://en.wikipedia.org/wiki/Pinhole_camera_model
    """

    def __init__(self, fx: float, fy: float, cx: float, cy: float, sensor_size_x_mm: float, sensor_size_y_mm: float, image_size_x: int, image_size_y: int):
        self.fx = fx
        self.fy = fy
        self.cx = cx
        self.cy = cy
        self.sensor_size_x_mm = sensor_size_x_mm
        self.sensor_size_y_mm = sensor_size_y_mm
        self.image_size_x = image_size_x
        self.image_size_y = image_size_y
    
    def __str__(self) -> str:
        return (f"Camera(fx={self.fx:.2f}, fy={self.fy:.2f}, "f"cx={self.cx:.1f}, cy={self.cy:.1f}, "f"sensor={self.sensor_size_x_mm:.3f}x{self.sensor_size_y_mm:.3f}mm, "f"resolution={self.image_size_x}x{self.image_size_y}px)")
    
    pass


class Waypoint:
    """
    Waypoints are positions where the drone should fly to and capture a photo.
    """

    pass
