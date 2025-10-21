"""Data models for the camera and user specification."""


class DatasetSpec:
    """
    Data model for specifications of an image dataset.
    
    Attributes:
        overlap (float): Forward overlap ratio between consecutive images (0-1)
        sidelap (float): Side overlap ratio between adjacent flight lines (0-1)
        height (float): Flight height above ground level in meters
        scan_dimension_x (float): Width of the survey area in meters
        scan_dimension_y (float): Length of the survey area in meters
        exposure_time_ms (float): Camera shutter exposure time in milliseconds
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

    Attributes:
        fx (float): Focal length in pixels along the x-axis
        fy (float): Focal length in pixels along the y-axis
        cx (float): Principal point x-coordinate in pixels (optical center)
        cy (float): Principal point y-coordinate in pixels (optical center)
        sensor_size_x_mm (float): Physical sensor width in millimeters
        sensor_size_y_mm (float): Physical sensor height in millimeters
        image_size_x (int): Image width in pixels
        image_size_y (int): Image height in pixels
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
    
    Attributes:
        x (float): X coordinate in meters (eastward direction)
        y (float): Y coordinate in meters (northward direction)
        z (float): Z coordinate in meters (altitude above ground)
        speed (float): Maximum speed at this waypoint in m/s for blur-free photos
    """

    def __init__(self, x: float, y: float, z: float, speed: float):
        self.x = x
        self.y = y
        self.z = z
        self.speed = speed
    
    def __str__(self) -> str:
        return (f"Waypoint(pos=({self.x:.1f}, {self.y:.1f}, {self.z:.1f}), "
                   f"speed={self.speed:.2f}m/s)")
    
    def __repr__(self) -> str:
        return self.__str__()
