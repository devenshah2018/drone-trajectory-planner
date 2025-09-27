import unittest
from copy import deepcopy

import numpy as np

from tests.common import TEST_CAMERA
import src.camera_utils as camera_utils


class CameraUnitsTest(unittest.TestCase):

    def test_project_world_point_to_image(self) -> None:

        # Case 1: Baseline
        world_point = np.array([10.0, 20.0, 50.0])
        expected_projection = np.array([640, 780.0])
        computed_projection = camera_utils.project_world_point_to_image(TEST_CAMERA, world_point)

        np.testing.assert_allclose(computed_projection, expected_projection)

        # Case 2: Swap out x and y
        world_point_ = np.array([world_point[1], world_point[0], world_point[2]])
        computed_projection = camera_utils.project_world_point_to_image(TEST_CAMERA, world_point_)
        np.testing.assert_allclose(computed_projection, expected_projection[::-1])

        # Case 3: Shift principal point to the bottom left
        camera_ = deepcopy(TEST_CAMERA)
        camera_.cx = camera_.image_size_x // 4
        camera_.cy = camera_.image_size_y * 3 // 4

        expected_projection = np.array([390.0, 1030.0])
        computed_projection = camera_utils.project_world_point_to_image(camera_, world_point)
        np.testing.assert_allclose(computed_projection, expected_projection)

        # Make two focal lengths different
        camera_ = deepcopy(TEST_CAMERA)
        camera_.fy = 0.75 * camera_.fx

        expected_projection = np.array([640.0, 710.0])
        computed_projection = camera_utils.project_world_point_to_image(camera_, world_point)
        np.testing.assert_allclose(computed_projection, expected_projection)

    def test_compute_image_footprint_on_surface(self) -> None:

        # Case 1: baseline
        height = 150
        expected_footprint = np.array([214.29, 214.29])
        computed_footprint = camera_utils.compute_image_footprint_on_surface(TEST_CAMERA, height)
        np.testing.assert_allclose(computed_footprint, expected_footprint, rtol=1e-1, atol=1e-1)

        # Case 2: double the height
        height = 300
        expected_footprint = np.array([428.57, 428.57])
        computed_footprint = camera_utils.compute_image_footprint_on_surface(TEST_CAMERA, height)
        np.testing.assert_allclose(computed_footprint, expected_footprint, rtol=1e-3, atol=1e-1)

        # Case 3: reduce focal lengths
        camera_ = deepcopy(TEST_CAMERA)
        camera_.fx = camera_.fx * 0.5
        camera_.fy = camera_.fy * 0.75
        height = 150
        expected_footprint = np.array([428.57, 285.71])
        computed_footprint = camera_utils.compute_image_footprint_on_surface(camera_, height)
        np.testing.assert_allclose(computed_footprint, expected_footprint, rtol=1e-3, atol=1e-1)

        # Case 4: increase number of pixels (this changes the focal length in mm)
        camera_ = deepcopy(TEST_CAMERA)
        camera_.image_size_x = camera_.image_size_x // 2
        height = 150
        expected_footprint = np.array([107.14, 214.29])
        computed_footprint = camera_utils.compute_image_footprint_on_surface(camera_, height)
        np.testing.assert_allclose(computed_footprint, expected_footprint, rtol=1e-1, atol=1e-1)

        # Case 5: increase sensor width (this changes focal length in mm proportionately)
        camera_ = deepcopy(TEST_CAMERA)
        camera_.sensor_size_y_mm = camera_.sensor_size_y_mm * 1.25
        height = 150
        expected_footprint = np.array([214.29, 214.29])
        computed_footprint = camera_utils.compute_image_footprint_on_surface(TEST_CAMERA, height)
        np.testing.assert_allclose(computed_footprint, expected_footprint, rtol=1e-1, atol=1e-1)

    def test_compute_gsd(self):
        # Case 1: baseline
        height = 150
        expected_gsd = 0.214
        computed_gsd = camera_utils.compute_ground_sampling_distance(TEST_CAMERA, height)
        self.assertAlmostEqual(computed_gsd, expected_gsd, places=3)

        # Case 3: asymmetric aspect ratio
        camera_ = deepcopy(TEST_CAMERA)
        camera_.fx = 5 * camera_.fy // 4
        expected_gsd = 0.171
        computed_gsd = camera_utils.compute_ground_sampling_distance(camera_, height)
        self.assertAlmostEqual(computed_gsd, expected_gsd, places=3)






if __name__ == '__main__':
    unittest.main()
