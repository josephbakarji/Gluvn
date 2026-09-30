import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from vision.calibration import fit_similarity
from vision.coordinate_transform import CoordinateTransform, quaternion_wxyz_to_rotation


class TestVisionAlignment(unittest.TestCase):
    def test_explicit_transform_and_inverse(self):
        transform = CoordinateTransform(
            quaternion_wxyz_to_rotation((np.sqrt(0.5), 0.0, 0.0, np.sqrt(0.5))),
            [1.0, 2.0, 3.0],
            2.0,
        )
        point = np.array([0.5, -1.0, 0.25])
        np.testing.assert_allclose(transform.inverse().transform_point(transform.transform_point(point)), point)

    def test_similarity_calibration_recovers_known_mapping(self):
        camera = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]], dtype=float)
        known = CoordinateTransform(
            np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]], dtype=float),
            [0.2, -0.3, 0.4], 0.5,
        )
        result = fit_similarity(camera, known.transform_points(camera), estimate_scale=True)
        np.testing.assert_allclose(result.transform_points(camera), known.transform_points(camera), atol=1e-7)


if __name__ == "__main__":
    unittest.main()
