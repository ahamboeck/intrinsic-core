# Copyright 2026 Intrinsic Innovation LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Analytic RGB-depth registration fixtures, independent of cameras/models."""

import unittest

from intrinsic_perception.intrinsic.perception.service.ioc_pose_estimator.service import depth_registration
import numpy as np


class DepthRegistrationTest(unittest.TestCase):

  def register(self, depth, depth_k=None, rgb_k=None, pose=None, shape=None):
    return depth_registration.register_depth_to_rgb(
        np.array(depth, dtype=np.float32),
        np.eye(3) if depth_k is None else depth_k,
        np.eye(3) if rgb_k is None else rgb_k,
        (len(depth), len(depth[0])) if shape is None else shape,
        np.eye(4) if pose is None else pose,
    )

  def test_identical_calibration_preserves_valid_depth(self):
    result = self.register([[1, 2], [0, np.nan]])
    np.testing.assert_array_equal(result, [[1, 2], [0, 0]])
    self.assertEqual(result.dtype, np.float32)

  def test_colocated_intrinsic_change_uses_rgb_pixel_rays(self):
    rgb_k = np.diag([2.0, 1.0, 1.0])
    result = self.register([[1, 2, 3]], rgb_k=rgb_k, shape=(1, 5))
    np.testing.assert_array_equal(result, [[1, 2, 2, 3, 3]])

  def test_registered_input_is_not_registered_again(self):
    k = np.diag([3.0, 4.0, 1.0])
    np.testing.assert_array_equal(self.register([[1, 2, 3]], k, k), [[1, 2, 3]])

  def test_translation_reprojects_and_z_buffer_keeps_nearest(self):
    # p0=(0,0,1), p1=(2,0,2); after tx=2 both project to RGB pixel 2.
    pose = np.eye(4)
    pose[0, 3] = 2
    result = self.register([[1, 2, 0]], pose=pose, shape=(1, 4))
    np.testing.assert_array_equal(result, [[0, 0, 1, 0]])

  def test_rgb_depth_is_transformed_axial_z_not_original_z(self):
    pose = np.eye(4)
    pose[2, 3] = 1
    np.testing.assert_array_equal(self.register([[2]], pose=pose), [[3]])

  def test_rotated_sensor_uses_extrinsic_rotation(self):
    pose = np.array(
        [
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [-1.0, 0.0, 0.0, 2.0],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )
    np.testing.assert_array_equal(
        self.register([[2]], pose=pose, shape=(1, 2)), [[0, 2]]
    )

  def test_out_of_view_behind_camera_and_invalid_depth_are_empty(self):
    pose = np.eye(4)
    pose[2, 3] = -3
    np.testing.assert_array_equal(
        self.register([[1, 0, -1, np.inf]], pose=pose), [[0, 0, 0, 0]]
    )
    pose = np.eye(4)
    pose[0, 3] = 100
    np.testing.assert_array_equal(self.register([[1]], pose=pose), [[0]])

  def test_invalid_calibration_rejects(self):
    for k in (np.zeros((3, 3)), np.eye(2), np.full((3, 3), np.nan)):
      with self.subTest(k=k), self.assertRaises(ValueError):
        self.register([[1]], depth_k=k)
    for pose in (np.eye(3), np.zeros((4, 4)), np.diag([-1.0, 1.0, 1.0, 1.0])):
      with self.subTest(pose=pose), self.assertRaises(ValueError):
        self.register([[1]], pose=pose)


if __name__ == "__main__":
  unittest.main()
