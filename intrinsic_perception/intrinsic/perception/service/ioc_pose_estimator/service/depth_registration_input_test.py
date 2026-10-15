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

"""Exercise registration at the pose service's capture extraction boundary."""

import types
import unittest

from intrinsic.math.python import pose3
from intrinsic.perception.proto.v1 import image_buffer_pb2
from intrinsic_perception.intrinsic.perception.service.ioc_pose_estimator.service import ioc_pose_estimator_service as service
import numpy as np


class DepthRegistrationInputTest(unittest.TestCase):

  def extract(
      self,
      rgb_k,
      depth_k,
      *,
      missing_pose=False,
      distortion=None,
      rgb_pose=None,
      depth_pose=None,
  ):
    rgb = np.zeros((2, 5, 3), dtype=np.uint8)
    depth = np.array([[1, 2, 3], [1, 2, 3]], dtype=np.float32)

    def sensor(array, kind, k, pose):
      return types.SimpleNamespace(
          array=array,
          intrinsic_matrix=k,
          sensor_name="fixture",
          camera_t_sensor=(
              None
              if missing_pose
              else (pose if pose is not None else pose3.Pose3())
          ),
          distortion_params=distortion,
          proto=types.SimpleNamespace(
              buffer=types.SimpleNamespace(pixel_type=kind)
          ),
      )

    sensors = {
        "rgb": sensor(rgb, image_buffer_pb2.PIXEL_INTENSITY, rgb_k, rgb_pose),
        "depth": sensor(
            depth, image_buffer_pb2.PIXEL_DEPTH, depth_k, depth_pose
        ),
    }
    request = types.SimpleNamespace(
        HasField=lambda name: name == "capture_result",
        capture_result=types.SimpleNamespace(sensor_images=sensors),
    )
    return service._extract_rgb_depth_and_intrinsics(request, None)

  def test_uses_both_intrinsics_and_outputs_rgb_shape(self):
    k = np.diag([2.0, 1.0, 1.0])
    rgb, depth, intrinsics = self.extract(k, np.eye(3))
    self.assertEqual(rgb.shape, (2, 5, 3))
    np.testing.assert_array_equal(depth, [[1, 2, 2, 3, 3]] * 2)
    np.testing.assert_array_equal(intrinsics, k)

  def test_extrinsics_are_relative_to_rgb_sensor(self):
    _, depth, _ = self.extract(
        np.eye(3),
        np.eye(3),
        rgb_pose=pose3.Pose3(translation=[3, 0, 0]),
        depth_pose=pose3.Pose3(translation=[5, 0, 0]),
    )
    np.testing.assert_array_equal(depth, [[0, 0, 1, 3, 0]] * 2)

  def test_missing_calibration_and_distortion_fail_explicitly(self):
    for kwargs in ({"missing_pose": True}, {"distortion": np.array([0.1])}):
      with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
        self.extract(np.eye(3), np.eye(3), **kwargs)
    with self.assertRaisesRegex(ValueError, "Depth sensor intrinsics"):
      self.extract(np.eye(3), None)


if __name__ == "__main__":
  unittest.main()
