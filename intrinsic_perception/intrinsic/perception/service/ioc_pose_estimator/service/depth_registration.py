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

"""Register pinhole axial depth (metres) into the RGB sensor's pixel grid."""

import numpy as np


def _intrinsics(value: np.ndarray) -> np.ndarray:
  k = np.asarray(value, dtype=np.float64)
  if (
      k.shape != (3, 3)
      or not np.all(np.isfinite(k))
      or k[0, 0] <= 0
      or k[1, 1] <= 0
      or not np.allclose(k[2], [0, 0, 1], atol=1e-12, rtol=0)
      or abs(k[0, 1]) > 1e-12
      or abs(k[1, 0]) > 1e-12
  ):
    raise ValueError("Registration requires finite pinhole camera intrinsics.")
  return k


def register_depth_to_rgb(
    depth: np.ndarray,
    depth_k: np.ndarray,
    rgb_k: np.ndarray,
    rgb_shape: tuple[int, int],
    rgb_t_depth: np.ndarray,
) -> np.ndarray:
  """Return RGB-grid axial Z in metres, with zero for missing measurements.

  Both images must be undistorted and have explicit sensor calibration. For
  co-located sensors, inverse nearest-neighbour resampling preserves measured
  depth without forward-splat holes. Otherwise reproject 3D points and retain
  the nearest positive RGB Z at each destination pixel (z-buffer). This does
  not fill occlusion holes, interpolate depths across surfaces, or infer scale.
  """
  depth = np.asarray(depth, dtype=np.float32)
  kd, kr = _intrinsics(depth_k), _intrinsics(rgb_k)
  transform = np.asarray(rgb_t_depth, dtype=np.float64)
  if (
      depth.ndim != 2
      or not depth.size
      or len(rgb_shape) != 2
      or any(type(size) is not int or size <= 0 for size in rgb_shape)
  ):
    raise ValueError("Registration requires nonempty depth and RGB dimensions.")
  if (
      transform.shape != (4, 4)
      or not np.all(np.isfinite(transform))
      or not np.allclose(transform[3], [0, 0, 0, 1], atol=1e-9, rtol=0)
      or not np.allclose(
          transform[:3, :3].T @ transform[:3, :3], np.eye(3), atol=1e-9, rtol=0
      )
      or not np.isclose(np.linalg.det(transform[:3, :3]), 1, atol=1e-9, rtol=0)
  ):
    raise ValueError(
        "Registration requires a proper RGB_T_depth rigid transform."
    )
  clean = np.where(np.isfinite(depth) & (depth > 0), depth, 0)
  height, width = rgb_shape
  if np.allclose(transform, np.eye(4), atol=1e-12, rtol=0):
    if depth.shape == rgb_shape and np.allclose(kd, kr, atol=1e-12, rtol=0):
      return clean
    y, x = np.indices(rgb_shape, dtype=np.float64)
    u = np.floor((x - kr[0, 2]) / kr[0, 0] * kd[0, 0] + kd[0, 2] + 0.5)
    v = np.floor((y - kr[1, 2]) / kr[1, 1] * kd[1, 1] + kd[1, 2] + 0.5)
    valid = (u >= 0) & (u < depth.shape[1]) & (v >= 0) & (v < depth.shape[0])
    result = np.zeros(rgb_shape, dtype=np.float32)
    result[valid] = clean[v[valid].astype(np.int64), u[valid].astype(np.int64)]
    return result

  y, x = np.nonzero(clean > 0)
  z = clean[y, x].astype(np.float64)
  points = np.column_stack(
      ((x - kd[0, 2]) * z / kd[0, 0], (y - kd[1, 2]) * z / kd[1, 1], z)
  )
  points = points @ transform[:3, :3].T + transform[:3, 3]
  points = points[np.all(np.isfinite(points), axis=1) & (points[:, 2] > 0)]
  z = points[:, 2]
  u = np.floor(points[:, 0] / z * kr[0, 0] + kr[0, 2] + 0.5)
  v = np.floor(points[:, 1] / z * kr[1, 1] + kr[1, 2] + 0.5)
  valid = (u >= 0) & (u < width) & (v >= 0) & (v < height)
  result = np.full(height * width, np.inf, dtype=np.float64)
  indices = v[valid].astype(np.int64) * width + u[valid].astype(np.int64)
  np.minimum.at(result, indices, z[valid])
  return (
      np.where(np.isfinite(result), result, 0)
      .reshape(rgb_shape)
      .astype(np.float32)
  )
