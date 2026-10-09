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

"""Model-independent, in-process instance segmentation contract.

This is not a service wire format. Model weights remain in the inference service.
"""

from dataclasses import dataclass
from typing import Protocol

import numpy as np


@dataclass(frozen=True)
class SegmentationResult:
  """Instances in the original RGB image grid, in matching instance order.

  Boxes are (N, 4) pixel xyxy coordinates; masks are (N, H, W) binary masks.
  Scores are (N,) backend detection scores, not calibrated probabilities.
  Visibility is optional: backends must not fabricate it. Visualization, when
  requested, is an annotated (H, W, 3) uint8 image. Arrays are not deep-frozen.
  """

  boxes: np.ndarray
  scores: np.ndarray
  masks: np.ndarray
  visibility: np.ndarray | None
  visualization: np.ndarray | None = None


class Segmenter(Protocol):
  """Produces instances without exposing model tensors to the caller."""

  def segment(
      self,
      image: np.ndarray,
      *,
      confidence_threshold: float,
      visibility_threshold: float,
      return_vis: bool = False,
  ) -> SegmentationResult:
    """Segments an RGB (H, W, 3) uint8 image.

    Thresholds retain the existing RF-DETR semantics in this first slice.
    A future backend without visibility must reject an unsupported visibility
    policy explicitly. Empty detections are valid; inference errors propagate.
    """
    ...
