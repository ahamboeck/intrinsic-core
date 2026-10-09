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

"""Backward-compatible name for the existing RF-DETR tensor adapter.

New pipeline code should depend on segmenter.Segmenter and call segment(), not
the RF-DETR-specific run_inference() tensor API.
"""

from intrinsic_perception.intrinsic.perception.service.ioc_pose_estimator.service.rfdetr_segmenter import RfDetrSegmenter

SegmentationModel = RfDetrSegmenter
