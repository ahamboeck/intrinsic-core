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

"""Composition boundary for the currently supported segmentation backend."""

from intrinsic.assets.proto.v1 import resolved_dependency_pb2
from specification.protocol import open_inference_grpc_pb2_grpc

from intrinsic_perception.intrinsic.perception.service.ioc_pose_estimator.service import rfdetr_segmenter
from intrinsic_perception.intrinsic.perception.service.ioc_pose_estimator.service import segmenter


def create_segmenter(
    ml_service_stub: open_inference_grpc_pb2_grpc.GRPCInferenceServiceStub,
    model_dependency: resolved_dependency_pb2.ResolvedDependency,
) -> segmenter.Segmenter:
  """Builds RF-DETR without changing existing configuration or defaults."""
  return rfdetr_segmenter.RfDetrSegmenter(
      ml_service_stub=ml_service_stub,
      model_dependency=model_dependency,
  )
