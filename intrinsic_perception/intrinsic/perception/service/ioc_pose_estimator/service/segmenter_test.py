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

"""Regression tests for the segmentation boundary and RF-DETR adapter."""

import unittest
from unittest import mock

import numpy as np
from specification.protocol import open_inference_grpc_pb2
from triton_common.protobuf import model_config_pb2

from intrinsic.assets.proto.v1 import resolved_dependency_pb2
from intrinsic_perception.intrinsic.perception.service.ioc_pose_estimator.service import ioc_pose_estimator_service
from intrinsic_perception.intrinsic.perception.service.ioc_pose_estimator.service import rfdetr_segmenter
from intrinsic_perception.intrinsic.perception.service.ioc_pose_estimator.service import segmentation_model
from intrinsic_perception.intrinsic.perception.service.ioc_pose_estimator.service import segmenter
from intrinsic_perception.intrinsic.perception.service.ioc_pose_estimator.service import segmenter_factory


class SegmenterTest(unittest.TestCase):

  def setUp(self):
    self.stub = mock.Mock()
    self.stub.ModelReady.return_value = (
        open_inference_grpc_pb2.ModelReadyResponse(ready=True)
    )
    self.dependency = resolved_dependency_pb2.ResolvedDependency()
    interface = self.dependency.interfaces[
        "data://intrinsic_proto.ml.inference_service.v1.MlModelAsset"
    ]
    interface.data.id.package = "ai.intrinsic"
    interface.data.id.name = "test_segmenter"
    config = model_config_pb2.ModelConfig()
    for name in ("boxes", "scores", "masks", "visibility"):
      config.output.add(name=name)
    self.adapter = rfdetr_segmenter.RfDetrSegmenter(
        self.stub, self.dependency, triton_config=config
    )
    self.image = np.arange(2 * 4 * 3, dtype=np.uint8).reshape(2, 4, 3)

  def test_original_image_becomes_existing_rfdetr_wire_request(self):
    outputs = {
        "boxes": np.array([[0, 0, 3, 1]], dtype=np.float32),
        "scores": np.array([0.9], dtype=np.float32),
        "masks": np.ones((1, 2, 4), dtype=bool),
        "visibility": np.array([0.8], dtype=np.float32),
    }
    with mock.patch.object(
        rfdetr_segmenter.oip_utils,
        "extract_np_tensor_from_oip_response",
        side_effect=lambda name, response: outputs[name],
    ):
      result = self.adapter.segment(
          self.image, confidence_threshold=0.6, visibility_threshold=0.7
      )

    request = self.stub.ModelInfer.call_args.args[0]
    self.assertEqual(request.model_name, "ai.intrinsic.test_segmenter")
    self.assertEqual([x.name for x in request.inputs], ["input", "thresholds"])
    self.assertEqual(list(request.inputs[0].shape), [1, 3, 2, 4])
    self.assertEqual(request.inputs[0].datatype, "UINT8")
    # Verify channel and pixel ordering, not just the resulting tensor shape.
    actual_image = np.frombuffer(request.raw_input_contents[0], np.uint8)
    np.testing.assert_array_equal(actual_image[:8], self.image[:, :, 0].ravel())
    np.testing.assert_array_equal(actual_image[8:16], self.image[:, :, 1].ravel())
    np.testing.assert_array_equal(actual_image[16:], self.image[:, :, 2].ravel())
    self.assertEqual(list(request.inputs[1].shape), [1, 2])
    self.assertEqual(request.inputs[1].datatype, "FP32")
    np.testing.assert_allclose(
        np.frombuffer(request.raw_input_contents[1], np.float32), [0.6, 0.7]
    )
    self.assertEqual([x.name for x in request.outputs], list(outputs))
    for name, value in outputs.items():
      self.assertIs(getattr(result, name), value)
    self.assertIsNone(result.visualization)

  def test_empty_detections_and_requested_visualization_are_preserved(self):
    boxes = np.empty((0, 4), dtype=np.float32)
    scores = np.empty((0,), dtype=np.float32)
    masks = np.empty((0, 2, 4), dtype=bool)
    visibility = np.empty((0,), dtype=np.float32)
    with mock.patch.object(
        self.adapter, "run_inference",
        return_value=(boxes, scores, masks, visibility, self.image),
    ) as infer:
      result = self.adapter.segment(
          self.image,
          confidence_threshold=0.6,
          visibility_threshold=0.7,
          return_vis=True,
      )
    self.assertEqual(result.masks.shape, (0, 2, 4))
    self.assertIs(result.visualization, self.image)
    self.assertTrue(infer.call_args.kwargs["return_vis"])

  def test_inference_failure_propagates(self):
    self.stub.ModelInfer.side_effect = RuntimeError("inference unavailable")
    with self.assertRaisesRegex(RuntimeError, "inference unavailable"):
      self.adapter.segment(
          self.image, confidence_threshold=0.6, visibility_threshold=0.7
      )

  def test_legacy_tensor_adapter_name_is_preserved(self):
    self.assertIs(
        segmentation_model.SegmentationModel,
        rfdetr_segmenter.RfDetrSegmenter,
    )

  def test_factory_preserves_rfdetr_configuration(self):
    with mock.patch.object(
        segmenter_factory.rfdetr_segmenter, "RfDetrSegmenter"
    ) as constructor:
      result = segmenter_factory.create_segmenter(self.stub, self.dependency)
    constructor.assert_called_once_with(
        ml_service_stub=self.stub, model_dependency=self.dependency
    )
    self.assertIs(result, constructor.return_value)

  def test_pipeline_accepts_segmenter_without_rfdetr_tensor_api(self):
    result = segmenter.SegmentationResult(
        boxes=np.empty((0, 4)),
        scores=np.empty((0,)),
        masks=np.empty((0, 2, 4), dtype=bool),
        visibility=None,
        visualization=self.image,
    )

    class FakeSegmenter:
      def segment(inner_self, image, **kwargs):
        self.assertIs(image, self.image)
        self.assertEqual(
            kwargs,
            dict(
                confidence_threshold=0.6,
                visibility_threshold=0.7,
                return_vis=True,
            ),
        )
        return result

    service = object.__new__(ioc_pose_estimator_service.IocPoseEstimatorService)
    service.segmentation_model = FakeSegmenter()
    masks, visualization = service._run_segmentation_model(self.image, 0.6, 0.7)
    self.assertIs(masks, result.masks)
    self.assertIs(visualization, self.image)


if __name__ == "__main__":
  unittest.main()
