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

"""Segmentation Model wrapper for IocPoseEstimatorService.

Handles Triton asset installation, readiness checking, and RF-DETR segmentation inference.
"""

import logging
import os
import time
from typing import Optional

import cv2
from google.protobuf import text_format
import grpc
import numpy as np
from specification.protocol import open_inference_grpc_pb2
from specification.protocol import open_inference_grpc_pb2_grpc
from triton_common.protobuf import model_config_pb2

from intrinsic_inference.core.utils import oip_mappings
from intrinsic_inference.core.utils import oip_utils
from incode.util.runfiles import runfiles
from intrinsic.assets import id_utils
from intrinsic.assets.proto.v1 import resolved_dependency_pb2
from intrinsic_perception.intrinsic.perception.service.ioc_pose_estimator.service import segmenter

_CONNECT_INFERENCE_SERVICE_TIMEOUT_SECONDS = 600
_CONNECT_INFERENCE_SERVICE_POLLING_INTERVAL_SECONDS = 20
_ML_MODEL_ASSET_URI = (
    "data://intrinsic_proto.ml.inference_service.v1.MlModelAsset"
)
_DEFAULT_CONFIG_PATH = "intrinsic_perception/intrinsic/perception/service/ioc_pose_estimator/model_files/segmentation/config.pbtxt"


def _load_triton_config(
    config_path: Optional[str] = None,
    default_relative_path: str = _DEFAULT_CONFIG_PATH,
) -> model_config_pb2.ModelConfig:
  """Loads and parses a Triton ModelConfig protobuf text file.

  Args:
    config_path: Explicit path to the config.pbtxt file. If None, resolves via
      runfiles using default_relative_path.
    default_relative_path: Relative runfiles path to use if config_path is None.

  Returns:
    The parsed ModelConfig message.

  Raises:
    FileNotFoundError: If the config file cannot be found.
  """
  resolved_path = config_path
  if resolved_path is None:
    try:
      r_path = runfiles.RlocationCurrentRepository(default_relative_path)
      if r_path and os.path.exists(r_path):
        resolved_path = r_path
      elif os.path.exists(default_relative_path):
        resolved_path = default_relative_path
    except Exception:
      if os.path.exists(default_relative_path):
        resolved_path = default_relative_path

  if not resolved_path or not os.path.exists(resolved_path):
    raise FileNotFoundError(
        f"Triton config file not found at '{resolved_path}' "
        f"(default: '{default_relative_path}')."
    )

  config = model_config_pb2.ModelConfig()
  with open(resolved_path, "r", encoding="utf-8") as f:
    text_format.Parse(f.read(), config)
  return config


def _extract_model_id(
    dependency: resolved_dependency_pb2.ResolvedDependency,
) -> str:
  """Extracts and validates the model asset ID from a ResolvedDependency.

  Args:
    dependency: ResolvedDependency describing the ML model asset.

  Returns:
    The validated model ID string in `<package>.<name>` format.

  Raises:
    ValueError: If dependency is not provided or has no valid data.id interface.
  """
  if not dependency:
    raise ValueError("model_dependency is required and cannot be None.")

  iface = dependency.interfaces.get(_ML_MODEL_ASSET_URI)
  if iface is None:
    for candidate in dependency.interfaces.values():
      if candidate.HasField("data") and candidate.data.HasField("id"):
        iface = candidate
        break

  if (
      iface is None
      or not iface.HasField("data")
      or not iface.data.HasField("id")
  ):
    raise ValueError(
        "ResolvedDependency does not contain a valid data interface with an ID."
        f" Interfaces present: {list(dependency.interfaces.keys())}"
    )

  try:
    return id_utils.id_from_proto(iface.data.id)
  except id_utils.IdValidationError as e:
    raise ValueError(
        "Invalid asset ID in ResolvedDependency data interface:"
        f" {iface.data.id}"
    ) from e


def _create_model_infer_request(
    inputs: dict[str, np.ndarray],
    model_name: str,
    output_names: Optional[list[str]] = None,
) -> open_inference_grpc_pb2.ModelInferRequest:
  """Creates an OIP ModelInferRequest proto with concrete runtime input shapes."""
  requested_outputs = [
      open_inference_grpc_pb2.ModelInferRequest.InferRequestedOutputTensor(
          name=out_name,
      )
      for out_name in output_names or []
  ]
  return open_inference_grpc_pb2.ModelInferRequest(
      model_name=model_name,
      inputs=[
          open_inference_grpc_pb2.ModelInferRequest.InferInputTensor(
              name=k,
              datatype=oip_mappings.numpy_to_oip_type(v.dtype),
              shape=v.shape,
          )
          for k, v in inputs.items()
      ],
      outputs=requested_outputs,
      raw_input_contents=[
          np.ascontiguousarray(v).tobytes() for v in inputs.values()
      ],
  )


class RfDetrSegmenter:
  """Wrapper class around Triton segmentation model (RF-DETR)."""

  def __init__(
      self,
      ml_service_stub: open_inference_grpc_pb2_grpc.GRPCInferenceServiceStub,
      model_dependency: resolved_dependency_pb2.ResolvedDependency,
      config_path: Optional[str] = None,
      triton_config: Optional[model_config_pb2.ModelConfig] = None,
  ):
    """Initializes SegmentationModel.

    Args:
      ml_service_stub: gRPC inference service stub.
      model_dependency: Required ResolvedDependency specifying the model ID.
      config_path: Optional path to the Triton config.pbtxt file.
      triton_config: Optional pre-parsed ModelConfig protobuf message.
    """
    if ml_service_stub is None:
      raise ValueError("ml_service_stub is required.")
    self._ml_service_stub = ml_service_stub
    self._model_id = _extract_model_id(model_dependency)
    self._triton_config = triton_config or _load_triton_config(config_path)
    self._output_names = [out.name for out in self._triton_config.output]

  @property
  def triton_config(self) -> model_config_pb2.ModelConfig:
    """Returns the loaded Triton ModelConfig."""
    return self._triton_config

  @property
  def output_names(self) -> list[str]:
    """Returns the output tensor names parsed from Triton config."""
    return self._output_names

  def wait_for_model_ready(self) -> bool:
    """Polls OIP ModelReady until Triton loads the segmentation model."""
    if self._ml_service_stub is None:
      raise ValueError("ML service stub is None, cannot check model readiness.")

    model_name = self._model_id
    start_time = time.time()

    # loop to check if model is ready
    while time.time() - start_time < _CONNECT_INFERENCE_SERVICE_TIMEOUT_SECONDS:
      model_ready_request = open_inference_grpc_pb2.ModelReadyRequest(
          name=model_name
      )
      try:
        response = self._ml_service_stub.ModelReady(model_ready_request)
        if response.ready:
          logging.info("Segmentation model %s is ready.", model_name)
          return True
      except grpc.RpcError as e:
        logging.debug("Error checking segmentation model readiness: %s", e)
      time.sleep(_CONNECT_INFERENCE_SERVICE_POLLING_INTERVAL_SECONDS)
    logging.warning(
        "Segmentation model %s did not become ready within %d seconds.",
        model_name,
        _CONNECT_INFERENCE_SERVICE_TIMEOUT_SECONDS,
    )
    return False

  def segment(
      self,
      image: np.ndarray,
      *,
      confidence_threshold: float,
      visibility_threshold: float,
      return_vis: bool = False,
  ) -> segmenter.SegmentationResult:
    """Adapts original-image RGB to RF-DETR's batched CHW tensor protocol."""
    image_tensor = np.expand_dims(
        np.transpose(image.astype(np.uint8), (2, 0, 1)), axis=0
    )
    outputs = self.run_inference(
        image_tensor,
        confidence_threshold=confidence_threshold,
        visibility_threshold=visibility_threshold,
        return_vis=return_vis,
    )
    boxes, scores, masks, visibility = outputs[:4]
    return segmenter.SegmentationResult(
        boxes=boxes,
        scores=scores,
        masks=masks,
        visibility=visibility,
        visualization=outputs[4] if return_vis else None,
    )

  def run_inference(
      self,
      image: np.ndarray,
      confidence_threshold: float,
      visibility_threshold: float,
      return_vis: bool = False,
  ) -> (
      tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]
      | tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]
  ):
    """Runs RF-DETR segmentation inference via OIP.

    Args:
      image: RGB image tensor of shape (1, 3, H, W) and uint8 dtype.
      confidence_threshold: Minimum detection confidence threshold in range
        [0.0, 1.0].
      visibility_threshold: Minimum object visibility score threshold in range
        [0.0, 1.0].
      return_vis: If True, returns an annotated OpenCV visualization image as
        the 5th tuple element.

    Returns:
      A tuple of (boxes, scores, masks, visibility) or
      (boxes, scores, masks, visibility, vis) if return_vis is True:
        - boxes: Predicted 2D bounding boxes of shape (N, 4) and float32 dtype
          in pixel coordinates [x1, y1, x2, y2].
        - scores: Detection confidence scores of shape (N,) and float32 dtype in
          the range [0.0, 1.0].
        - masks: Binary segmentation masks of shape (N, H, W) and bool dtype.
        - visibility: Object visibility scores of shape (N,) and float32 dtype
          in the range [0.0, 1.0].
        - vis: (Optional) Annotated image of shape (H, W, 3) and uint8 dtype.

    Raises:
      ValueError: If the ML service stub is None.
    """
    if self._ml_service_stub is None:
      raise ValueError(
          "ML service stub is None, cannot run segmentation inference."
      )

    model_name = self._model_id
    logging.info("Checking if %s is ready...", model_name)
    self.wait_for_model_ready()

    logging.info(
        "running rfdetr with confidence threshold: %s and visibility"
        " threshold: %s",
        confidence_threshold,
        visibility_threshold,
    )
    image_np = np.ascontiguousarray(image, dtype=np.uint8)
    thresh_np = np.ascontiguousarray(
        [[confidence_threshold, visibility_threshold]],
        dtype=np.float32,
    )

    inference_request = _create_model_infer_request(
        {"input": image_np, "thresholds": thresh_np},
        model_name=self._model_id,
        output_names=self._output_names,
    )

    logging.info("Running segmentation inference...")
    start_time = time.perf_counter()
    response = self._ml_service_stub.ModelInfer(inference_request)
    inference_duration = time.perf_counter() - start_time
    logging.info(
        "Segmentation inference completed in %.3f seconds.",
        inference_duration,
    )

    boxes = oip_utils.extract_np_tensor_from_oip_response("boxes", response)
    scores = oip_utils.extract_np_tensor_from_oip_response("scores", response)
    masks = oip_utils.extract_np_tensor_from_oip_response("masks", response)
    visibility = oip_utils.extract_np_tensor_from_oip_response(
        "visibility", response
    )

    if return_vis:
      vis = self._visualize_predictions(image, boxes, scores, masks, visibility)
      return boxes, scores, masks, visibility, vis
    else:
      return boxes, scores, masks, visibility

  def _visualize_predictions(
      self,
      img: np.ndarray,
      boxes: np.ndarray,
      scores: np.ndarray,
      masks: np.ndarray,
      visibility: np.ndarray,
  ) -> np.ndarray:
    """Overlays parsed prediction maps natively via OpenCV directly onto the provided image.

    Args:
      img: Input image numpy array.
      boxes: Detected 2D bounding boxes.
      scores: Detection confidence scores.
      masks: 2D segmentation masks.
      visibility: Object visibility scores.

    Returns:
      A numpy array (H, W, 3) uint8 with visual bounding boxes, masks, and
      labels overlaid.
    """
    img_draw = img.copy()
    if img_draw.ndim == 4:
      img_draw = np.squeeze(img_draw, axis=0)
    if (
        img_draw.ndim == 3
        and img_draw.shape[0] in (1, 3, 4)
        and img_draw.shape[2] not in (1, 3, 4)
    ):
      img_draw = np.transpose(img_draw, (1, 2, 0))
    img_draw = np.ascontiguousarray(img_draw, dtype=np.uint8)

    if len(boxes) == 0:
      return img_draw

    if masks is not None and len(masks) > 0:
      for mask in masks:
        mask_bool = np.asarray(mask) > 0
        color = np.random.randint(0, 255, (1, 1, 3), dtype=np.uint8)
        img_draw[mask_bool] = (img_draw[mask_bool] * 0.5 + color * 0.5).astype(
            np.uint8
        )

    for idx, box in enumerate(boxes):
      x1, y1, x2, y2 = map(int, box)
      score = float(scores[idx]) if idx < len(scores) else 0.0

      cv2.rectangle(img_draw, (x1, y1), (x2, y2), (0, 255, 0), 2)

      label = f"Obj {idx}: {score:.2f}"
      if visibility is not None and idx < len(visibility):
        label += f" | Vis: {float(visibility[idx]):.2f}"

      font = cv2.FONT_HERSHEY_SIMPLEX
      (text_w, text_h), _ = cv2.getTextSize(label, font, 0.4, 1)
      cv2.rectangle(
          img_draw,
          (x1, y1 - text_h - 4),
          (x1 + text_w, y1),
          (0, 255, 0),
          -1,
      )
      cv2.putText(
          img_draw,
          label,
          (x1, max(y1 - 5, 0)),
          font,
          0.4,
          (255, 255, 255),
          1,
          cv2.LINE_AA,
      )

    return img_draw
