// Copyright 2026 Intrinsic Innovation LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "intrinsic/perception/proto/conversion/sensor_image.h"

#include <optional>
#include <utility>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/time/time.h"
#include "google/protobuf/timestamp.pb.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/math/proto_conversion.h"
#include "intrinsic/perception/cameras/sensor_image.h"
#include "intrinsic/perception/core/camera_params.h"
#include "intrinsic/perception/core/encoding.h"
#include "intrinsic/perception/core/image.h"
#include "intrinsic/perception/core/image_traits.h"
#include "intrinsic/perception/proto/conversion/camera_params.h"
#include "intrinsic/perception/proto/conversion/image_buffer.h"
#include "intrinsic/perception/proto/image_buffer.pb.h"
#include "intrinsic/perception/proto/sensor_config.pb.h"
#include "intrinsic/perception/proto/sensor_image.pb.h"
#include "intrinsic/util/proto_time.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic_proto::perception {

absl::StatusOr<intrinsic::perception::SensorImage> FromProto(
    const SensorImage& sensor_image) {
  const ImageBuffer& image_buffer = sensor_image.buffer();
  if (image_buffer.packing_type() == PACKING_TYPE_UNSPECIFIED) {
    return absl::InvalidArgumentError("Invalid packing type");
  }

  INTR_ASSIGN_OR_RETURN(const absl::Time acquisition_time,
                        intrinsic::ToAbslTime(sensor_image.acquisition_time()));

  std::optional<intrinsic::perception::CameraParams> camera_params;
  std::optional<intrinsic::Pose3d> camera_t_sensor;
  if (sensor_image.sensor_config().has_camera_params()) {
    camera_params = FromProto(sensor_image.sensor_config().camera_params());
  }
  if (sensor_image.sensor_config().has_camera_t_sensor()) {
    INTR_ASSIGN_OR_RETURN(camera_t_sensor,
                          intrinsic_proto::FromProto(
                              sensor_image.sensor_config().camera_t_sensor()));
  }

  if (image_buffer.type() == DataType::TYPE_8U) {
    if (image_buffer.pixel_type() != PixelType::PIXEL_INTENSITY) {
      return absl::InvalidArgumentError("8U image must have Intensity pixels");
    }
    if (image_buffer.num_channels() == 3) {
      INTR_ASSIGN_OR_RETURN(
          intrinsic::perception::Image<intrinsic::perception::Rgb8u> rgb8u,
          FromProto<intrinsic::perception::Rgb8u>(image_buffer));
      return intrinsic::perception::SensorImage(
          sensor_image.sensor_config().id(), acquisition_time, camera_params,
          camera_t_sensor, std::move(rgb8u));
    } else if (image_buffer.num_channels() == 1) {
      INTR_ASSIGN_OR_RETURN(
          intrinsic::perception::Image<intrinsic::perception::Gray8u> gray8u,
          FromProto<intrinsic::perception::Gray8u>(image_buffer));
      return intrinsic::perception::SensorImage(
          sensor_image.sensor_config().id(), acquisition_time, camera_params,
          camera_t_sensor, std::move(gray8u));
    } else {
      return absl::InvalidArgumentError("8U image must have 1 or 3 channels");
    }
  } else if (image_buffer.type() == DataType::TYPE_32F) {
    if (image_buffer.pixel_type() == PixelType::PIXEL_INTENSITY &&
        image_buffer.num_channels() == 1) {
      INTR_ASSIGN_OR_RETURN(
          intrinsic::perception::Image<intrinsic::perception::Gray32f> gray32f,
          FromProto<intrinsic::perception::Gray32f>(image_buffer));
      return intrinsic::perception::SensorImage(
          sensor_image.sensor_config().id(), acquisition_time, camera_params,
          camera_t_sensor, std::move(gray32f));
    } else if (image_buffer.pixel_type() == PIXEL_NORMAL &&
               image_buffer.num_channels() == 3) {
      INTR_ASSIGN_OR_RETURN(
          intrinsic::perception::Image<intrinsic::perception::Normal32f>
              normal32f,
          FromProto<intrinsic::perception::Normal32f>(image_buffer));
      return intrinsic::perception::SensorImage(
          sensor_image.sensor_config().id(), acquisition_time, camera_params,
          camera_t_sensor, std::move(normal32f));
    } else if (image_buffer.pixel_type() == PixelType::PIXEL_DEPTH &&
               image_buffer.num_channels() == 1) {
      INTR_ASSIGN_OR_RETURN(
          intrinsic::perception::Image<intrinsic::perception::Depth32f>
              depth32f,
          FromProto<intrinsic::perception::Depth32f>(image_buffer));
      return intrinsic::perception::SensorImage(
          sensor_image.sensor_config().id(), acquisition_time, camera_params,
          camera_t_sensor, std::move(depth32f));
    } else if (image_buffer.pixel_type() == PixelType::PIXEL_POINT &&
               image_buffer.num_channels() == 3) {
      INTR_ASSIGN_OR_RETURN(
          intrinsic::perception::Image<intrinsic::perception::Point32f>
              point32f,
          FromProto<intrinsic::perception::Point32f>(image_buffer));
      return intrinsic::perception::SensorImage(
          sensor_image.sensor_config().id(), acquisition_time, camera_params,
          camera_t_sensor, std::move(point32f));
    } else {
      return absl::InvalidArgumentError(
          "Invalid pixel type/channel combination");
    }
  } else {
    return absl::InvalidArgumentError("Image must have 8U or 32F data");
  }
}

absl::StatusOr<SensorImage> ToProto(
    const intrinsic::perception::SensorImage& sensor_image,
    intrinsic::perception::Encoding encoding) {
  SensorConfig sensor_config_proto;
  sensor_config_proto.set_id(sensor_image.sensor_id());
  if (sensor_image.camera_params().has_value()) {
    *sensor_config_proto.mutable_camera_params() =
        ToProto(sensor_image.camera_params().value());
  }
  if (sensor_image.camera_t_sensor().has_value()) {
    *sensor_config_proto.mutable_camera_t_sensor() =
        ToProto(sensor_image.camera_t_sensor().value());
  }

  google::protobuf::Timestamp acquisition_time;
  INTR_ASSIGN_OR_RETURN(acquisition_time, intrinsic::FromAbslTime(
                                              sensor_image.acquisition_time()));

  SensorImage sensor_image_proto;
  *sensor_image_proto.mutable_sensor_config() = sensor_config_proto;
  *sensor_image_proto.mutable_acquisition_time() = acquisition_time;
  if (sensor_image.IsEmpty()) {
    return sensor_image_proto;
  }

  ImageBuffer buffer;
  if (sensor_image.rgb8u().has_value()) {
    INTR_ASSIGN_OR_RETURN(buffer, ToProto<intrinsic::perception::Rgb8u>(
                                      sensor_image.rgb8u().value(), encoding));
  } else if (sensor_image.gray8u().has_value()) {
    INTR_ASSIGN_OR_RETURN(buffer, ToProto<intrinsic::perception::Gray8u>(
                                      sensor_image.gray8u().value(), encoding));
  } else if (sensor_image.gray32f().has_value()) {
    INTR_ASSIGN_OR_RETURN(buffer,
                          ToProto<intrinsic::perception::Gray32f>(
                              sensor_image.gray32f().value(), encoding));
  } else if (sensor_image.depth32f().has_value()) {
    INTR_ASSIGN_OR_RETURN(buffer,
                          ToProto<intrinsic::perception::Depth32f>(
                              sensor_image.depth32f().value(), encoding));
  } else if (sensor_image.point32f().has_value()) {
    INTR_ASSIGN_OR_RETURN(buffer,
                          ToProto<intrinsic::perception::Point32f>(
                              sensor_image.point32f().value(), encoding));
  } else if (sensor_image.normal32f().has_value()) {
    INTR_ASSIGN_OR_RETURN(buffer,
                          ToProto<intrinsic::perception::Normal32f>(
                              sensor_image.normal32f().value(), encoding));
  } else {
    return absl::InvalidArgumentError("No image to export!");
  }

  buffer.set_packing_type(PACKING_TYPE_INTERLEAVED);
  *sensor_image_proto.mutable_buffer() = std::move(buffer);

  return sensor_image_proto;
}

}  // namespace intrinsic_proto::perception
