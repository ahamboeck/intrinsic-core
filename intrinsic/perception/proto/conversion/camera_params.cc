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

#include "intrinsic/perception/proto/conversion/camera_params.h"

#include <optional>
#include <utility>

#include "intrinsic/perception/core/camera_params.h"
#include "intrinsic/perception/core/distortion_params.h"
#include "intrinsic/perception/proto/camera_params.pb.h"
#include "intrinsic/perception/proto/conversion/distortion_params.h"
#include "intrinsic/perception/proto/conversion/intrinsic_params.h"

namespace intrinsic_proto::perception {

intrinsic::perception::CameraParams FromProto(
    const CameraParams& camera_params) {
  std::optional<intrinsic::perception::DistortionParams> distortion_params;
  if (camera_params.has_distortion_params()) {
    distortion_params = FromProto(camera_params.distortion_params());
  }
  return {FromProto(camera_params.intrinsic_params()),
          std::move(distortion_params)};
}

CameraParams ToProto(const intrinsic::perception::CameraParams& camera_params) {
  intrinsic_proto::perception::CameraParams proto;
  *proto.mutable_intrinsic_params() = ToProto(camera_params.intrinsic_params);
  if (camera_params.distortion_params.has_value()) {
    *proto.mutable_distortion_params() =
        ToProto(camera_params.distortion_params.value());
  }
  return proto;
}

}  // namespace intrinsic_proto::perception
