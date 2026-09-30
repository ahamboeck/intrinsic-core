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

#include "intrinsic/perception/proto/conversion/pixel_type.h"

#include "intrinsic/perception/core/pixel_type.h"
#include "intrinsic/perception/proto/image_buffer.pb.h"

namespace intrinsic_proto::perception {

intrinsic::perception::PixelType FromProto(PixelType pixel_type) {
  switch (pixel_type) {
    case PixelType::PIXEL_INTENSITY:
      return intrinsic::perception::PixelType::kIntensity;
    case PixelType::PIXEL_DEPTH:
      return intrinsic::perception::PixelType::kDepth;
    case PixelType::PIXEL_POINT:
      return intrinsic::perception::PixelType::kPoint;
    case PixelType::PIXEL_NORMAL:
      return intrinsic::perception::PixelType::kNormal;
    case PixelType::PIXEL_UNSPECIFIED:
    default:
      return intrinsic::perception::PixelType::kUnspecified;
  }
}

PixelType ToProto(intrinsic::perception::PixelType pixel_type) {
  switch (pixel_type) {
    case intrinsic::perception::PixelType::kIntensity:
      return PixelType::PIXEL_INTENSITY;
    case intrinsic::perception::PixelType::kDepth:
      return PixelType::PIXEL_DEPTH;
    case intrinsic::perception::PixelType::kPoint:
      return PixelType::PIXEL_POINT;
    case intrinsic::perception::PixelType::kNormal:
      return PixelType::PIXEL_NORMAL;
    case intrinsic::perception::PixelType::kUnspecified:
    default:
      return PixelType::PIXEL_UNSPECIFIED;
  }
}

}  // namespace intrinsic_proto::perception
