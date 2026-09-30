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

#include "intrinsic/perception/proto/conversion/intrinsic_params.h"

#include "intrinsic/perception/core/intrinsic_params.h"
#include "intrinsic/perception/proto/conversion/dimensions.h"
#include "intrinsic/perception/proto/intrinsic_params.pb.h"

namespace intrinsic_proto::perception {

intrinsic::perception::IntrinsicParams FromProto(
    const IntrinsicParams& intrinsic_params) {
  return {FromProto(intrinsic_params.dimensions()),
          intrinsic_params.focal_length_x(), intrinsic_params.focal_length_y(),
          intrinsic_params.principal_point_x(),
          intrinsic_params.principal_point_y()};
}

IntrinsicParams ToProto(
    const intrinsic::perception::IntrinsicParams& intrinsics) {
  IntrinsicParams proto;
  proto.set_focal_length_x(intrinsics.focal_length_x());
  proto.set_focal_length_y(intrinsics.focal_length_y());
  proto.set_principal_point_x(intrinsics.principal_point_x());
  proto.set_principal_point_y(intrinsics.principal_point_y());
  *proto.mutable_dimensions() = ToProto(intrinsics.dimensions());
  return proto;
}

}  // namespace intrinsic_proto::perception
