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

#include "intrinsic/perception/proto/conversion/distortion_params.h"

#include "intrinsic/perception/core/distortion_params.h"
#include "intrinsic/perception/proto/distortion_params.pb.h"

namespace intrinsic_proto::perception {

intrinsic::perception::DistortionParams FromProto(
    const DistortionParams& distortion_params) {
  return intrinsic::perception::DistortionParams{.k1 = distortion_params.k1(),
                                                 .k2 = distortion_params.k2(),
                                                 .p1 = distortion_params.p1(),
                                                 .p2 = distortion_params.p2(),
                                                 .k3 = distortion_params.k3(),
                                                 .k4 = distortion_params.k4(),
                                                 .k5 = distortion_params.k5(),
                                                 .k6 = distortion_params.k6(),
                                                 .s1 = distortion_params.s1(),
                                                 .s2 = distortion_params.s2(),
                                                 .s3 = distortion_params.s3(),
                                                 .s4 = distortion_params.s4(),
                                                 .tx = distortion_params.tx(),
                                                 .ty = distortion_params.ty()};
}

DistortionParams ToProto(
    const intrinsic::perception::DistortionParams& distortion_params) {
  DistortionParams proto;
  proto.set_k1(distortion_params.k1);
  proto.set_k2(distortion_params.k2);
  proto.set_p1(distortion_params.p1);
  proto.set_p2(distortion_params.p2);
  // If k3 is non-zero, set it.
  if (distortion_params.k3 != 0.0) {
    proto.set_k3(distortion_params.k3);
  }
  // If any of k4, k5, k6 is non-zero, set all of them.
  if (distortion_params.k4 != 0.0 || distortion_params.k5 != 0.0 ||
      distortion_params.k6 != 0.0) {
    proto.set_k4(distortion_params.k4);
    proto.set_k5(distortion_params.k5);
    proto.set_k6(distortion_params.k6);
  }
  // If any of s1, s2, s3, s4 is non-zero, set all of them.
  if (distortion_params.s1 != 0.0 || distortion_params.s2 != 0.0 ||
      distortion_params.s3 != 0.0 || distortion_params.s4 != 0.0) {
    proto.set_s1(distortion_params.s1);
    proto.set_s2(distortion_params.s2);
    proto.set_s3(distortion_params.s3);
    proto.set_s4(distortion_params.s4);
  }
  // If either tx or ty is non-zero, set both of them.
  if (distortion_params.tx != 0.0 || distortion_params.ty != 0.0) {
    proto.set_tx(distortion_params.tx);
    proto.set_ty(distortion_params.ty);
  }
  return proto;
}

}  // namespace intrinsic_proto::perception
