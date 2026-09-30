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

load("//bazel:cc_macros.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "trajectory_execution_utils",
    srcs = ["trajectory_execution_utils.cc"],
    hdrs = ["trajectory_execution_utils.h"],
    deps = [
        "//intrinsic/stats:scoped_span",
        "//intrinsic/util:proto_time",
        "//intrinsic/util/grpc:channel_interface",
        "//intrinsic/util/proto:any",
        "//intrinsic/util/status:status_macros",
        "//intrinsic/util/thread",
        "//intrinsic_control/intrinsic/icon/actions:adio_info",
        "//intrinsic_control/intrinsic/icon/actions:trajectory_tracking_action_info",
        "//intrinsic_control/intrinsic/icon/cc_client:client",
        "//intrinsic_control/intrinsic/icon/cc_client:client_utils",
        "//intrinsic_control/intrinsic/icon/cc_client:condition",
        "//intrinsic_control/intrinsic/icon/cc_client:session",
        "//intrinsic_control/intrinsic/icon/common:builtins",
        "//intrinsic_control/intrinsic/icon/common:id_types",
        "//intrinsic_control/intrinsic/icon/equipment:equipment_utils",
        "//intrinsic_sdk/intrinsic/skills/cc:skill_canceller",
        "@abseil-cpp//absl/cleanup",
        "@abseil-cpp//absl/container:flat_hash_set",
        "@abseil-cpp//absl/log",
        "@abseil-cpp//absl/status",
        "@abseil-cpp//absl/status:statusor",
        "@abseil-cpp//absl/strings",
        "@abseil-cpp//absl/time",
        "@abseil-cpp//absl/types:span",
        "@intrinsic_apis//intrinsic/icon/proto:joint_space_cc_proto",
        "@intrinsic_apis//intrinsic/icon/proto/v1:condition_types_cc_proto",
        "@intrinsic_apis//intrinsic/logging/proto:context_cc_proto",
    ] + [],
)
