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

package(default_visibility = [
    "//visibility:public",
])

cc_library(
    name = "camera_params",
    srcs = ["camera_params.cc"],
    hdrs = ["camera_params.h"],
    deps = [
        ":distortion_params",
        ":intrinsic_params",
        "//intrinsic/perception/proto:camera_params_cc_proto",
        "//intrinsic_perception/intrinsic/perception/core:camera_params",
        "//intrinsic_perception/intrinsic/perception/core:distortion_params",
    ],
)

cc_library(
    name = "capture_result",
    srcs = ["capture_result.cc"],
    hdrs = ["capture_result.h"],
    deps = [
        ":sensor_image",
        "//intrinsic/perception/proto:capture_result_cc_proto",
        "//intrinsic/perception/proto:sensor_image_cc_proto",
        "//intrinsic/util:proto_time",
        "//intrinsic/util/status:status_macros",
        "//intrinsic_perception/intrinsic/perception/cameras:capture_result",
        "//intrinsic_perception/intrinsic/perception/cameras:sensor_image",
        "//intrinsic_perception/intrinsic/perception/core:encoding",
        "@abseil-cpp//absl/container:flat_hash_map",
        "@abseil-cpp//absl/status:statusor",
        "@abseil-cpp//absl/time",
    ],
)

cc_library(
    name = "dimensions",
    srcs = ["dimensions.cc"],
    hdrs = ["dimensions.h"],
    deps = [
        "//intrinsic/perception/proto:dimensions_cc_proto",
        "//intrinsic_perception/intrinsic/perception/core:dimensions",
    ],
)

cc_library(
    name = "distortion_params",
    srcs = ["distortion_params.cc"],
    hdrs = ["distortion_params.h"],
    deps = [
        "//intrinsic/perception/proto:distortion_params_cc_proto",
        "//intrinsic_perception/intrinsic/perception/core:distortion_params",
    ],
)

cc_library(
    name = "pixel_type",
    srcs = ["pixel_type.cc"],
    hdrs = ["pixel_type.h"],
    deps = [
        "//intrinsic/perception/proto:image_buffer_cc_proto",
        "//intrinsic_perception/intrinsic/perception/core:pixel_type",
    ],
)

cc_library(
    name = "image_buffer",
    srcs = ["image_buffer.cc"],
    hdrs = ["image_buffer.h"],
    deps = [
        ":dimensions",
        ":pixel_type",
        "//intrinsic/perception/proto:dimensions_cc_proto",
        "//intrinsic/perception/proto:image_buffer_cc_proto",
        "//intrinsic/util/status:ret_check",
        "//intrinsic/util/status:status_macros",
        "//intrinsic_perception/intrinsic/perception/core:color_gradient",
        "//intrinsic_perception/intrinsic/perception/core:conversion",
        "//intrinsic_perception/intrinsic/perception/core:encoding",
        "//intrinsic_perception/intrinsic/perception/core:image",
        "//intrinsic_perception/intrinsic/perception/core:image_traits",
        "//intrinsic_perception/intrinsic/perception/core:opencv_wrapper",
        "//intrinsic_perception/intrinsic/perception/core:operators",
        "//intrinsic_perception/intrinsic/perception/core:pixel_type",
        "@abseil-cpp//absl/log",
        "@abseil-cpp//absl/log:check",
        "@abseil-cpp//absl/status",
        "@abseil-cpp//absl/status:statusor",
        "@abseil-cpp//absl/strings",
        "@abseil-cpp//absl/strings:str_format",
        "@abseil-cpp//absl/strings:string_view",
        "@libyuv",
        "@opencv//:core",
        "@opencv//:imgcodecs",
        "@opencv//:imgproc",
    ],
)

cc_library(
    name = "intrinsic_params",
    srcs = ["intrinsic_params.cc"],
    hdrs = ["intrinsic_params.h"],
    deps = [
        ":dimensions",
        "//intrinsic/perception/proto:intrinsic_params_cc_proto",
        "//intrinsic_perception/intrinsic/perception/core:intrinsic_params",
    ],
)

cc_library(
    name = "sensor_image",
    srcs = ["sensor_image.cc"],
    hdrs = ["sensor_image.h"],
    deps = [
        ":camera_params",
        ":image_buffer",
        "//intrinsic/perception/proto:image_buffer_cc_proto",
        "//intrinsic/perception/proto:sensor_config_cc_proto",
        "//intrinsic/perception/proto:sensor_image_cc_proto",
        "//intrinsic/util:proto_time",
        "//intrinsic/util/status:status_macros",
        "//intrinsic_perception/intrinsic/perception/cameras:sensor_image",
        "//intrinsic_perception/intrinsic/perception/core:camera_params",
        "//intrinsic_perception/intrinsic/perception/core:encoding",
        "//intrinsic_perception/intrinsic/perception/core:image",
        "//intrinsic_perception/intrinsic/perception/core:image_traits",
        "//intrinsic_sdk/intrinsic/math:pose3",
        "//intrinsic_sdk/intrinsic/math:proto_conversion",
        "@abseil-cpp//absl/status",
        "@abseil-cpp//absl/status:statusor",
        "@abseil-cpp//absl/time",
        "@com_google_protobuf//:protobuf",
    ],
)
