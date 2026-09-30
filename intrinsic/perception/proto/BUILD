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

# This package contains public protos that are exposed to clients of Intrinsic perception.

load("@com_google_protobuf//bazel:cc_proto_library.bzl", "cc_proto_library")
load("@com_google_protobuf//bazel:py_proto_library.bzl", "py_proto_library")
load("//bazel:go_macros.bzl", "go_proto_library")
load("//bazel:protos.bzl", "proto_library")

proto_library(
    name = "camera_params_proto",
    srcs = ["camera_params.proto"],
    visibility = ["//intrinsic:public_api_users"],
    deps = [
        ":distortion_params_proto",
        ":intrinsic_params_proto",
    ],
)

py_proto_library(
    name = "camera_params_py_pb2",
    visibility = ["//intrinsic:public_api_users"],
    deps = [":camera_params_proto"],
)

proto_library(
    name = "dimensions_proto",
    srcs = ["dimensions.proto"],
    visibility = ["//intrinsic:public_api_users"],
)

go_proto_library(
    name = "dimensions_go_proto",
    importpath = "intrinsic/perception/proto/dimensions_go_proto",
    protos = [":dimensions_proto"],
    visibility = ["//intrinsic:public_api_users"],
)

proto_library(
    name = "distortion_params_proto",
    srcs = ["distortion_params.proto"],
    visibility = ["//intrinsic:public_api_users"],
)

cc_proto_library(
    name = "distortion_params_cc_proto",
    visibility = ["//intrinsic:public_api_users"],
    deps = [":distortion_params_proto"],
)

py_proto_library(
    name = "distortion_params_py_pb2",
    visibility = ["//intrinsic:public_api_users"],
    deps = [":distortion_params_proto"],
)

go_proto_library(
    name = "distortion_params_go_proto",
    importpath = "intrinsic/perception/proto/distortion_params_go_proto",
    protos = [":distortion_params_proto"],
    visibility = ["//intrinsic:public_api_users"],
)

proto_library(
    name = "intrinsic_params_proto",
    srcs = ["intrinsic_params.proto"],
    visibility = ["//intrinsic:public_api_users"],
    deps = [":dimensions_proto"],
)

cc_proto_library(
    name = "intrinsic_params_cc_proto",
    visibility = ["//intrinsic:public_api_users"],
    deps = [":intrinsic_params_proto"],
)

py_proto_library(
    name = "intrinsic_params_py_pb2",
    visibility = ["//intrinsic:public_api_users"],
    deps = [":intrinsic_params_proto"],
)

go_proto_library(
    name = "intrinsic_params_go_proto",
    importpath = "intrinsic/perception/proto/intrinsic_params_go_proto",
    protos = [":intrinsic_params_proto"],
    visibility = ["//intrinsic:public_api_users"],
    deps = [":dimensions_go_proto"],
)

proto_library(
    name = "image_buffer_proto",
    srcs = ["image_buffer.proto"],
    visibility = ["//intrinsic:public_api_users"],
    deps = [":dimensions_proto"],
)

proto_library(
    name = "sensor_config",
    srcs = ["sensor_config.proto"],
    visibility = ["//intrinsic:public_api_users"],
    deps = [
        ":camera_params_proto",
        "@intrinsic_apis//intrinsic/math/proto:pose_proto",
    ],
)

py_proto_library(
    name = "sensor_config_py_pb2",
    visibility = ["//intrinsic:public_api_users"],
    deps = [":sensor_config"],
)

cc_proto_library(
    name = "sensor_config_cc_proto",
    visibility = ["//intrinsic:public_api_users"],
    deps = ["sensor_config"],
)

go_proto_library(
    name = "sensor_config_go_proto",
    importpath = "intrinsic/perception/proto/sensor_config_go_proto",
    protos = [":sensor_config"],
    visibility = ["//intrinsic:public_api_users"],
    deps = [
        ":camera_params_go_proto",
        "@intrinsic_apis//intrinsic/math/proto:pose_go_proto",
    ],
)

proto_library(
    name = "sensor_image",
    srcs = ["sensor_image.proto"],
    visibility = ["//intrinsic:public_api_users"],
    deps = [
        ":image_buffer_proto",
        ":sensor_config",
        "@com_google_protobuf//:timestamp_proto",
    ],
)

py_proto_library(
    name = "sensor_image_py_pb2",
    visibility = ["//intrinsic:public_api_users"],
    deps = [":sensor_image"],
)

cc_proto_library(
    name = "sensor_image_cc_proto",
    visibility = ["//intrinsic:public_api_users"],
    deps = ["sensor_image"],
)

go_proto_library(
    name = "sensor_image_go_proto",
    importpath = "intrinsic/perception/proto/sensor_image_go_proto",
    protos = [":sensor_image"],
    visibility = ["//intrinsic:internal_api_users"],
    deps = [
        ":image_buffer_go_proto",
        ":sensor_config_go_proto",
    ],
)

proto_library(
    name = "capture_result_proto",
    srcs = ["capture_result.proto"],
    visibility = ["//intrinsic:public_api_users"],
    deps = [
        ":sensor_image",
        "@com_google_protobuf//:duration_proto",
        "@com_google_protobuf//:timestamp_proto",
    ],
)

cc_proto_library(
    name = "capture_result_cc_proto",
    visibility = ["//intrinsic:public_api_users"],
    deps = [":capture_result_proto"],
)

py_proto_library(
    name = "capture_result_py_pb2",
    visibility = ["//intrinsic:public_api_users"],
    deps = [":capture_result_proto"],
)

go_proto_library(
    name = "capture_result_go_proto",
    importpath = "intrinsic/perception/proto/capture_result_go_proto",
    protos = [":capture_result_proto"],
    visibility = ["//intrinsic:internal_api_users"],
    deps = [":sensor_image_go_proto"],
)

py_proto_library(
    name = "image_buffer_py_pb2",
    visibility = ["//intrinsic:public_api_users"],
    deps = [":image_buffer_proto"],
)

go_proto_library(
    name = "image_buffer_go_proto",
    importpath = "intrinsic/perception/proto/image_buffer_go_proto",
    protos = [":image_buffer_proto"],
    visibility = ["//intrinsic:public_api_users"],
    deps = [":dimensions_go_proto"],
)

cc_proto_library(
    name = "camera_params_cc_proto",
    visibility = ["//intrinsic:public_api_users"],
    deps = [":camera_params_proto"],
)

go_proto_library(
    name = "camera_params_go_proto",
    importpath = "intrinsic/perception/proto/camera_params_go_proto",
    protos = [":camera_params_proto"],
    visibility = ["//intrinsic:public_api_users"],
    deps = [
        ":distortion_params_go_proto",
        ":intrinsic_params_go_proto",
    ],
)

cc_proto_library(
    name = "dimensions_cc_proto",
    visibility = ["//intrinsic:internal_api_users"],
    deps = [":dimensions_proto"],
)

py_proto_library(
    name = "dimensions_py_pb2",
    visibility = ["//intrinsic:public_api_users"],
    deps = [":dimensions_proto"],
)

cc_proto_library(
    name = "image_buffer_cc_proto",
    visibility = ["//intrinsic:internal_api_users"],
    deps = ["image_buffer_proto"],
)
