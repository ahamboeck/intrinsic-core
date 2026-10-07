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

package(
    default_visibility = ["//visibility:public"],
)

alias(
    name = "demangle",
    actual = "//intrinsic_sdk/intrinsic/util:demangle",
    tags = ["go/intrinsic-ioc-transition-tdd"],
)

alias(
    name = "hash",
    actual = "//intrinsic_sdk/intrinsic/util:hash",
    tags = ["go/intrinsic-ioc-transition-tdd"],
)

alias(
    name = "aggregate_type",
    actual = "//intrinsic_sdk/intrinsic/util:aggregate_type",
    tags = ["go/intrinsic-ioc-transition-tdd"],
)

alias(
    name = "atomic_sequence_num",
    actual = "//intrinsic_sdk/intrinsic/util:atomic_sequence_num",
    tags = ["go/intrinsic-ioc-transition-tdd"],
)

cc_library(
    name = "cyclic_runner",
    srcs = ["cyclic_runner.cc"],
    hdrs = ["cyclic_runner.h"],
    deps = [
        "//intrinsic/util/time",
        "//intrinsic/util/time:timer",
        "@abseil-cpp//absl/base:core_headers",
        "@abseil-cpp//absl/status",
        "@abseil-cpp//absl/strings",
        "@abseil-cpp//absl/synchronization",
        "@abseil-cpp//absl/time",
    ],
)

cc_library(
    name = "file_helpers",
    srcs = ["file_helpers.cc"],
    hdrs = ["file_helpers.h"],
    deps = [
        "//intrinsic/util/status:status_macros",
        "@abseil-cpp//absl/status",
        "@abseil-cpp//absl/status:statusor",
        "@abseil-cpp//absl/strings",
        "@com_google_protobuf//:protobuf",
        "@or_tools//ortools/base:file",
    ],
)

alias(
    name = "fixed_vector",
    actual = "//intrinsic_sdk/intrinsic/util:fixed_vector",
    tags = ["go/intrinsic-ioc-transition-tdd"],
)

alias(
    name = "invalid_until_set",
    actual = "//intrinsic_sdk/intrinsic/util:invalid_until_set",
    tags = ["go/intrinsic-ioc-transition-tdd"],
)

alias(
    name = "macros",
    actual = "//intrinsic_sdk/intrinsic/util:macros",
    tags = ["go/intrinsic-ioc-transition-tdd"],
)

alias(
    name = "page_fault_info",
    actual = "//intrinsic_sdk/intrinsic/util:page_fault_info",
    tags = ["go/intrinsic-ioc-transition-tdd"],
)

cc_library(
    name = "unique_id",
    srcs = ["unique_id.cc"],
    hdrs = ["unique_id.h"],
    deps = [
        "@abseil-cpp//absl/random",
        "@abseil-cpp//absl/strings:str_format",
        "@abseil-cpp//absl/strings:string_view",
    ],
)

alias(
    name = "decorators",
    actual = "//intrinsic_sdk/intrinsic/util:decorators",
    tags = ["go/intrinsic-ioc-transition-tdd"],
)

alias(
    name = "eigen",
    actual = "//intrinsic_sdk/intrinsic/util:eigen",
    tags = ["go/intrinsic-ioc-transition-tdd"],
)

cc_library(
    name = "functional",
    hdrs = [
        "functional.h",
        "functional_internal.h",
    ],
    # This is legacy code that is hard to read. Semi-deprecated and restricted to existing users.
    visibility = ["//intrinsic_control/intrinsic/icon/reflexxes:__pkg__"],
)

cc_library(
    name = "full_precision",
    hdrs = ["full_precision.h"],
    deps = ["@abseil-cpp//absl/strings:str_format"],
)

alias(
    name = "lru_cache",
    actual = "//intrinsic_sdk/intrinsic/util:lru_cache",
    tags = ["go/intrinsic-ioc-transition-tdd"],
)

alias(
    name = "memory_lock",
    actual = "//intrinsic_sdk/intrinsic/util:memory_lock",
    tags = ["go/intrinsic-ioc-transition-tdd"],
)

alias(
    name = "string_type",
    actual = "//intrinsic_sdk/intrinsic/util:string_type",
    tags = ["go/intrinsic-ioc-transition-tdd"],
)

alias(
    name = "proto_time",
    actual = "//intrinsic_sdk/intrinsic/util:proto_time",
    tags = ["go/intrinsic-ioc-transition-tdd"],
)

alias(
    name = "cloud",
    actual = "//intrinsic_sdk/intrinsic/util:cloud",
    tags = ["go/intrinsic-ioc-transition-tdd"],
)

cc_library(
    name = "testcallback",
    srcs = ["testcallback.cc"],
    hdrs = ["testcallback.h"],
    deps = [
        "//intrinsic/util/status:status_macros",
        "@abseil-cpp//absl/base:core_headers",
        "@abseil-cpp//absl/base:no_destructor",
        "@abseil-cpp//absl/container:flat_hash_map",
        "@abseil-cpp//absl/functional:any_invocable",
        "@abseil-cpp//absl/status",
        "@abseil-cpp//absl/status:statusor",
        "@abseil-cpp//absl/strings",
        "@abseil-cpp//absl/strings:string_view",
        "@abseil-cpp//absl/synchronization",
    ],
)

alias(
    name = "log_lines",
    actual = "//intrinsic_sdk/intrinsic/util:log_lines",
    tags = ["go/intrinsic-ioc-transition-tdd"],
)

alias(
    name = "flag",
    actual = "//intrinsic_sdk/intrinsic/util:flag",
    tags = ["go/intrinsic-ioc-transition-tdd"],
)

alias(
    name = "version",
    actual = "//intrinsic_sdk/intrinsic/util:version",
    tags = ["go/intrinsic-ioc-transition-tdd"],
)
