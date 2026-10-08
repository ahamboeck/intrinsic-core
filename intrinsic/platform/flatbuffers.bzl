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

"""Forwarding definitions to intrinsic_sdk."""

# Forwarding definitions were added in support of go/intrinsic-ioc-transition-tdd.

# buildifier: disable=bzl-visibility
load(
    "//intrinsic_sdk/intrinsic/platform:flatbuffers.bzl",
    _FlatBuffersBinaryInfo = "FlatBuffersBinaryInfo",
    _FlatBuffersInfo = "FlatBuffersInfo",
    _cc_flatbuffers_library = "cc_flatbuffers_library",
    _cc_lite_flatbuffers_library = "cc_lite_flatbuffers_library",
    _flatbuffers_library = "flatbuffers_library",
    _py_flatbuffers_library = "py_flatbuffers_library",
)

FlatBuffersInfo = _FlatBuffersInfo
FlatBuffersBinaryInfo = _FlatBuffersBinaryInfo
cc_flatbuffers_library = _cc_flatbuffers_library
cc_lite_flatbuffers_library = _cc_lite_flatbuffers_library
flatbuffers_library = _flatbuffers_library
py_flatbuffers_library = _py_flatbuffers_library
