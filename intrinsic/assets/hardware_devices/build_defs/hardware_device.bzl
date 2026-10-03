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

load(
    "//intrinsic_sdk/intrinsic/assets/hardware_devices/build_defs:hardware_device.bzl",
    _HardwareDeviceAssetInfo = "HardwareDeviceAssetInfo",
    _intrinsic_hardware_device = "intrinsic_hardware_device",
)

HardwareDeviceAssetInfo = _HardwareDeviceAssetInfo
intrinsic_hardware_device = _intrinsic_hardware_device
