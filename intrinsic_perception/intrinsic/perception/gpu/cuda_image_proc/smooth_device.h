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

#ifndef INTRINSIC_PERCEPTION_GPU_CUDA_IMAGE_PROC_SMOOTH_DEVICE_H_
#define INTRINSIC_PERCEPTION_GPU_CUDA_IMAGE_PROC_SMOOTH_DEVICE_H_

#include <cuda_runtime.h>

namespace intrinsic::perception {

constexpr int kGaussian5x5BlockSize = 12;
constexpr int kGaussian5x5KernelWidth = 5;
constexpr int kGaussian5x5HalfKernelWidth = kGaussian5x5KernelWidth / 2;
constexpr int kGaussian5x5BlockSizePadded =
    kGaussian5x5BlockSize + 2 * kGaussian5x5HalfKernelWidth;

inline constexpr float kGaussian5x5Kernel1d[kGaussian5x5KernelWidth] = {
    0.0625f, 0.25f, 0.375f, 0.25f, 0.0625f};

// Returns true if the current thread lies within the unpadded interior of a
// `(kGaussian5x5BlockSizePadded x kGaussian5x5BlockSizePadded)` thread block.
__device__ __forceinline__ bool IsGaussian5x5InteriorThread() {
  return threadIdx.x >= kGaussian5x5HalfKernelWidth &&
         threadIdx.x < kGaussian5x5HalfKernelWidth + kGaussian5x5BlockSize &&
         threadIdx.y >= kGaussian5x5HalfKernelWidth &&
         threadIdx.y < kGaussian5x5HalfKernelWidth + kGaussian5x5BlockSize;
}

// Evaluates the 1D 5x5 Gaussian horizontal convolution at
// `(block_col, block_row)` within a shared-memory tile `block_src` of row
// stride `kGaussian5x5BlockSizePadded`.
template <typename ScalarType>
__device__ __forceinline__ float ConvolveGaussian5x5Row(
    const ScalarType* block_src, int block_col, int block_row) {
  float sum = 0.0f;
#pragma unroll
  for (int j = 0; j < kGaussian5x5KernelWidth; ++j) {
    const int block_col_with_offset =
        block_col - kGaussian5x5HalfKernelWidth + j;
    const float val =
        static_cast<float>(block_src[block_row * kGaussian5x5BlockSizePadded +
                                     block_col_with_offset]);
    sum += kGaussian5x5Kernel1d[j] * val;
  }
  return sum;
}

// Evaluates the 1D 5x5 Gaussian vertical convolution at
// `(block_col, block_row)` within a shared-memory tile `block_src` of row
// stride `kGaussian5x5BlockSizePadded`.
__device__ __forceinline__ float ConvolveGaussian5x5Col(const float* block_src,
                                                        int block_col,
                                                        int block_row) {
  float sum = 0.0f;
#pragma unroll
  for (int j = 0; j < kGaussian5x5KernelWidth; ++j) {
    const int block_row_with_offset =
        block_row - kGaussian5x5HalfKernelWidth + j;
    const float val =
        block_src[block_row_with_offset * kGaussian5x5BlockSizePadded +
                  block_col];
    sum += kGaussian5x5Kernel1d[j] * val;
  }
  return sum;
}

}  // namespace intrinsic::perception

#endif  // INTRINSIC_PERCEPTION_GPU_CUDA_IMAGE_PROC_SMOOTH_DEVICE_H_
