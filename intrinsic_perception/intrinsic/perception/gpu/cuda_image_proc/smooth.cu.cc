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

#include "intrinsic/perception/gpu/cuda_image_proc/smooth.h"

#include <cuda_runtime_api.h>
#include <device_types.h>
#include <vector_types.h>

#include "absl/status/statusor.h"
#include "intrinsic/perception/gpu/cuda_image_proc/border_extrapolation.h"
#include "intrinsic/perception/gpu/cuda_image_proc/border_extrapolation_device.h"
#include "intrinsic/perception/gpu/cuda_image_proc/smooth_device.h"
#include "intrinsic/perception/gpu/cuda_utils/cuda_image.h"
#include "intrinsic/perception/gpu/cuda_utils/cuda_math.h"
#include "intrinsic/perception/gpu/cuda_utils/cuda_utils.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic {
namespace perception {
namespace {

template <typename ImageTraits>
__global__ void SmoothImageWithGaussian5x5Kernel(
    const typename ImageTraits::ScalarType* image_src,
    typename ImageTraits::ScalarType* image_dst, int cols, int rows,
    BorderType border_type, float sum_weight) {
  constexpr int kNumChannels = ImageTraits::kNumChannels;

  constexpr int kHalfKernelWidth = kGaussian5x5HalfKernelWidth;
  constexpr int kPaddedBlockWidth = kGaussian5x5BlockSizePadded;

  // Compute global image row and col coordinates for this thread id.
  const int image_col =
      blockIdx.x * kGaussian5x5BlockSize + threadIdx.x - kHalfKernelWidth;
  const int image_row =
      blockIdx.y * kGaussian5x5BlockSize + threadIdx.y - kHalfKernelWidth;

  __shared__ typename ImageTraits::ScalarType
      block_src[kPaddedBlockWidth * kPaddedBlockWidth];
  __shared__ float block_src_temp[kPaddedBlockWidth * kPaddedBlockWidth];

  for (int i = 0; i < kNumChannels; ++i) {
    typename ImageTraits::ScalarType src_pix =
        static_cast<ImageTraits::ScalarType>(0);
    int image_col_extrap =
        GetExtrapolatedPixelIndex(image_col, cols, border_type);
    int image_row_extrap =
        GetExtrapolatedPixelIndex(image_row, rows, border_type);
    if (image_col_extrap >= 0 && image_row_extrap >= 0) {
      src_pix = image_src[image_row_extrap * cols * kNumChannels +
                          image_col_extrap * kNumChannels + i];
    }
    block_src[threadIdx.y * kPaddedBlockWidth + threadIdx.x] = src_pix;
    __syncthreads();

    // Computes convolution in x direction. This convolution is performed by
    // all threads whose x coordinate falls into the un-padded region of size
    // kGaussian5x5BlockSize x kGaussian5x5BlockSize, but whose y coordinate
    // extends top and bottom into the padded region.
    if (threadIdx.x >= kHalfKernelWidth &&
        threadIdx.x < kHalfKernelWidth + kGaussian5x5BlockSize) {
      block_src_temp[threadIdx.y * kPaddedBlockWidth + threadIdx.x] =
          ConvolveGaussian5x5Row(block_src, threadIdx.x, threadIdx.y);
    }
    __syncthreads();

    // Computes convolution in y direction. This convolution is performed by
    // all threads whose x and y coordinates fall into the un-padded region of
    // size kGaussian5x5BlockSize x kGaussian5x5BlockSize.
    if (IsGaussian5x5InteriorThread()) {
      const float sum =
          ConvolveGaussian5x5Col(block_src_temp, threadIdx.x, threadIdx.y);

      if (image_col >= 0 && image_col < cols && image_row >= 0 &&
          image_row < rows) {
        image_dst[image_row * cols * kNumChannels + image_col * kNumChannels +
                  i] = static_cast<ImageTraits::ScalarType>(sum_weight * sum);
      }
    }
    __syncthreads();
  }
}

}  // namespace

template <typename ImageTraits>
absl::StatusOr<CudaImage<ImageTraits>> SmoothImageWithGaussian5x5(
    const CudaImage<ImageTraits>& image, BorderType border_type,
    float sum_weight) {
  INTR_ASSIGN_OR_RETURN(CudaImage<ImageTraits> image_smooth,
                        CudaImage<ImageTraits>::Create(image.dimensions()));
  const dim3 num_blocks(CeilSizeDiv(image.cols(), kGaussian5x5BlockSize),
                        CeilSizeDiv(image.rows(), kGaussian5x5BlockSize));
  const dim3 num_threads_per_block(kGaussian5x5BlockSizePadded,
                                   kGaussian5x5BlockSizePadded);
  SmoothImageWithGaussian5x5Kernel<ImageTraits>
      <<<num_blocks, num_threads_per_block>>>(image.data(), image_smooth.data(),
                                              image.cols(), image.rows(),
                                              border_type, sum_weight);
  CUDA_CHECK(cudaStreamSynchronize(nullptr));
  return image_smooth;
}

template absl::StatusOr<CudaImage<Gray8u>> SmoothImageWithGaussian5x5<Gray8u>(
    const CudaImage<Gray8u>& image, BorderType border_type, float sum_weight);

template absl::StatusOr<CudaImage<Rgb8u>> SmoothImageWithGaussian5x5<Rgb8u>(
    const CudaImage<Rgb8u>& image, BorderType border_type, float sum_weight);

template absl::StatusOr<CudaImage<Gray32f>> SmoothImageWithGaussian5x5<Gray32f>(
    const CudaImage<Gray32f>& image, BorderType border_type, float sum_weight);

template absl::StatusOr<CudaImage<Generic32f>>
SmoothImageWithGaussian5x5<Generic32f>(const CudaImage<Generic32f>& image,
                                       BorderType border_type,
                                       float sum_weight);

template absl::StatusOr<CudaImage<Generic32f3>>
SmoothImageWithGaussian5x5<Generic32f3>(const CudaImage<Generic32f3>& image,
                                        BorderType border_type,
                                        float sum_weight);

template absl::StatusOr<CudaImage<Generic16i>>
SmoothImageWithGaussian5x5<Generic16i>(const CudaImage<Generic16i>& image,
                                       BorderType border_type,
                                       float sum_weight);

template absl::StatusOr<CudaImage<Generic16i3>>
SmoothImageWithGaussian5x5<Generic16i3>(const CudaImage<Generic16i3>& image,
                                        BorderType border_type,
                                        float sum_weight);

}  // namespace perception
}  // namespace intrinsic
