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

#include "intrinsic/perception/gpu/cuda_image_proc/laplacian.h"

#include <cuda_runtime_api.h>
#include <device_types.h>
#include <vector_types.h>

#include "absl/status/statusor.h"
#include "intrinsic/perception/core/image_traits.h"
#include "intrinsic/perception/gpu/cuda_image_proc/border_extrapolation.h"
#include "intrinsic/perception/gpu/cuda_image_proc/border_extrapolation_device.h"
#include "intrinsic/perception/gpu/cuda_image_proc/laplacian_device.h"
#include "intrinsic/perception/gpu/cuda_utils/cuda_image.h"
#include "intrinsic/perception/gpu/cuda_utils/cuda_math.h"
#include "intrinsic/perception/gpu/cuda_utils/cuda_utils.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic {
namespace perception {
namespace {

template <typename ImageTraits>
__global__ void ApplyLaplacian3x3Kernel(
    const typename ImageTraits::ScalarType* image_src, float* image_dst,
    int cols, int rows, BorderType border_type) {
  constexpr int kNumChannels = ImageTraits::kNumChannels;
  // Compute global image row and col coordinates for this thread.
  const int image_col = blockIdx.x * kLaplacian3x3BlockSize + threadIdx.x -
                        kLaplacian3x3HalfKernelWidth;
  const int image_row = blockIdx.y * kLaplacian3x3BlockSize + threadIdx.y -
                        kLaplacian3x3HalfKernelWidth;
  const int image_col_extrap =
      GetExtrapolatedPixelIndex(image_col, cols, border_type);
  const int image_row_extrap =
      GetExtrapolatedPixelIndex(image_row, rows, border_type);

  __shared__ typename ImageTraits::ScalarType
      block_src[kLaplacian3x3BlockSizePadded * kLaplacian3x3BlockSizePadded];

  bool within_image_bounds = false;
  if (image_col >= 0 && image_col < cols && image_row >= 0 &&
      image_row < rows) {
    within_image_bounds = true;
    image_dst += image_row * cols * kNumChannels + image_col * kNumChannels;
  }

  for (int i = 0; i < kNumChannels; ++i) {
    typename ImageTraits::ScalarType src_pix =
        static_cast<typename ImageTraits::ScalarType>(0);
    if (image_col_extrap >= 0 && image_row_extrap >= 0) {
      src_pix = image_src[image_row_extrap * cols * kNumChannels +
                          image_col_extrap * kNumChannels + i];
    }

    block_src[threadIdx.y * kLaplacian3x3BlockSizePadded + threadIdx.x] =
        src_pix;
    __syncthreads();

    if (within_image_bounds && IsLaplacian3x3InteriorThread()) {
      image_dst[i] = ConvolveLaplacian3x3(block_src, threadIdx.x, threadIdx.y);
    }
    __syncthreads();
  }
}

}  // namespace

template <typename ImageTraits>
absl::StatusOr<CudaImage<GenericTrait<float, ImageTraits::kNumChannels>>>
ApplyLaplacian3x3(const CudaImage<ImageTraits>& image, BorderType border_type) {
  INTR_ASSIGN_OR_RETURN(
      (CudaImage<GenericTrait<float, ImageTraits::kNumChannels>>
           image_laplacian),
      (CudaImage<GenericTrait<float, ImageTraits::kNumChannels>>::Create(
          image.dimensions())));
  const dim3 num_blocks(CeilSizeDiv(image.cols(), kLaplacian3x3BlockSize),
                        CeilSizeDiv(image.rows(), kLaplacian3x3BlockSize));
  const dim3 num_threads_per_block(kLaplacian3x3BlockSizePadded,
                                   kLaplacian3x3BlockSizePadded);
  ApplyLaplacian3x3Kernel<ImageTraits><<<num_blocks, num_threads_per_block>>>(
      image.data(), image_laplacian.data(), image.cols(), image.rows(),
      border_type);
  CUDA_CHECK(cudaStreamSynchronize(nullptr));
  return image_laplacian;
}

template absl::StatusOr<CudaImage<Generic32f>> ApplyLaplacian3x3<Gray8u>(
    const CudaImage<Gray8u>& image, BorderType border_type);

template absl::StatusOr<CudaImage<Generic32f3>> ApplyLaplacian3x3<Rgb8u>(
    const CudaImage<Rgb8u>& image, BorderType border_type);

template absl::StatusOr<CudaImage<Generic32f>> ApplyLaplacian3x3<Gray32f>(
    const CudaImage<Gray32f>& image, BorderType border_type);

}  // namespace perception
}  // namespace intrinsic
