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

#include "intrinsic/perception/gpu/cuda_image_proc/color_conversion.h"

#include "absl/status/statusor.h"
#include "intrinsic/perception/gpu/cuda_image_proc/color_conversion_device.h"
#include "intrinsic/perception/gpu/cuda_utils/cuda_image.h"
#include "intrinsic/perception/gpu/cuda_utils/cuda_math.h"
#include "intrinsic/perception/gpu/cuda_utils/cuda_utils.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic {
namespace perception {
namespace {

template <typename OutputImageTraits, typename InputImageTraits>
__global__ void RgbToGrayKernel(
    const typename InputImageTraits::ScalarType* image_rgb,
    typename OutputImageTraits::ScalarType* image_gray, int cols, int rows) {
  const unsigned int x = blockIdx.x * blockDim.x + threadIdx.x;
  const unsigned int y = blockIdx.y * blockDim.y + threadIdx.y;
  if (x >= cols || y >= rows) {
    return;
  }
  image_rgb += y * cols * 3 + x * 3;
  image_gray += y * cols + x;
  image_gray[0] = RgbPixelToGray<OutputImageTraits>(image_rgb[0], image_rgb[1],
                                                    image_rgb[2]);
}

}  // namespace

template <typename OutputImageTraits, typename InputImageTraits>
absl::StatusOr<CudaImage<OutputImageTraits>> RgbToGray(
    const CudaImage<InputImageTraits>& image_rgb) {
  INTR_ASSIGN_OR_RETURN(
      CudaImage<OutputImageTraits> image_gray,
      CudaImage<OutputImageTraits>::Create(image_rgb.dimensions()));
  constexpr int kBlockSize = 16;
  const dim3 num_blocks(CeilSizeDiv(image_rgb.cols(), kBlockSize),
                        CeilSizeDiv(image_rgb.rows(), kBlockSize));
  const dim3 num_threads_per_block(kBlockSize, kBlockSize);
  RgbToGrayKernel<OutputImageTraits, InputImageTraits>
      <<<num_blocks, num_threads_per_block>>>(
          image_rgb.data(), image_gray.data(), image_rgb.cols(),
          image_rgb.rows());
  CUDA_CHECK(cudaStreamSynchronize(nullptr));
  return image_gray;
}

template absl::StatusOr<CudaImage<Gray8u>> RgbToGray(
    const CudaImage<Rgb8u>& image_rgb);

template absl::StatusOr<CudaImage<Gray32f>> RgbToGray(
    const CudaImage<Rgb32f>& image_rgb);

}  // namespace perception
}  // namespace intrinsic
