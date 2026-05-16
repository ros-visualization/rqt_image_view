/*
 * Copyright (c) 2026, Arne Baeyens
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef RQT_IMAGE_VIEW__LINEAR_TO_SRGB_HPP_
#define RQT_IMAGE_VIEW__LINEAR_TO_SRGB_HPP_

#include <algorithm>
#include <cmath>

#include <opencv2/core/core.hpp>

namespace rqt_image_view
{
/**
 * Create a LUT with the sRGB OETF (linear -> sRGB encoding)
 */
inline cv::Mat buildLinearToSrgbLut()
{
  cv::Mat lut(1, 256, CV_8UC1);
  uchar * data = lut.ptr<uchar>();
  for (int i = 0; i < 256; ++i) {
    const double linear = static_cast<double>(i) / 255.0;
    const double encoded = linear <= 0.0031308 ?
      12.92 * linear :
      1.055 * std::pow(linear, 1.0 / 2.4) - 0.055;
    const int v = std::clamp(static_cast<int>(std::lround(encoded * 255.0)), 0, 255);
    data[i] = static_cast<uchar>(v);
  }
  return lut;
}

/**
 * Apply the sRGB OETF (linear -> sRGB encoding)
 * and return a freshly allocated result. The input is assumed to be 8-bit
 * per channel, with values representing linear light (0..255 mapped to
 * linear 0..1). The returned image holds sRGB-encoded values suitable for
 * direct display on an sRGB monitor.
 *
 * A new output buffer is always allocated rather than writing in place,
 * because the caller may pass a Mat whose data is shared with read-only or
 * aliased memory (e.g. cv_bridge::toCvShare backed by a ROS message buffer).
 */
inline cv::Mat linearToSrgb(const cv::Mat & image)
{
  static const cv::Mat lut = buildLinearToSrgbLut();
  cv::Mat encoded;
  cv::LUT(image, lut, encoded);
  return encoded;
}

}  // namespace rqt_image_view

#endif  // RQT_IMAGE_VIEW__LINEAR_TO_SRGB_HPP_
