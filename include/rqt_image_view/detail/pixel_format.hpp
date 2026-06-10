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

#ifndef RQT_IMAGE_VIEW__DETAIL__PIXEL_FORMAT_HPP_
#define RQT_IMAGE_VIEW__DETAIL__PIXEL_FORMAT_HPP_

#include <sensor_msgs/msg/image.hpp>

#include <QString>  // NOLINT

namespace rqt_image_view
{
namespace detail
{

// Format the raw pixel value at (x, y) of `msg` for display in the info bar's
// hover label.
//
// Output contract (tested):
//   single-channel encodings (MONO8, MONO16, TYPE_8UC1, TYPE_16UC1, TYPE_32FC1)
//                                     → plain "value:<N>"
//   RGB8, BGR8, RGB16, BGR16          → HTML rich text "<span>R:<N></span>
//                                                       <span>G:<N></span>
//                                                       <span>B:<N></span>",
//                                       R/G/B always in display order
//   RGBA8, BGRA8, RGBA16, BGRA16      → same as above plus
//                                       "<span>A:<N></span>"
//   other recognised encodings        → plain "raw=[HH HH ...]" uppercase hex
//   unrecognised / underivable stride → plain "(unsupported)"
//   out-of-image or truncated row     → plain "(out of bounds)"
//
// Multi-byte channel reads assume host endianness; messages whose
// `is_bigendian` flag disagrees with the host route to the raw-hex fallback.
[[nodiscard]] QString formatPixelValue(const sensor_msgs::msg::Image & msg, int x, int y);

}  // namespace detail
}  // namespace rqt_image_view

#endif  // RQT_IMAGE_VIEW__DETAIL__PIXEL_FORMAT_HPP_
