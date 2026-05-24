/*
 * Copyright (c) 2026, Arne Baeyens
 * Copyright (c) 2011, Dirk Thomas, TU Darmstadt
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
 *   * Neither the name of the TU Darmstadt nor the names of its
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

#ifndef RQT_IMAGE_VIEW__DETAIL__PIXEL_MAPPING_HPP_
#define RQT_IMAGE_VIEW__DETAIL__PIXEL_MAPPING_HPP_

#include <optional>

#include <QPoint>  // NOLINT

namespace rqt_image_view
{
namespace detail
{

// Recognised rotation values. Anything else passed as `rotation_degrees`
// makes mapWidgetToImagePixel return std::nullopt.
constexpr int ROTATION_0 = 0;
constexpr int ROTATION_90 = 90;
constexpr int ROTATION_180 = 180;
constexpr int ROTATION_270 = 270;

// Map a cursor position in the on-screen image widget back to the original
// (unrotated) image pixel.
//
// @param widget_x,widget_y  Cursor coordinates inside the image widget.
// @param frame_w,frame_h    Image widget dimensions in pixels.
// @param image_w,image_h    Original image dimensions in pixels (pre-rotation).
// @param rotation_degrees   Active display rotation. Must be one of
//                           ROTATION_{0,90,180,270}; any other value returns
//                           std::nullopt.
// @returns The clamped (x, y) pixel in the original image, or std::nullopt
//          when any input is degenerate (zero dimensions, cursor outside the
//          widget, unrecognised rotation).
[[nodiscard]] std::optional<QPoint> mapWidgetToImagePixel(
  int widget_x, int widget_y,
  int frame_w, int frame_h,
  int image_w, int image_h,
  int rotation_degrees);

}  // namespace detail
}  // namespace rqt_image_view

#endif  // RQT_IMAGE_VIEW__DETAIL__PIXEL_MAPPING_HPP_
