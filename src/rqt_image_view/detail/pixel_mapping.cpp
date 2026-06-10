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

#include <rqt_image_view/detail/pixel_mapping.hpp>

#include <algorithm>

namespace rqt_image_view
{
namespace detail
{

std::optional<QPoint> mapWidgetToImagePixel(
  int widget_x, int widget_y,
  int frame_w, int frame_h,
  int image_w, int image_h,
  int rotation_degrees)
{
  if (image_w <= 0 || image_h <= 0 || frame_w <= 0 || frame_h <= 0) {
    return std::nullopt;
  }
  if (widget_x < 0 || widget_y < 0 || widget_x >= frame_w || widget_y >= frame_h) {
    return std::nullopt;
  }

  // Displayed dimensions swap when the image is shown rotated by 90/270 degrees.
  const bool swapped = (rotation_degrees == ROTATION_90 || rotation_degrees == ROTATION_270);
  const int disp_w = swapped ? image_h : image_w;
  const int disp_h = swapped ? image_w : image_h;

  const double x_disp = static_cast<double>(widget_x) / frame_w * disp_w;
  const double y_disp = static_cast<double>(widget_y) / frame_h * disp_h;

  int x_orig = 0;
  int y_orig = 0;
  switch (rotation_degrees) {
    case ROTATION_0:
      x_orig = static_cast<int>(x_disp);
      y_orig = static_cast<int>(y_disp);
      break;
    case ROTATION_90:
      x_orig = static_cast<int>(y_disp);
      y_orig = static_cast<int>((disp_w - 1) - x_disp);
      break;
    case ROTATION_180:
      x_orig = static_cast<int>((disp_w - 1) - x_disp);
      y_orig = static_cast<int>((disp_h - 1) - y_disp);
      break;
    case ROTATION_270:
      x_orig = static_cast<int>((disp_h - 1) - y_disp);
      y_orig = static_cast<int>(x_disp);
      break;
    default:
      return std::nullopt;
  }
  x_orig = std::clamp(x_orig, 0, image_w - 1);
  y_orig = std::clamp(y_orig, 0, image_h - 1);
  return QPoint(x_orig, y_orig);
}

}  // namespace detail
}  // namespace rqt_image_view
