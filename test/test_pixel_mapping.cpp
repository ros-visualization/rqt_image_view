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

#include <gtest/gtest.h>

#include <rqt_image_view/detail/pixel_mapping.hpp>

using rqt_image_view::detail::mapWidgetToImagePixel;

TEST(MapWidgetToImagePixel, ZeroDimensionReturnsNullopt)
{
  EXPECT_FALSE(mapWidgetToImagePixel(0, 0, 100, 100, 0, 100, 0).has_value());
  EXPECT_FALSE(mapWidgetToImagePixel(0, 0, 100, 100, 100, 0, 0).has_value());
  EXPECT_FALSE(mapWidgetToImagePixel(0, 0, 0, 100, 100, 100, 0).has_value());
  EXPECT_FALSE(mapWidgetToImagePixel(0, 0, 100, 0, 100, 100, 0).has_value());
}

TEST(MapWidgetToImagePixel, CursorOutsideFrameReturnsNullopt)
{
  EXPECT_FALSE(mapWidgetToImagePixel(-1, 0, 100, 100, 100, 100, 0).has_value());
  EXPECT_FALSE(mapWidgetToImagePixel(100, 0, 100, 100, 100, 100, 0).has_value());
  EXPECT_FALSE(mapWidgetToImagePixel(0, -1, 100, 100, 100, 100, 0).has_value());
  EXPECT_FALSE(mapWidgetToImagePixel(0, 100, 100, 100, 100, 100, 0).has_value());
}

TEST(MapWidgetToImagePixel, InvalidRotationReturnsNullopt)
{
  EXPECT_FALSE(mapWidgetToImagePixel(50, 50, 100, 100, 100, 100, 45).has_value());
  EXPECT_FALSE(mapWidgetToImagePixel(50, 50, 100, 100, 100, 100, -90).has_value());
  EXPECT_FALSE(mapWidgetToImagePixel(50, 50, 100, 100, 100, 100, 360).has_value());
}

TEST(MapWidgetToImagePixel, Rotation0WithMatchingFrameMapsIdentity)
{
  const auto p = mapWidgetToImagePixel(13, 27, 100, 100, 100, 100, 0);
  ASSERT_TRUE(p.has_value());
  EXPECT_EQ(p->x(), 13);
  EXPECT_EQ(p->y(), 27);
}

TEST(MapWidgetToImagePixel, Rotation180InvertsBothAxes)
{
  const auto p = mapWidgetToImagePixel(10, 20, 100, 100, 100, 100, 180);
  ASSERT_TRUE(p.has_value());
  EXPECT_EQ(p->x(), 89);
  EXPECT_EQ(p->y(), 79);
}

TEST(MapWidgetToImagePixel, Rotation90SwapsAndFlipsX)
{
  // Image is 100×80 ⇒ displayed frame 80×100 after rotating 90° CW.
  // Widget (40, 50) inside the 80×100 frame should map to original (50, 39).
  const auto p = mapWidgetToImagePixel(40, 50, 80, 100, 100, 80, 90);
  ASSERT_TRUE(p.has_value());
  EXPECT_EQ(p->x(), 50);
  EXPECT_EQ(p->y(), 39);
}

TEST(MapWidgetToImagePixel, Rotation270SwapsAndFlipsY)
{
  // Same image as 90° test, rotated 90° CCW instead. Widget (40, 50) in a
  // 80×100 frame ⇒ original (49, 40).
  const auto p = mapWidgetToImagePixel(40, 50, 80, 100, 100, 80, 270);
  ASSERT_TRUE(p.has_value());
  EXPECT_EQ(p->x(), 49);
  EXPECT_EQ(p->y(), 40);
}

TEST(MapWidgetToImagePixel, ResultIsClampedToImageBounds)
{
  // Every rotation, every corner of the frame ⇒ pixel inside [0, w-1] × [0, h-1].
  for (int rot : {0, 90, 180, 270}) {
    for (int wx : {0, 99}) {
      for (int wy : {0, 99}) {
        const auto p = mapWidgetToImagePixel(wx, wy, 100, 100, 100, 100, rot);
        ASSERT_TRUE(p.has_value()) << "rot=" << rot << " (" << wx << ", " << wy << ")";
        EXPECT_GE(p->x(), 0);
        EXPECT_LE(p->x(), 99);
        EXPECT_GE(p->y(), 0);
        EXPECT_LE(p->y(), 99);
      }
    }
  }
}

TEST(MapWidgetToImagePixel, FrameDownscaledFromImageStillMaps)
{
  // Image 800×600 displayed at 80×60 (downscaled 10×); cursor at frame (8, 6)
  // lands at exactly (80, 60) by direct proportion (no rounding involved).
  const auto p = mapWidgetToImagePixel(8, 6, 80, 60, 800, 600, 0);
  ASSERT_TRUE(p.has_value());
  EXPECT_EQ(p->x(), 80);
  EXPECT_EQ(p->y(), 60);
}
