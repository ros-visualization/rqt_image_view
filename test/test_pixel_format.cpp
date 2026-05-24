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

#include <cstdint>
#include <cstring>
#include <string>

#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <rqt_image_view/detail/pixel_format.hpp>

namespace
{

namespace enc = sensor_msgs::image_encodings;

sensor_msgs::msg::Image makeImage(
  std::uint32_t w, std::uint32_t h, const std::string & encoding, std::uint32_t step)
{
  sensor_msgs::msg::Image msg;
  msg.width = w;
  msg.height = h;
  msg.encoding = encoding;
  msg.step = step;
  msg.is_bigendian = 0;
  msg.data.assign(static_cast<std::size_t>(step) * h, 0);
  return msg;
}

}  // namespace

TEST(FormatPixelValue, OutOfBoundsCoordinatesReportSentinel)
{
  auto msg = makeImage(4, 4, enc::MONO8, 4);
  EXPECT_EQ(rqt_image_view::detail::formatPixelValue(msg, -1, 0), QString("(out of bounds)"));
  EXPECT_EQ(rqt_image_view::detail::formatPixelValue(msg, 0, -1), QString("(out of bounds)"));
  EXPECT_EQ(rqt_image_view::detail::formatPixelValue(msg, 4, 0), QString("(out of bounds)"));
  EXPECT_EQ(rqt_image_view::detail::formatPixelValue(msg, 0, 4), QString("(out of bounds)"));
}

TEST(FormatPixelValue, TruncatedDataReportsSentinelInsteadOfReadingPastEnd)
{
  // Step is correct, but the data vector is shorter than declared height.
  auto msg = makeImage(4, 4, enc::MONO8, 4);
  msg.data.resize(2);
  EXPECT_EQ(rqt_image_view::detail::formatPixelValue(msg, 0, 1), QString("(out of bounds)"));
}

TEST(FormatPixelValue, MalformedStepNeverReadsBeyondRow)
{
  // RGB8 requires 3 bytes/pixel, so width=4 implies step ≥ 12. A publisher
  // that reports step=6 (truncating the row to 2 valid pixels) must not lead
  // to an out-of-row read at x=3.
  auto msg = makeImage(4, 1, enc::RGB8, 6);
  msg.data.assign(6, 0xAB);
  // Within the valid prefix this is decoded normally.
  EXPECT_NE(rqt_image_view::detail::formatPixelValue(msg, 0, 0), QString("(out of bounds)"));
  // Past the prefix we must get a bounds sentinel, not heap-OOB bytes.
  EXPECT_EQ(rqt_image_view::detail::formatPixelValue(msg, 3, 0), QString("(out of bounds)"));
}

TEST(FormatPixelValue, Mono8RendersValueLabel)
{
  auto msg = makeImage(2, 1, enc::MONO8, 2);
  msg.data = {42, 200};
  const auto s0 = rqt_image_view::detail::formatPixelValue(msg, 0, 0);
  const auto s1 = rqt_image_view::detail::formatPixelValue(msg, 1, 0);
  EXPECT_TRUE(s0.contains("value:")) << s0.toStdString();
  EXPECT_TRUE(s0.contains("42")) << s0.toStdString();
  EXPECT_TRUE(s1.contains("200")) << s1.toStdString();
}

TEST(FormatPixelValue, Mono16RendersDecimalValue)
{
  auto msg = makeImage(1, 1, enc::MONO16, 2);
  msg.data = {0x34, 0x12};  // 0x1234 little-endian
  EXPECT_TRUE(rqt_image_view::detail::formatPixelValue(msg, 0, 0).contains("4660"));
}

TEST(FormatPixelValue, Float32RendersValueLabel)
{
  auto msg = makeImage(1, 1, enc::TYPE_32FC1, 4);
  const float v = 3.14159f;
  std::memcpy(msg.data.data(), &v, sizeof(v));
  const auto s = rqt_image_view::detail::formatPixelValue(msg, 0, 0);
  EXPECT_TRUE(s.contains("3.14")) << s.toStdString();
}

TEST(FormatPixelValue, Rgb8ShowsAllThreeChannels)
{
  auto msg = makeImage(1, 1, enc::RGB8, 3);
  msg.data = {10, 20, 30};
  const auto s = rqt_image_view::detail::formatPixelValue(msg, 0, 0);
  EXPECT_TRUE(s.contains("R:")) << s.toStdString();
  EXPECT_TRUE(s.contains("G:")) << s.toStdString();
  EXPECT_TRUE(s.contains("B:")) << s.toStdString();
  EXPECT_TRUE(s.contains("10"));
  EXPECT_TRUE(s.contains("20"));
  EXPECT_TRUE(s.contains("30"));
}

TEST(FormatPixelValue, Bgr8DisplaysRGBOrderRegardlessOfMemoryLayout)
{
  // Memory layout for BGR8 is B@0, G@1, R@2; the display order is still RGB.
  // So the R: label must show the value at offset 2, not offset 0.
  auto msg = makeImage(1, 1, enc::BGR8, 3);
  msg.data = {/*B=*/10, /*G=*/20, /*R=*/30};
  const auto s = rqt_image_view::detail::formatPixelValue(msg, 0, 0);
  const auto r_pos = s.indexOf("R:");
  const auto g_pos = s.indexOf("G:");
  const auto b_pos = s.indexOf("B:");
  ASSERT_NE(r_pos, -1);
  ASSERT_NE(g_pos, -1);
  ASSERT_NE(b_pos, -1);
  EXPECT_LT(r_pos, g_pos);
  EXPECT_LT(g_pos, b_pos);
  EXPECT_TRUE(s.mid(r_pos, g_pos - r_pos).contains("30"));
  EXPECT_TRUE(s.mid(b_pos).contains("10"));
}

TEST(FormatPixelValue, Rgba8IncludesAlphaChannel)
{
  auto msg = makeImage(1, 1, enc::RGBA8, 4);
  msg.data = {1, 2, 3, 255};
  const auto s = rqt_image_view::detail::formatPixelValue(msg, 0, 0);
  EXPECT_TRUE(s.contains("A:")) << s.toStdString();
  EXPECT_TRUE(s.contains("255"));
}

TEST(FormatPixelValue, Bgra16DisplaysFourChannels)
{
  auto msg = makeImage(1, 1, enc::BGRA16, 8);
  // 10, 20, 30, 40 little-endian
  msg.data = {0x0A, 0x00, 0x14, 0x00, 0x1E, 0x00, 0x28, 0x00};
  const auto s = rqt_image_view::detail::formatPixelValue(msg, 0, 0);
  EXPECT_TRUE(s.contains("R:")) << s.toStdString();
  EXPECT_TRUE(s.contains("A:")) << s.toStdString();
  // BGRA in memory ⇒ B=10, G=20, R=30, A=40
  EXPECT_TRUE(s.contains("10"));
  EXPECT_TRUE(s.contains("20"));
  EXPECT_TRUE(s.contains("30"));
  EXPECT_TRUE(s.contains("40"));
}

TEST(FormatPixelValue, UnknownEncodingWithDerivableStrideDumpsRawHex)
{
  // "fancy_format" is not recognised by sensor_msgs::image_encodings, but
  // msg.step (3) lets us derive a 3-byte stride and dump those bytes.
  auto msg = makeImage(1, 1, "fancy_format", 3);
  msg.data = {0xDE, 0xAD, 0xBE};
  const auto s = rqt_image_view::detail::formatPixelValue(msg, 0, 0);
  EXPECT_TRUE(s.startsWith("raw=")) << s.toStdString();
  EXPECT_TRUE(s.toUpper().contains("DE"));
  EXPECT_TRUE(s.toUpper().contains("AD"));
  EXPECT_TRUE(s.toUpper().contains("BE"));
}

TEST(FormatPixelValue, ZeroStepReturnsOutOfBounds)
{
  // A step of zero cannot satisfy the row-bound invariant; the function must
  // refuse to read rather than silently use a fallback that could overrun.
  auto msg = makeImage(4, 4, "fancy_format", 0);
  msg.data.assign(16, 0);
  EXPECT_EQ(rqt_image_view::detail::formatPixelValue(msg, 0, 0), QString("(out of bounds)"));
}
