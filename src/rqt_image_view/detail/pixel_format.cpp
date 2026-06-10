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

#include <rqt_image_view/detail/pixel_format.hpp>

#if defined(__BYTE_ORDER__) && __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__
#error "rqt_image_view assumes a little-endian host"
#endif

#include <cstdint>
#include <cstring>
#include <string>

#include <sensor_msgs/image_encodings.hpp>

#include <QChar>  // NOLINT
#include <QLatin1Char>  // NOLINT
#include <QString>  // NOLINT

namespace rqt_image_view
{
namespace detail
{
namespace
{

// Pad widths matching the maximum decimal value of each channel encoding.
constexpr int PAD_8BIT = 3;   // 0..255
constexpr int PAD_16BIT = 5;  // 0..65535

// NBSP as fill so HTML rendering doesn't collapse the alignment padding.
constexpr QChar NBSP(0x00A0);

[[nodiscard]] std::uint16_t readU16(const std::uint8_t * p) noexcept
{
  std::uint16_t v;
  std::memcpy(&v, p, sizeof(v));
  return v;
}

[[nodiscard]] float readF32(const std::uint8_t * p) noexcept
{
  float v;
  std::memcpy(&v, p, sizeof(v));
  return v;
}

// Channel-coloured RGB/A spans. Palette follows ros2/rviz PR #1716
// (https://github.com/ros2/rviz/pull/1716); contrast on dark themes is a
// known limitation.
QString formatRgbHtml(unsigned r, unsigned g, unsigned b, int pad)
{
  return QStringLiteral(
    "<span style='color:#c00'>R:%1</span> "
    "<span style='color:#0a0'>G:%2</span> "
    "<span style='color:#06c'>B:%3</span>")
         .arg(r, pad, 10, NBSP)
         .arg(g, pad, 10, NBSP)
         .arg(b, pad, 10, NBSP);
}

QString formatRgbaHtml(unsigned r, unsigned g, unsigned b, unsigned a, int pad)
{
  return formatRgbHtml(r, g, b, pad) +
         QStringLiteral(" <span style='color:#aaa'>A:%1</span>").arg(a, pad, 10, NBSP);
}

// Bytes per pixel for an encoding whose layout is "<channels> × <bits>/8".
// Returns 0 for encodings whose layout is not that simple (Bayer, planar YUV,
// etc.) — those fall through to the raw-hex stride-guess path.
std::size_t bytesPerPixel(const std::string & encoding)
{
  namespace enc = sensor_msgs::image_encodings;
  try {
    const int channels = enc::numChannels(encoding);
    const int bit_depth = enc::bitDepth(encoding);
    if (channels <= 0 || bit_depth <= 0 || (bit_depth % 8) != 0) {
      return 0;
    }
    return channels * (bit_depth / 8);
  } catch (const std::runtime_error &) {
    return 0;
  }
}

QString formatRawHex(const std::uint8_t * p, std::size_t n)
{
  QString out = QStringLiteral("raw=[");
  for (std::size_t i = 0; i < n; ++i) {
    if (i != 0) {
      out += QLatin1Char(' ');
    }
    out += QStringLiteral("%1").arg(p[i], 2, 16, QLatin1Char('0')).toUpper();
  }
  out += QLatin1Char(']');
  return out;
}

}  // namespace

QString formatPixelValue(const sensor_msgs::msg::Image & msg, int x, int y)
{
  // The bounds arithmetic below multiplies two uint32 fields (step, height)
  // and needs size_t to be wide enough to hold the product without wrapping.
  // Bail out on 32-bit builds rather than allow OOB reads from a malicious bag.
  if constexpr (sizeof(std::size_t) < 8) {
    return QStringLiteral("(not supported on 32-bit)");
  }

  namespace enc = sensor_msgs::image_encodings;

  if (x < 0 || y < 0 || msg.width == 0 || msg.height == 0) {
    return QStringLiteral("(out of bounds)");
  }
  const auto ux = static_cast<std::uint32_t>(x);
  const auto uy = static_cast<std::uint32_t>(y);
  if (ux >= msg.width || uy >= msg.height) {
    return QStringLiteral("(out of bounds)");
  }

  const std::size_t data_size = msg.data.size();
  const std::size_t step = msg.step;
  if (step == 0 || step > data_size || step * msg.height > data_size) {
    return QStringLiteral("(out of bounds)");
  }
  const std::size_t row_offset = uy * step;

  const std::size_t bpp = bytesPerPixel(msg.encoding);
  if (bpp > 0) {
    const std::size_t pixel_offset = ux * bpp;
    if (row_offset + pixel_offset + bpp > data_size) {
      return QStringLiteral("(out of bounds)");
    }
    const std::uint8_t * row = msg.data.data() + row_offset;
    const std::uint8_t * pixel = row + pixel_offset;

    // Host is little-endian (enforced at top of file); a big-endian message
    // on multi-byte channels routes to the raw-hex fallback below.
    const bool endian_ok = (bpp == 1) || msg.is_bigendian == 0;
    if (endian_ok) {
      const std::string & e = msg.encoding;
      if (e == enc::MONO8 || e == enc::TYPE_8UC1) {
        return QStringLiteral("value:%1").arg(pixel[0], PAD_8BIT, 10, NBSP);
      } else if (e == enc::MONO16 || e == enc::TYPE_16UC1) {
        return QStringLiteral("value:%1").arg(readU16(pixel), PAD_16BIT, 10, NBSP);
      } else if (e == enc::TYPE_32FC1) {
        return QStringLiteral("value:%1")
               .arg(static_cast<double>(readF32(pixel)), 0, 'g', 6);
      } else if (e == enc::RGB8) {
        return formatRgbHtml(pixel[0], pixel[1], pixel[2], PAD_8BIT);
      } else if (e == enc::BGR8) {
        return formatRgbHtml(pixel[2], pixel[1], pixel[0], PAD_8BIT);
      } else if (e == enc::RGBA8) {
        return formatRgbaHtml(pixel[0], pixel[1], pixel[2], pixel[3], PAD_8BIT);
      } else if (e == enc::BGRA8) {
        return formatRgbaHtml(pixel[2], pixel[1], pixel[0], pixel[3], PAD_8BIT);
      } else if (e == enc::RGB16) {
        return formatRgbHtml(
          readU16(pixel), readU16(pixel + 2), readU16(pixel + 4), PAD_16BIT);
      } else if (e == enc::BGR16) {
        return formatRgbHtml(
          readU16(pixel + 4), readU16(pixel + 2), readU16(pixel), PAD_16BIT);
      } else if (e == enc::RGBA16) {
        return formatRgbaHtml(
          readU16(pixel), readU16(pixel + 2), readU16(pixel + 4),
          readU16(pixel + 6), PAD_16BIT);
      } else if (e == enc::BGRA16) {
        return formatRgbaHtml(
          readU16(pixel + 4), readU16(pixel + 2), readU16(pixel),
          readU16(pixel + 6), PAD_16BIT);
      }
    }
    // Recognised-stride encoding but unhandled name (e.g. TYPE_8UC2) or
    // endian-mismatched multi-byte data: dump raw bytes.
    return formatRawHex(pixel, bpp);
  }

  // Encoding with no recognisable per-pixel stride: derive from msg.step.
  // Cap the dump so a malicious publisher can't coerce a megabyte QString.
  constexpr std::size_t MAX_RAW_BYTES_PER_PIXEL = 16;
  const std::size_t guessed_bpp = step / msg.width;
  if (guessed_bpp == 0 || guessed_bpp > MAX_RAW_BYTES_PER_PIXEL) {
    return QStringLiteral("(unsupported)");
  }
  const std::size_t pixel_offset = ux * guessed_bpp;
  if (row_offset + pixel_offset + guessed_bpp > data_size) {
    return QStringLiteral("(out of bounds)");
  }
  return formatRawHex(msg.data.data() + row_offset + pixel_offset, guessed_bpp);
}

}  // namespace detail
}  // namespace rqt_image_view
