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

#include <cmath>

#include <opencv2/core/core.hpp>

#include <rqt_image_view/linear_to_srgb.hpp>

namespace
{
const cv::Mat & lut()
{
  static const cv::Mat L = rqt_image_view::buildLinearToSrgbLut();
  return L;
}

}  // namespace

TEST(LinearToSrgb, MidGray18PercentMapsToAround118)
{
  // 18% linear reflectance is the classic "middle gray". After sRGB
  // encoding it should sit roughly in the upper-middle of the 8-bit range.
  const int linear_index = static_cast<int>(std::lround(0.18 * 255.0));
  const int encoded = lut().at<uchar>(linear_index);
  EXPECT_GE(encoded, 116);
  EXPECT_LE(encoded, 120);
}

TEST(LinearToSrgb, LiftsDarkValues)
{
  // The whole point of the feature: linear midtones should appear brighter
  // after encoding than they did before.
  for (int i = 1; i < 200; ++i) {
    EXPECT_GT(lut().at<uchar>(i), i)
      << "Encoded value at index " << i << " should be greater than input";
  }
}

TEST(LinearToSrgb, DoesNotMutateAliasedBuffer)
{
  // Regression test: cv_bridge::toCvShare hands out a cv::Mat whose data
  // is shared with the underlying ROS message memory. If linearToSrgb were
  // to write into that shared buffer, it would mutate foreign memory,
  // which is undefined behavior and can crash. Verify that a Mat sharing
  // the input buffer is left untouched.
  cv::Mat original(1, 4, CV_8UC3);
  original.at<cv::Vec3b>(0, 0) = cv::Vec3b(10, 20, 30);
  original.at<cv::Vec3b>(0, 1) = cv::Vec3b(46, 46, 46);
  original.at<cv::Vec3b>(0, 2) = cv::Vec3b(100, 150, 200);
  original.at<cv::Vec3b>(0, 3) = cv::Vec3b(255, 255, 255);

  const cv::Mat aliased = original;   // shallow copy: shares the same data buffer
  ASSERT_EQ(aliased.data, original.data);

  const cv::Mat encoded = rqt_image_view::linearToSrgb(aliased);

  EXPECT_EQ(original.at<cv::Vec3b>(0, 0), cv::Vec3b(10, 20, 30));
  EXPECT_EQ(original.at<cv::Vec3b>(0, 1), cv::Vec3b(46, 46, 46));
  EXPECT_EQ(original.at<cv::Vec3b>(0, 2), cv::Vec3b(100, 150, 200));
  EXPECT_EQ(original.at<cv::Vec3b>(0, 3), cv::Vec3b(255, 255, 255));

  // And the returned image should hold encoded values.
  EXPECT_GT(encoded.at<cv::Vec3b>(0, 1)[0], 46);
}

TEST(LinearToSrgb, EncodesEveryChannelOfRgbImage)
{
  cv::Mat img(2, 2, CV_8UC3);
  img.at<cv::Vec3b>(0, 0) = cv::Vec3b(0, 0, 0);
  img.at<cv::Vec3b>(0, 1) = cv::Vec3b(255, 255, 255);
  img.at<cv::Vec3b>(1, 0) = cv::Vec3b(46, 46, 46);    // ~18% linear
  img.at<cv::Vec3b>(1, 1) = cv::Vec3b(46, 128, 200);  // mixed channels

  const cv::Mat encoded = rqt_image_view::linearToSrgb(img);

  EXPECT_EQ(encoded.at<cv::Vec3b>(0, 0), cv::Vec3b(0, 0, 0));
  EXPECT_EQ(encoded.at<cv::Vec3b>(0, 1), cv::Vec3b(255, 255, 255));

  const cv::Vec3b mid = encoded.at<cv::Vec3b>(1, 0);
  for (int c = 0; c < 3; ++c) {
    EXPECT_GE(mid[c], 116);
    EXPECT_LE(mid[c], 120);
  }

  // Mixed-channel pixel: each channel encoded independently and
  // strictly brighter than the input (we're well above the toe).
  const cv::Vec3b mixed = encoded.at<cv::Vec3b>(1, 1);
  EXPECT_GT(mixed[0], 46);
  EXPECT_GT(mixed[1], 128);
  EXPECT_GT(mixed[2], 200);
}
