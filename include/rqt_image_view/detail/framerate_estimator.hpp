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

#ifndef RQT_IMAGE_VIEW__DETAIL__FRAMERATE_ESTIMATOR_HPP_
#define RQT_IMAGE_VIEW__DETAIL__FRAMERATE_ESTIMATOR_HPP_

#include <chrono>
#include <cstddef>
#include <deque>
#include <optional>

namespace rqt_image_view
{
namespace detail
{

// Sliding-window framerate estimator over recent image arrivals.
//
// Uses caller-supplied steady_clock arrival times rather than header.stamp so
// the readout reflects what the consumer is actually seeing (bag replay rate,
// dropped frames, buffering all included).
//
// Not thread-safe; caller synchronizes between the producer (executor) and
// consumer (GUI) threads.
class FramerateEstimator
{
public:
  using SteadyClock = std::chrono::steady_clock;
  using TimePoint = SteadyClock::time_point;

  static constexpr std::size_t MAX_SAMPLES = 20;
  static constexpr double STALENESS_FLOOR_SEC = 1.0;
  static constexpr double STALENESS_PERIOD_MULTIPLIER = 2.5;

  // Record a single message arrival at wall_now.
  void addSample(TimePoint wall_now);

  // Rate in Hz over the window, or nullopt for too few samples / stalled.
  [[nodiscard]] std::optional<double> compute(TimePoint wall_now) const noexcept;

  void reset() noexcept;

  [[nodiscard]] std::size_t sampleCount() const noexcept {return arrivals_.size();}

private:
  std::deque<TimePoint> arrivals_;
};

}  // namespace detail
}  // namespace rqt_image_view

#endif  // RQT_IMAGE_VIEW__DETAIL__FRAMERATE_ESTIMATOR_HPP_
