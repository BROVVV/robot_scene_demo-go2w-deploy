// Copyright 2026 robot_scene_demo maintainers
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
//
// You may obtain a copy of the License at
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Timestamp auto-policy for PandarXT-16 point clouds (plan §5.2).
//
// The Hesai driver normally provides per-point FLOAT64 timestamps.  They may
// be absolute seconds, relative scan time, zeroed, or expressed in ns/us/ms.
// We never guess silently: every frame is classified into one explicit mode
// and non-monotonic input is flagged for the health monitor.
//
// All thresholds are configurable; nothing is hard-coded in the nodes.

#ifndef GO2W_PLAIN_SLAM_BRIDGE__TIMESTAMP_POLICY_HPP_
#define GO2W_PLAIN_SLAM_BRIDGE__TIMESTAMP_POLICY_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

namespace go2w_plain_slam_bridge
{

enum class TimestampMode
{
  kAbsoluteSeconds,
  kRelativeScan,
  kSyntheticLinear,
  kConvertedUnits,
  kError,
};

inline std::string timestamp_mode_name(TimestampMode mode)
{
  switch (mode) {
    case TimestampMode::kAbsoluteSeconds:
      return "ABSOLUTE_SECONDS";
    case TimestampMode::kRelativeScan:
      return "RELATIVE_SCAN";
    case TimestampMode::kSyntheticLinear:
      return "TIMESTAMP_SYNTHETIC";
    case TimestampMode::kConvertedUnits:
      return "CONVERTED_UNITS";
    case TimestampMode::kError:
      return "TIMESTAMP_ERROR";
  }
  return "UNKNOWN";
}

struct TimestampPolicyResult
{
  TimestampMode mode = TimestampMode::kError;
  // Resolved per-point timestamps in seconds, expressed relative to the same
  // epoch as the header stamp (i.e. absolute seconds on the host clock).
  std::vector<double> timestamps;
  bool non_monotonic = false;
};

inline bool is_finite(double v)
{
  return std::isfinite(v);
}

// Resolve per-point timestamps given the message header stamp (seconds).
//
//   raw               : per-point timestamp field values (empty -> synthetic)
//   header_stamp_sec  : msg.header.stamp in seconds
//   scan_period_s     : nominal scan period (synthetic fallback spacing)
//   absolute_tolerance_s : allowed |timestamp - header| for absolute mode
TimestampPolicyResult resolve_timestamps(
  const std::vector<double> & raw,
  double header_stamp_sec,
  double scan_period_s,
  double absolute_tolerance_s)
{
  TimestampPolicyResult result;
  const std::size_t n = raw.size();
  if (n == 0) {
    result.mode = TimestampMode::kSyntheticLinear;
    result.timestamps.clear();
    return result;
  }

  // 1. NaN/Inf is never silently swallowed.
  for (const double v : raw) {
    if (!is_finite(v)) {
      result.mode = TimestampMode::kError;
      return result;
    }
  }

  // 2. All-zero -> synthetic linear fallback.
  bool all_zero = true;
  for (const double v : raw) {
    if (std::abs(v) > 1e-12) {
      all_zero = false;
      break;
    }
  }
  if (all_zero) {
    result.mode = TimestampMode::kSyntheticLinear;
    if (n > 1) {
      result.timestamps.reserve(n);
      for (std::size_t i = 0; i < n; ++i) {
        result.timestamps.push_back(
          header_stamp_sec + scan_period_s * static_cast<double>(i) /
          static_cast<double>(n - 1));
      }
      result.non_monotonic = false;
    } else {
      result.timestamps.push_back(header_stamp_sec);
    }
    return result;
  }

  // Median as a robust location estimate.
  const std::size_t mid = n / 2;
  std::vector<double> sorted(raw.begin(), raw.end());
  std::sort(sorted.begin(), sorted.end());
  const double median = (n % 2 == 1) ? sorted[mid] : 0.5 * (sorted[mid - 1] + sorted[mid]);
  const double span = sorted.back() - sorted.front();

  // 3. Absolute seconds (median close to header stamp).
  if (std::abs(median - header_stamp_sec) < absolute_tolerance_s) {
    result.mode = TimestampMode::kAbsoluteSeconds;
    result.timestamps = raw;
  } else {
    // 4. Explicit unit conversion (ns / us / ms) before relative fallback:
    //    only applied when the scaled median aligns with the header stamp.
    bool converted = false;
    const double scales[] = {1e-9, 1e-6, 1e-3};
    const char * unit_names[] = {"ns", "us", "ms"};
    for (int i = 0; i < 3; ++i) {
      const double scaled_median = median * scales[i];
      if (std::abs(scaled_median - header_stamp_sec) < absolute_tolerance_s) {
        result.timestamps.reserve(n);
        for (const double v : raw) {
          result.timestamps.push_back(v * scales[i]);
        }
        result.mode = TimestampMode::kConvertedUnits;
        converted = true;
        (void)unit_names;
        break;
      }
    }
    if (!converted) {
      // 5. Relative scan time (small magnitude, short span).
      if (std::abs(median) < 10.0 && span < 0.2) {
        result.mode = TimestampMode::kRelativeScan;
        result.timestamps.reserve(n);
        for (const double v : raw) {
          result.timestamps.push_back(header_stamp_sec + v);
        }
      } else {
        // 6. Unclassifiable: explicit error, never silent guessing.
        result.mode = TimestampMode::kError;
        return result;
      }
    }
  }

  // 7. Monotonicity check (light tolerance for duplicate equal stamps).
  bool non_monotonic = false;
  for (std::size_t i = 1; i < result.timestamps.size(); ++i) {
    if (result.timestamps[i] + 1e-6 < result.timestamps[i - 1]) {
      non_monotonic = true;
      break;
    }
  }
  result.non_monotonic = non_monotonic;
  return result;
}

}  // namespace go2w_plain_slam_bridge

#endif  // GO2W_PLAIN_SLAM_BRIDGE__TIMESTAMP_POLICY_HPP_
