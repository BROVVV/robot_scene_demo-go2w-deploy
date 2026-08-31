/*
 * plain_slam_ros2
 * 
 * Copyright (c) 2025 Naoki Akai
 * All rights reserved.
 *
 * This software is provided free of charge for academic and personal use only.
 * Commercial use is strictly prohibited without prior written permission from the author.
 *
 * Conditions:
 *   - Non-commercial use only
 *   - Attribution required in any academic or derivative work
 *   - Redistribution permitted with this license header intact
 *
 * Disclaimer:
 *   This software is provided "as is" without warranty of any kind.
 *   The author is not liable for any damages arising from its use.
 *
 * For commercial licensing inquiries, please contact:
 *   Naoki Akai
 *   Email: n.akai.goo[at]gmail.com   ([at] -> @)
 *   Subject: [plain_slam_ros2] Commercial License Inquiry
 */

#pragma once

#include <fstream>
#include <iostream>
#include <vector>

#include <plain_slam/types.hpp>

namespace pslam {

bool WritePointCloud(
  const std::string& fname,
  const PointCloud3f& cloud);

bool WritePointCloud(
  const std::string& fname,
  const PointCloud3f& cloud,
  const std::vector<float>& intensities);

bool ReadPointCloud(
  const std::string& fname,
  PointCloud3f& cloud);

bool ReadPointCloud(
  const std::string& fname,
  PointCloud3f& cloud,
  std::vector<float>& intensities);

bool WriteBinaryPCD(
  const std::string& fname,
  const PointCloud3f& cloud);

bool WriteBinaryPCD(
  const std::string& fname,
  const PointCloud3f& cloud,
  const std::vector<float>& intensities);

bool ReadPCD(
  const std::string& fname,
  PointCloud3f& cloud,
  std::vector<float>& intensities);

} // namespace pslam
