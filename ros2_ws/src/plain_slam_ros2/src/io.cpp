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

#include <plain_slam/io.hpp>

namespace pslam {

bool WritePointCloud(
  const std::string& fname,
  const PointCloud3f& cloud) {
  std::ofstream ofs(fname, std::ios::binary);
  if (!ofs) {
    std::cerr << "[ERROR] Cannot open file for writing: " << fname << std::endl;
    return false;
  }

  uint32_t num_points = static_cast<uint32_t>(cloud.size());
  ofs.write(reinterpret_cast<const char*>(&num_points), sizeof(uint32_t));

  for (size_t i = 0; i < cloud.size(); ++i) {
    const Eigen::Vector3f& p = cloud[i];
    const float data[3] = {p.x(), p.y(), p.z()};
    ofs.write(reinterpret_cast<const char*>(data), sizeof(float) * 3);
  }

  ofs.close();

  return true;
}

bool WritePointCloud(
  const std::string& fname,
  const PointCloud3f& cloud,
  const std::vector<float>& intensities) {
  if (cloud.size() != intensities.size()) {
    std::cerr << "[ERROR] cloud.size() != intensities.size()" << std::endl;
    return false;
  }

  std::ofstream ofs(fname, std::ios::binary);
  if (!ofs) {
    std::cerr << "[ERROR] Cannot open file for writing: " << fname << std::endl;
    return false;
  }

  uint32_t num_points = static_cast<uint32_t>(cloud.size());
  ofs.write(reinterpret_cast<const char*>(&num_points), sizeof(uint32_t));

  for (size_t i = 0; i < cloud.size(); ++i) {
    const Eigen::Vector3f& p = cloud[i];
    const float data[4] = {p.x(), p.y(), p.z(), intensities[i]};
    ofs.write(reinterpret_cast<const char*>(data), sizeof(float) * 4);
  }

  ofs.close();

  return true;
}

bool ReadPointCloud(
  const std::string& fname,
  PointCloud3f& cloud) {
  std::ifstream ifs(fname, std::ios::binary);
  if (!ifs) {
    std::cerr << "[ERROR] Cannot open file for reading: " << fname << std::endl;
    return false;
  }

  uint32_t num_points = 0;
  ifs.read(reinterpret_cast<char*>(&num_points), sizeof(uint32_t));
  if (!ifs) {
    std::cerr << "[ERROR] Failed to read number of points" << std::endl;
    return false;
  }

  cloud.clear();
  cloud.resize(num_points);

  for (uint32_t i = 0; i < num_points; ++i) {
    float data[3];
    ifs.read(reinterpret_cast<char*>(data), sizeof(float) * 3);
    if (!ifs) {
      std::cerr << "[ERROR] Failed to read point data at index " << i << std::endl;
      return false;
    }
    cloud[i] = Eigen::Vector3f(data[0], data[1], data[2]);
  }

  ifs.close();

  return true;
}

bool ReadPointCloud(
  const std::string& fname,
  PointCloud3f& cloud,
  std::vector<float>& intensities) {
  std::ifstream ifs(fname, std::ios::binary);
  if (!ifs) {
    std::cerr << "[ERROR] Cannot open file for reading: " << fname << std::endl;
    return false;
  }

  uint32_t num_points = 0;
  ifs.read(reinterpret_cast<char*>(&num_points), sizeof(uint32_t));
  if (!ifs) {
    std::cerr << "[ERROR] Failed to read number of points" << std::endl;
    return false;
  }

  cloud.clear();
  intensities.clear();
  cloud.resize(num_points);
  intensities.resize(num_points);

  for (uint32_t i = 0; i < num_points; ++i) {
    float data[4];
    ifs.read(reinterpret_cast<char*>(data), sizeof(float) * 4);
    if (!ifs) {
      std::cerr << "[ERROR] Failed to read point data at index " << i << std::endl;
      return false;
    }
    cloud[i] = Eigen::Vector3f(data[0], data[1], data[2]);
    intensities[i] = data[3];
  }

  ifs.close();

  return true;
}

bool WriteBinaryPCD(
  const std::string& fname,
  const PointCloud3f& cloud) {
  std::ofstream ofs(fname, std::ios::binary);
  if (!ofs) {
    std::cerr << "[ERROR] Cannot open file: " << fname << std::endl;
    return false;
  }

  const size_t num_points = cloud.size();
  const std::string header =
    "VERSION .7\n"
    "FIELDS x y z\n"
    "SIZE 4 4 4\n"
    "TYPE F F F\n"
    "COUNT 1 1 1\n"
    "WIDTH " + std::to_string(num_points) + "\n"
    "HEIGHT 1\n"
    "VIEWPOINT 0 0 0 1 0 0 0\n"
    "POINTS " + std::to_string(num_points) + "\n"
    "DATA binary\n";

  ofs.write(header.c_str(), header.size());

  for (size_t i = 0; i < num_points; ++i) {
    const float data[3] = {cloud[i].x(), cloud[i].y(), cloud[i].z()};
    ofs.write(reinterpret_cast<const char*>(data), sizeof(float) * 3);
  }

  ofs.close();

  return true;
}

bool WriteBinaryPCD(
  const std::string& fname,
  const PointCloud3f& cloud,
  const std::vector<float>& intensities) {
  if (cloud.size() != intensities.size()) {
    std::cerr << "[ERROR] cloud size != intensity size" << std::endl;
    return false;
  }

  std::ofstream ofs(fname, std::ios::binary);
  if (!ofs) {
    std::cerr << "[ERROR] Cannot open file: " << fname << std::endl;
    return false;
  }

  const size_t num_points = cloud.size();
  const std::string header =
    "VERSION .7\n"
    "FIELDS x y z intensity\n"
    "SIZE 4 4 4 4\n"
    "TYPE F F F F\n"
    "COUNT 1 1 1 1\n"
    "WIDTH " + std::to_string(num_points) + "\n"
    "HEIGHT 1\n"
    "VIEWPOINT 0 0 0 1 0 0 0\n"
    "POINTS " + std::to_string(num_points) + "\n"
    "DATA binary\n";

  ofs.write(header.c_str(), header.size());

  for (size_t i = 0; i < num_points; ++i) {
    const float data[4] = {cloud[i].x(), cloud[i].y(), cloud[i].z(), intensities[i]};
    ofs.write(reinterpret_cast<const char*>(data), sizeof(float) * 4);
  }

  ofs.close();

  return true;
}

bool ReadPCD(
  const std::string& fname,
  PointCloud3f& cloud,
  std::vector<float>& intensities) {
  std::ifstream ifs(fname, std::ios::binary);
  if (!ifs) {
    std::cerr << "[ERROR] Cannot open file: " << fname << std::endl;
    return false;
  }

  std::string line;
  size_t num_points = 0;
  bool has_intensity = false;
  std::string data_type;
  size_t header_end_pos = 0;

  while (std::getline(ifs, line)) {
    if (line.substr(0, 6) == "FIELDS") {
      std::istringstream ss(line.substr(7));
      std::string field;
      while (ss >> field) {
        if (field == "intensity") {
          has_intensity = true;
        }
      }
    } else if (line.substr(0, 6) == "POINTS") {
      num_points = std::stoul(line.substr(7));
    } else if (line.substr(0, 4) == "DATA") {
      std::istringstream ss(line);
      std::string token;
      ss >> token >> data_type;
      header_end_pos = static_cast<size_t>(ifs.tellg());
      break;
    }
  }

  if (num_points == 0 || (data_type != "binary" && data_type != "ascii")) {
    std::cerr << "[ERROR] Unsupported or malformed PCD file." << std::endl;
    return false;
  }

  cloud.clear();
  intensities.clear();
  cloud.reserve(num_points);
  if (has_intensity) {
    intensities.reserve(num_points);
  }

  if (data_type == "binary") {
    ifs.seekg(header_end_pos);
    for (size_t i = 0; i < num_points; ++i) {
      float xyz[3];
      ifs.read(reinterpret_cast<char*>(xyz), sizeof(float) * 3);
      cloud.emplace_back(xyz[0], xyz[1], xyz[2]);

      if (has_intensity) {
        float intensity;
        ifs.read(reinterpret_cast<char*>(&intensity), sizeof(float));
        intensities.push_back(intensity);
      }
    }

  } else if (data_type == "ascii") {
    std::string data_line;
    while (std::getline(ifs, data_line)) {
      std::istringstream ss(data_line);
      float x, y, z;
      ss >> x >> y >> z;
      cloud.emplace_back(x, y, z);

      if (has_intensity) {
        float intensity;
        ss >> intensity;
        intensities.push_back(intensity);
      }
    }
  }

  return true;
}

} // namespace pslam
