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

#include <iostream>
#include <chrono>

#include <Eigen/Geometry>

#include <plain_slam/types.hpp>
#include <plain_slam/io.hpp>
#include <plain_slam/gicp.hpp>

int main() {
  const std::string souce_pcd_file = "./src/plain_slam_ros2/data/left_mid360.pcd";
  const std::string target_pcd_file = "./src/plain_slam_ros2/data/right_mid360.pcd";
  const std::string output_pcd_file = "./src/plain_slam_ros2/data/aligned.pcd";

  pslam::PointCloud3f source_cloud;
  std::vector<float> source_intensities;
  if (pslam::ReadPCD(souce_pcd_file, source_cloud, source_intensities) == false) {
    std::cerr << "[Error] Failed to read " << souce_pcd_file << std::endl;
    return 1;
  }

  pslam::PointCloud3f target_cloud;
  std::vector<float> target_intensities;
  if (pslam::ReadPCD(target_pcd_file, target_cloud, target_intensities) == false) {
    std::cerr << "[Error] Failed to read " << target_pcd_file << std::endl;
    return 1;
  }

  pslam::GICP gicp;
  gicp.SetMaxIteration(50);
  gicp.SetMaxCorrespondenceDist(1.0f);
  gicp.SetHuberDelta(0.1f);
  gicp.SetSourceCloud(source_cloud, true);
  gicp.SetTargetCloud(target_cloud, true);

  // Set the initial transformation if you have the initial guess.
  // Sophus::SE3f T_init;
  // gicp.SetTransformation(T_init);

  const auto t1 = std::chrono::high_resolution_clock::now();

  gicp.Align();

  const auto t2 = std::chrono::high_resolution_clock::now();
  std::cout << "elapsed time [msec]: " 
    << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() << std::endl;

  const Sophus::SE3f T = gicp.GetTransformation();
  const Eigen::Quaternionf q = Eigen::Quaternionf(T.rotationMatrix());
  const Eigen::Vector3f t = T.translation();
  std::cout << "T:" << std::endl << T.matrix() << std::endl;
  std::cout << "quaternion:" << std::endl << q.coeffs().transpose() << std::endl;  // Eigen >= 3.4 ostream op missing on 3.3
  std::cout << "translation:" << std::endl << t << std::endl;

  pslam::PointCloud3f aligned_cloud;
  gicp.Transform(source_cloud, aligned_cloud);

  pslam::WriteBinaryPCD(output_pcd_file, aligned_cloud, source_intensities);

  return 0;
}
