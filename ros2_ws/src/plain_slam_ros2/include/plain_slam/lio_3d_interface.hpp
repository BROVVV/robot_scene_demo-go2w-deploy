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

#include <iostream>
#include <vector>
#include <mutex>
#include <filesystem> 

#include <yaml-cpp/yaml.h>

#include <boost/circular_buffer.hpp>

#include <plain_slam/types.hpp>
#include <plain_slam/io.hpp>
#include <plain_slam/gravity_direc_estimator.hpp>
#include <plain_slam/imu_preintegrator.hpp>
#include <plain_slam/keyframe_detector.hpp>
#include <plain_slam/voxel_grid_filter.hpp>
#include <plain_slam/normal_map.hpp>
#include <plain_slam/hierarchical_geometric_observer.hpp>
#include <plain_slam/joint_optimizer.hpp>
// #include <plain_slam/lidar_imu_calibration.hpp>

namespace pslam {

class LIO3DInterface {
 public:
  LIO3DInterface();

  ~LIO3DInterface();

  void ReadLIOParams();

  void SetScanCloud(
    const PointCloud3f& scan_cloud,
    const std::vector<float>& scan_intensities,
    const std::vector<double>& scan_stamps);
   
  void SetIMUMeasure(const IMUMeasure& measure);

  bool ReadMapCloud(const std::string map_cloud_file);

  bool ReadMapCloudPCD(const std::string& map_cloud_dir);

  void SetLocalizationMode(bool enable) {
    localization_mode_ = enable;
  }

  void SetParamFilesDir(const std::string& dir) {
    param_files_dir_ = dir;
  } 

  void EnableGravityEstimation() {
    gravity_estimation_enabled_ = true;
  }

  bool IsGravityEstimationEnabled() const {
    return gravity_estimation_enabled_;
  }

  const PointCloud3f& GetNormalMapCloud() const {
    return normal_map_.GetMapCloud();
  }

  bool IsMapUpdated() const {
    return is_map_updated_;
  }

  void SetMapUpdated(bool flag) {
    is_map_updated_ = flag;
  }

  const PointCloud3f& GetAlignedScanCloud() const {
    return aligned_scan_cloud_;
  }

  const PointCloud3f& GetDeskewedScanCloud() const {
    return scan_cloud_;
  }

  const std::vector<float> GetClippedScanIntensities() const {
    return scan_intensities_;
  }

  const std::vector<double> GetClippedScanStamps() const {
    return scan_stamps_;
  }

  const Sophus::SE3f GetIMUPose() const {
    return imu_state_.T;
  }

  void GetActiveMapCloud(PointCloud3f& cloud) const {
    normal_map_.GetActiveMapCloud(cloud);
  }

  void GetIMUOdometry(
    State& state,
    StateCov& state_cov) {
    {
      std::lock_guard<std::mutex> lock(imu_mutex_);
      state = imu_odom_state_;
      state_cov = imu_odom_state_cov_;
    }
    state.v = state.T.rotationMatrix().inverse() * state.v;
  }

 private:
  bool localization_mode_;
  std::string param_files_dir_;

  State imu_state_;
  StateCov imu_state_cov_;
  State imu_odom_state_;
  StateCov imu_odom_state_cov_;

  float acc_scale_;
  GravityDirecEstimator g_estimator_;
  IMUPreintegrator preintegrator_;
  KeyframeDetector kf_detector_;
  float min_active_points_rate_;

  PointCloud3f scan_cloud_;
  PointCloud3f aligned_scan_cloud_;
  std::vector<float> scan_intensities_;
  std::vector<double> scan_stamps_;

  float scan_cloud_clip_range_;
  float scan_cloud_filter_size_;

  boost::circular_buffer<IMUMeasure> imu_measures_;
  std::mutex imu_mutex_;

  bool gravity_estimation_enabled_;

  bool is_map_initialized_;
  NormalMap normal_map_;
  double last_map_updated_stamp_;

  bool is_map_updated_;

  bool use_loose_coupling_;
  size_t num_max_iteration_;
  size_t num_max_matching_points_;
  float max_correspondence_dist_;
  float optimization_convergence_th_;

  HierarchicalGeometricObserver hg_observer_;
  JointOptimizer joint_optimizer_;

  // LiDARIMUCalibration li_calibrator_;

  void PrintIMUMeasure(const IMUMeasure& measure) {
    std::cout << "stamp = " << measure.stamp << std::endl;
    std::cout << "dt = " << measure.dt << std::endl;
    std::cout << "ax = " << measure.acc.x() << " ay = " << measure.acc.y() << " az = " << measure.acc.z() << std::endl;
    std::cout << "gx = " << measure.gyro.x() << " gy = " << measure.gyro.y() << " gz = " << measure.gyro.z() << std::endl;
  }

  void PrintState(const State& state) {
    std::cout << "T" << std::endl << state.T.matrix() << std::endl;    
    std::cout << "vx = " << state.v.x() << ", vy = " << state.v.y() << ", vz = " << state.v.z() << std::endl;
    std::cout << "gbx = " << state.gb.x() << ", gby = " << state.gb.y() << ", gbz = " << state.gb.z() << std::endl;
    std::cout << "abx = " << state.ab.x() << ", aby = " << state.ab.y() << ", abz = " << state.ab.z() << std::endl;
    std::cout << "gx = " << state.gacc.x() << ", gy = " << state.gacc.y() << ", gz = " << state.gacc.z() << std::endl;
    std::cout << "Til" << std::endl << state.Til.matrix() << std::endl;    
  }

  void WriteLiDARPose(double stamp, const State& state) {
    static std::ofstream ofs("/tmp/pslam_lidar_pose.txt", std::ios::out | std::ios::trunc);
    static std::once_flag once;
    std::call_once(once, []{
      ofs.setf(std::ios::fixed, std::ios::floatfield);
      ofs << std::setprecision(9);
    });
  
    const Sophus::SE3f Tol = state.T * state.Til;
    const Eigen::Vector3f t = Tol.translation();
    const Eigen::Quaternionf q(Tol.rotationMatrix());
    ofs << stamp << ' '
        << t.x()   << ' ' << t.y()   << ' ' << t.z()   << ' '
        << q.w()   << ' ' << q.x()   << ' ' << q.y()   << ' ' << q.z() << '\n';
  }
};

} // namespace pslam
