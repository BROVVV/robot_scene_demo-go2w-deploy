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

#include <Eigen/Cholesky>
#include <plain_slam/hierarchical_geometric_observer.hpp>

namespace pslam {

HierarchicalGeometricObserver::HierarchicalGeometricObserver() {
  gamma1_ = 4.0f; // k_q
  gamma2_ = 1.0f; // k_gb
  // k_p: dt * k_p was 0.45, i.e. only 45% of the scan-matching correction was
  // adopted per frame. The remaining 55% accumulated during pure rotation.
  gamma3_ = 9.0f; // k_p
  gamma4_ = 11.25f; // k_v
  // k_ab: dt * k_ab was 0.225, letting the accelerometer bias absorb the
  // position residual of a rotation (measured: |ab| reached 1.0 m/s^2).
  // A MEMS bias drifts far slower than that.
  gamma5_ = 0.05f; // k_ab

  huber_delta_ = 0.5f;
}

HierarchicalGeometricObserver::~HierarchicalGeometricObserver() {

}

bool HierarchicalGeometricObserver::Estimate(
  const State& pred_state,
  const PointCloud3f& scan_cloud,
  size_t max_iter_num,
  size_t num_max_matching_points,
  float max_correspondence_dist,
  float convergence_th,
  float preintegration_time,
  State& updated_state,
  NormalMap& normal_map) {
  const State initial_state = pred_state;

  int scan_step = scan_cloud.size() / num_max_matching_points;
  if (scan_step < 1) {
    scan_step = 1;
  }

  Sophus::SE3f pose = pred_state.T;
  bool has_converged = false;

  for (size_t iter_num = 0; iter_num < max_iter_num; ++iter_num) {
    Eigen::Matrix<float, 6, 6> H = Eigen::Matrix<float, 6, 6>::Zero();
    Eigen::Matrix<float, 6, 1> b = Eigen::Matrix<float, 6, 1>::Zero();

    size_t used_points_num = 0;
    size_t corresp_num = 0;

    for (size_t i = 0; i < scan_cloud.size(); i += scan_step) {
      used_points_num++;

      const Eigen::Vector3f query = pose * scan_cloud[i];
      Eigen::Vector3f target, normal;
      if (!normal_map.FindCorrespondence(query, max_correspondence_dist, target, normal)) {
        continue;
      }

      const Eigen::Matrix<float, 1, 3> nT = normal.transpose();
      const float e = nT * (target - query);
      const Eigen::Vector3f pn = nT * Sophus::SO3f::hat(query).matrix();

      Eigen::Matrix<float, 1, 6> J;
      J << -normal(0), -normal(1), -normal(2), pn(0), pn(1), pn(2);
      const Eigen::Matrix<float, 6, 1> JT = J.transpose();

      if (huber_delta_ > 0.0f) {
        const float abs_e = std::abs(e);
        const float w = (abs_e <= huber_delta_) ? 1.0f : (huber_delta_ / abs_e);
        H += w * JT * J;
        b += w * JT * e;
      } else {
        H += JT * J;
        b += JT * e;
      }

      corresp_num++;
      // [GO2W-FIX-R11] 删除上游的 corresp_num >= num_max_matching_points 提前
      // break。理由（20260904 实测）：scan_step = scan_n / num_max_matching_points
      // 已把采样点数压到 ~num_max_matching_points，该 break 是第二道冗余限流，且
      // 因 active_points_rate < 1 每帧必然提前触发：scan_n=61774 / num_max=8000
      // → scan_step=7、采样 8825 点，rate 0.937 时在第 8538 个采样点（原始索引
      // 59766/61774）就退出，末尾 3.3% 的点从不参与配准（旋转帧 rate 0.97 时
      // 达 6.5%）。雷达点按方位角顺序排列，被丢弃的恒是雷达系同一段扇区，经常量
      // 外参映射后即机体系固定方向 —— 与实测「漂移方向恒为 yaw+43°」同源。缺失
      // 扇区让点到面 ICP 的法向约束出现固定空洞，配准最优解沿该方向系统性偏移。
    }

    active_points_rate_ = static_cast<float>(corresp_num)
                        / static_cast<float>(used_points_num);
    const Eigen::LDLT<Eigen::Matrix<float, 6, 6>> solver(H);
    const Eigen::Matrix<float, 6, 1> d = solver.solve(-b);
    pose = Sophus::SE3f::exp(d) * pose;
    const float epsilon = d.norm();
    last_iter_num_ = iter_num + 1;
    last_epsilon_ = epsilon;
    if (epsilon < convergence_th) {
      has_converged = true;
      break;
    }
  }

  if (!has_converged) {
    return false;
  }

  // Observer-based update
  // Estimated
  const float dt = preintegration_time;
  const Eigen::Vector3f p_mes = pose.translation();
  const Eigen::Quaternionf q_mes = Eigen::Quaternionf(pose.rotationMatrix());

  // Preintegration
  const Eigen::Vector3f p_int = initial_state.T.translation();
  const Eigen::Quaternionf q_int = Eigen::Quaternionf(initial_state.T.rotationMatrix());
  const Eigen::Matrix3f rot_mat_int = q_int.toRotationMatrix();
  const Eigen::Vector3f v_int = initial_state.v;
  const Eigen::Vector3f ab_int = initial_state.ab;
  const Eigen::Vector3f gb_int = initial_state.gb;

  // Errors between the estimated and preintegration
  const Eigen::Vector3f pe = p_mes - p_int;
  const Eigen::Quaternionf qe = q_int.conjugate() * q_mes;
  const float real = 1.0f - abs(qe.w());
  const float sgn = (qe.w() < 0.0f) ? -1.0f : 1.0f;
  const Eigen::Quaternionf q(real, sgn * qe.x(), sgn * qe.y(), sgn * qe.z());

  // Updates
  Eigen::Quaternionf q_update = q_int * q;
  q_update.w() *= dt * gamma1_;
  q_update.x() *= dt * gamma1_;
  q_update.y() *= dt * gamma1_;
  q_update.z() *= dt * gamma1_;
  const Eigen::Vector3f gb_update = dt * gamma2_ * qe.w()
                                  * Eigen::Vector3f(qe.x(), qe.y(), qe.z());
  const Eigen::Vector3f p_update = dt * gamma3_ * pe;
  const Eigen::Vector3f v_update = dt * gamma4_ * pe;
  const Eigen::Vector3f ab_update = dt * gamma5_ * rot_mat_int.transpose() * pe;

  // New estimate
  Eigen::Quaternionf new_q(q_int.w() + q_update.w(), q_int.x() + q_update.x(),
    q_int.y() + q_update.y(), q_int.z() + q_update.z());
  new_q.normalize();
  const Sophus::SO3f new_R(new_q);
  const Eigen::Vector3f new_p = p_int + p_update;
  const Eigen::Vector3f new_v = v_int + v_update;
  const Eigen::Vector3f new_gb = gb_int - gb_update;
  const Eigen::Vector3f new_ab = ab_int - ab_update;
  const Sophus::SE3f new_T(new_R, new_p);

  updated_state.T = new_T;
  updated_state.v = new_v;
  updated_state.gb = new_gb;
  updated_state.ab = new_ab;

  return true;
}

} // namespace pslam
