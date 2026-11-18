// Adapter for the installed slam_toolbox 2.6.x public/protected API.
// Continuous REAL scans advance the upstream scan_header; only frontend
// keyframes enter scan matching. No synthetic time, odometry or TF publisher.
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include "slam_toolbox/slam_toolbox_async.hpp"
#include "std_msgs/msg/string.hpp"

class KeyframeSlamToolbox : public slam_toolbox::AsynchronousSlamToolbox
{
public:
  explicit KeyframeSlamToolbox(const rclcpp::NodeOptions & options)
  : AsynchronousSlamToolbox(options)
  {
    map_id_ = declare_parameter<std::string>("map_id", "runtime_map");
    map_epoch_ = declare_parameter<std::string>("map_epoch", "");
    sensor_profile_id_ = declare_parameter<std::string>("sensor_profile_id", "unknown");
    mapping_use_keyframes_ = declare_parameter<bool>("mapping_use_keyframes", true);
    direct_scan_minimum_interval_s_ = declare_parameter<double>(
      "direct_scan_minimum_interval_s", 0.0);
    keyframe_minimum_translation_m_ = declare_parameter<double>(
      "mapping_keyframe_minimum_translation_m", 0.05);
    keyframe_minimum_rotation_rad_ = declare_parameter<double>(
      "mapping_keyframe_minimum_rotation_rad", 0.05);
    if (!std::isfinite(keyframe_minimum_translation_m_) ||
      !std::isfinite(keyframe_minimum_rotation_rad_) ||
      keyframe_minimum_translation_m_ <= 0.0 || keyframe_minimum_rotation_rad_ <= 0.0 ||
      !std::isfinite(direct_scan_minimum_interval_s_) || direct_scan_minimum_interval_s_ < 0.0)
    {
      throw std::runtime_error("mapping keyframe thresholds must be finite and positive");
    }
    if (map_epoch_.empty()) {
      map_epoch_ = "epoch_" + std::to_string(get_clock()->now().nanoseconds());
    }
    auto qos = rclcpp::QoS(1).reliable().transient_local();
    graph_pub_ = create_publisher<std_msgs::msg::String>(
      "/go2w/slam/keyframe_graph", qos);
    graph_timer_ = create_wall_timer(
      std::chrono::milliseconds(500), [this]() {publishKeyframeGraph();});
  }

  void startContinuousScans()
  {
    const auto topic = declare_parameter<std::string>(
      "continuous_scan_topic", "/go2w/lidar/mapping_scan");
    // Direct-scan mode uses the upstream scan subscription as the sole input.
    // It deliberately accepts fresh scans while stationary; no second
    // subscription may feed the same message into the graph twice.
    if (!mapping_use_keyframes_) {return;}
    if (topic == scan_topic_) {
      throw std::runtime_error("continuous scans must be separate from keyframes");
    }
    continuous_ = create_subscription<sensor_msgs::msg::LaserScan>(
      topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::ConstSharedPtr scan) {
        if (!current(scan)) {return;}
        // A new mapper may miss the frontend's one-shot initial keyframe
        // while its acquisition-time TF cache is still empty. Bootstrap from
        // a subsequent real scan; laserCallback retains all pose/time/pause
        // checks. Once initialized, continuous scans never add map nodes.
        if (!mapped_) {
          laserCallback(scan);
          return;
        }
        karto::Pose2 pose;
        // Real acquisition-time odometry is required before advancing the
        // same header the upstream async callback updates on skipped scans.
        if (!pose_helper_->getOdomPose(pose, scan->header.stamp)) {return;}
        advanceHeader(scan);
      });
  }

protected:
  void laserCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr scan) override
  {
    if (!current(scan) || isPaused(slam_toolbox::NEW_MEASUREMENTS)) {return;}
    const auto scan_stamp_ns = rclcpp::Time(scan->header.stamp).nanoseconds();
    karto::Pose2 pose;
    if (!pose_helper_->getOdomPose(pose, scan->header.stamp)) {return;}
    if (!mapping_use_keyframes_ && last_direct_scan_stamp_ns_ > 0) {
      const double elapsed = static_cast<double>(scan_stamp_ns - last_direct_scan_stamp_ns_) * 1e-9;
      if (!std::isfinite(elapsed) || elapsed <= 0.0) {return;}
      if (direct_scan_minimum_interval_s_ > 0.0 &&
        elapsed < direct_scan_minimum_interval_s_ - 1e-6)
      {
        // A skipped real scan still proves the acquisition-time odometry
        // needed to stamp the unchanged map->odom transform.  Keeping that
        // transform current is cheap and independent of scan matching.
        advanceHeader(scan);
        return;
      }
    }
    // Independently reject duplicate graph nodes, including direct-scan mode.
    // The old direct path added four stationary scans every second forever;
    // after roughly twenty minutes its ever-growing graph saturated a core,
    // delayed map->odom by seconds and made localization fail.  Real scans
    // still refresh scan_header above/below, while the graph grows only after
    // physical translation or yaw exceeds the configured keyframe threshold.
    if (mapped_) {
      const double moved = std::sqrt(last_pose_.SquaredDistance(pose));
      const double turned = std::abs(std::remainder(
        pose.GetHeading()-last_pose_.GetHeading(), 2.0*std::acos(-1.0)));
      if (moved < keyframe_minimum_translation_m_ - 1e-6 &&
        turned < keyframe_minimum_rotation_rad_ - 1e-6)
      {
        if (!mapping_use_keyframes_) {
          last_direct_scan_stamp_ns_ = scan_stamp_ns;
          advanceHeader(scan);
        }
        return;
      }
    }
    auto * laser = getLaser(scan);
    if (!laser) {return;}
    // Upstream shouldProcessScan is deliberately replaced: 2.6.x checks only
    // distance and drops pure turns. Karto's internal distance gate is zero;
    // both frontend and this adapter enforce translation OR yaw instead.
    if (addScan(laser, scan, pose)) {
      last_pose_ = pose;
      if (!mapping_use_keyframes_) {last_direct_scan_stamp_ns_ = scan_stamp_ns;}
      mapped_ = true;
      first_measurement_ = false;
      advanceHeader(scan);
    }
  }

private:
  void publishKeyframeGraph()
  {
    struct NodeSnapshot
    {
      std::uint64_t unique_id;
      int state_id;
      double source_stamp;
      double x;
      double y;
      double yaw;
    };
    std::vector<NodeSnapshot> nodes_snapshot;
    {
      boost::mutex::scoped_lock lock(smapper_mutex_);
      auto * mapper = smapper_ ? smapper_->getMapper() : nullptr;
      if (!mapper) {return;}
      for (const auto * scan : mapper->GetAllProcessedScans()) {
        if (!scan) {continue;}
        const auto & pose = scan->GetCorrectedPose();
        nodes_snapshot.push_back(NodeSnapshot{
          static_cast<std::uint64_t>(scan->GetUniqueId()),
          static_cast<int>(scan->GetStateId()), scan->GetTime(),
          pose.GetX(), pose.GetY(), pose.GetHeading()});
      }
    }
    std::ostringstream nodes;
    nodes << std::setprecision(12);
    nodes << "[";
    std::ostringstream signature;
    signature << std::setprecision(12);
    bool first = true;
    for (const auto & scan : nodes_snapshot) {
      if (!first) {nodes << ",";}
      first = false;
      nodes << "{\"anchor_id\":\"keyframe_" << scan.unique_id
            << "\",\"unique_id\":" << scan.unique_id
            << ",\"state_id\":" << scan.state_id
            << ",\"source_stamp\":" << scan.source_stamp
            << ",\"corrected_pose\":[" << scan.x << ","
            << scan.y << "," << scan.yaw << "]}";
      signature << scan.unique_id << ":" << scan.x << ":"
                << scan.y << ":" << scan.yaw << ";";
    }
    nodes << "]";
    const auto current_signature = signature.str();
    if (current_signature == graph_signature_) {return;}
    graph_signature_ = current_signature;
    ++graph_revision_;
    std_msgs::msg::String message;
    std::ostringstream payload;
    payload << "{\"schema_version\":\"go2w_keyframe_graph_v1\""
            << ",\"map_id\":\"" << escapeJson(map_id_) << "\""
            << ",\"map_epoch\":\"" << escapeJson(map_epoch_) << "\""
            << ",\"frame_id\":\"" << escapeJson(map_frame_) << "\""
            << ",\"sensor_profile_id\":\"" << escapeJson(sensor_profile_id_) << "\""
            << ",\"graph_revision\":" << graph_revision_
            << ",\"node_count\":" << nodes_snapshot.size()
            << ",\"nodes\":" << nodes.str() << "}";
    message.data = payload.str();
    graph_pub_->publish(message);
  }

  static std::string escapeJson(const std::string & value)
  {
    std::ostringstream escaped;
    for (const char character : value) {
      if (character == '\\' || character == '"') {escaped << '\\';}
      escaped << character;
    }
    return escaped.str();
  }

  bool current(const sensor_msgs::msg::LaserScan::ConstSharedPtr & scan)
  {
    const double age = (now()-rclcpp::Time(scan->header.stamp)).seconds();
    return scan->header.frame_id == base_frame_ && !scan->ranges.empty() &&
      std::isfinite(age) && age >= -.05 && age <= .20;
  }

  void advanceHeader(const sensor_msgs::msg::LaserScan::ConstSharedPtr & scan)
  {
#ifdef GO2W_FOXY_SLAM_TOOLBOX
    // Foxy SlamToolbox has no scan_header member. Its own TF timer supplies
    // timestamps; live TF freshness still needs a sensor-backed acceptance.
    (void)scan;
#else
    boost::mutex::scoped_lock lock(map_to_odom_mutex_);
    if (rclcpp::Time(scan->header.stamp) > rclcpp::Time(scan_header.stamp)) {
      scan_header = scan->header;
    }
#endif
  }

  bool mapped_{false};
  karto::Pose2 last_pose_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr continuous_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr graph_pub_;
  rclcpp::TimerBase::SharedPtr graph_timer_;
  std::string map_id_;
  std::string map_epoch_;
  std::string sensor_profile_id_;
  std::string graph_signature_;
  std::uint64_t graph_revision_{0};
  bool mapping_use_keyframes_{true};
  double direct_scan_minimum_interval_s_{0.0};
  std::int64_t last_direct_scan_stamp_ns_{0};
  double keyframe_minimum_translation_m_{0.05};
  double keyframe_minimum_rotation_rad_{0.05};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<KeyframeSlamToolbox>(rclcpp::NodeOptions());
  node->configure();
  node->startContinuousScans();
  node->loadPoseGraphByParams();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
