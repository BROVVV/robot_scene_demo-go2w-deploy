// Capture per-edge DDS publisher identities; old rclpy drops MessageInfo GIDs.
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <chrono>
#include <iomanip>
#include <map>
#include <sstream>
#include <string>

class TfAuthorities : public rclcpp::Node {
 public:
  TfAuthorities() : Node("go2w_tf_authority_monitor") {
    pub_ = create_publisher<std_msgs::msg::String>("/go2w/nav/tf_authorities", 10);
    sub_ = create_subscription<tf2_msgs::msg::TFMessage>("/tf", rclcpp::QoS(100).best_effort(),
      [this](tf2_msgs::msg::TFMessage::ConstSharedPtr msg, const rclcpp::MessageInfo & info) {
        const auto & gid = info.get_rmw_message_info().publisher_gid;
        std::ostringstream out;
        for (size_t i = 0; i < RMW_GID_STORAGE_SIZE; ++i) {
          out << std::hex << std::setw(2) << std::setfill('0') << unsigned(gid.data[i]);
        }
        for (const auto & t : msg->transforms) {
          std::string edge = t.header.frame_id + "->" + t.child_frame_id;
          if (edge == "map->odom_fused" || edge == "odom_fused->base_link") {
            owners_[edge][out.str()] = std::chrono::steady_clock::now();
          }
        }
      });
    timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() {
      auto now = std::chrono::steady_clock::now();
      std::ostringstream out;
      out << "{";
      bool first_edge = true;
      for (auto & edge : owners_) {
        if (!first_edge) out << ",";
        first_edge = false;
        out << '"' << edge.first << "\":[";
        bool first_gid = true;
        for (auto it = edge.second.begin(); it != edge.second.end();) {
          if (std::chrono::duration<double>(now - it->second).count() >= 1.0) {
            it = edge.second.erase(it);
            continue;
          }
          if (!first_gid) out << ",";
          first_gid = false;
          out << '"' << it->first << '"';
          ++it;
        }
        out << "]";
      }
      out << "}";
      std_msgs::msg::String msg;
      msg.data = out.str();
      pub_->publish(msg);
    });
  }
 private:
  using Clock = std::chrono::steady_clock;
  std::map<std::string, std::map<std::string, Clock::time_point>> owners_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TfAuthorities>());
  rclcpp::shutdown();
  return 0;
}
