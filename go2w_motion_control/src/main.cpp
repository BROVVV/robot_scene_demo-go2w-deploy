#include "go2w_motion_control/motion_action_server.hpp"

#include <chrono>
#include <exception>
#include <stdexcept>
#include <thread>

#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<go2w_motion_control::MotionActionServer>();
  // Motion execution already runs in MotionActionServer's dedicated worker
  // thread.  Keeping ROS entity dispatch single-threaded avoids a Foxy
  // rclcpp_action wait-set race ("Executing action server but nothing is
  // ready") that can otherwise terminate a MultiThreadedExecutor while the
  // worker publishes feedback or completes a goal.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  // A goal completion can still invalidate a Foxy action wait-set between
  // readiness inspection and dispatch.  With a single-threaded executor the
  // exception is raised on this thread, where it can be recovered safely.
  while (rclcpp::ok()) {
    try {
      executor.spin();
    } catch (const std::runtime_error &caught) {
      RCLCPP_ERROR(node->get_logger(), "Foxy executor recovered: %s",
                   caught.what());
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
  executor.remove_node(node);
  node.reset();
  rclcpp::shutdown();
  return 0;
}
