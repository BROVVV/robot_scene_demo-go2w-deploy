#include "go2w_state_transport/state_codec.hpp"
#include <rclcpp/rclcpp.hpp>
#include <unitree_go/msg/low_state.hpp>
#include <iostream>
#include <memory>
#include <stdexcept>

namespace transport=go2w_state_transport;
class NetworkLowCapture final : public rclcpp::Node {
public:
  NetworkLowCapture(const std::string & output,const std::string & topic)
  : Node("go2w_network_low_state_capture"), output_(output), topic_(topic),
    boot_(transport::boot_identity())
  {
    subscription_=create_subscription<unitree_go::msg::LowState>(topic,
      rclcpp::QoS(20).reliable(), [this](const unitree_go::msg::LowState::SharedPtr m) {
        transport::LowSample s;
        s.monotonic=transport::monotonic_now();s.wall=transport::wall_now();
        s.boot_id=boot_;s.topic=topic_;
        for (std::size_t i=0;i<16;++i) {s.q[i]=m->motor_state[i].q;s.dq[i]=m->motor_state[i].dq;}
        s.quaternion=m->imu_state.quaternion;s.gyroscope=m->imu_state.gyroscope;
        s.accelerometer=m->imu_state.accelerometer;s.rpy=m->imu_state.rpy;
        s.temperature=m->imu_state.temperature;s.soc=m->bms_state.soc;
        try {transport::atomic_write(output_,transport::encode(s));}
        catch (const std::exception & e) {
          RCLCPP_ERROR_THROTTLE(get_logger(),*get_clock(),5000,"state capture failed: %s",e.what());
        }
      });
  }
private:
  std::string output_,topic_,boot_;
  rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr subscription_;
};
int main(int argc,char ** argv) {
  try {
    std::string output,topic="/lf/lowstate";
    for (int i=1;i<argc;++i) {
      const std::string arg=argv[i];
      if (arg=="--output" && i+1<argc) {output=argv[++i];}
      else if (arg=="--topic" && i+1<argc) {topic=argv[++i];}
      else if (arg=="--help") {std::cout<<"Read-only native LowState capture --output FILE [--topic TOPIC]\n";return 0;}
      else {throw std::invalid_argument("unknown or incomplete argument: "+arg);}
    }
    if (output.empty()) {throw std::invalid_argument("--output required");}
    rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<NetworkLowCapture>(output,topic));
    rclcpp::shutdown();return 0;
  } catch (const std::exception & e) {std::cerr<<e.what()<<'\n';return 2;}
}
