#include "go2w_state_transport/state_codec.hpp"
#include <rclcpp/rclcpp.hpp>
#include <unitree_go/msg/low_state.hpp>
#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>

namespace transport=go2w_state_transport;
class LocalLowRelay final : public rclcpp::Node {
public:
  LocalLowRelay(const std::string & input,const std::string & topic,double age)
  : Node("go2w_local_low_state_relay"),input_(input),boot_(transport::boot_identity()),max_age_(age)
  {
    publisher_=create_publisher<unitree_go::msg::LowState>(topic,rclcpp::QoS(20).reliable());
    timer_=create_wall_timer(std::chrono::milliseconds(20),[this]() {
      const auto text=transport::bounded_read(input_);if (!text) {return;}
      const auto s=transport::decode(*text,transport::monotonic_now(),transport::wall_now(),max_age_,boot_);
      if (!s || s->monotonic<=last_) {return;}
      unitree_go::msg::LowState m;
      for (std::size_t i=0;i<16;++i) {m.motor_state[i].q=s->q[i];m.motor_state[i].dq=s->dq[i];}
      m.imu_state.quaternion=s->quaternion;m.imu_state.gyroscope=s->gyroscope;
      m.imu_state.accelerometer=s->accelerometer;m.imu_state.rpy=s->rpy;
      m.imu_state.temperature=s->temperature;m.bms_state.soc=s->soc;
      publisher_->publish(m);last_=s->monotonic;
    });
  }
private:
  std::string input_,boot_;double max_age_,last_=0.;
  rclcpp::Publisher<unitree_go::msg::LowState>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char ** argv) {
  try {
    std::string input,topic="/go2w/motion/local_lowstate";double age=.30;
    for (int i=1;i<argc;++i) {
      const std::string arg=argv[i];
      if (arg=="--input" && i+1<argc) {input=argv[++i];}
      else if (arg=="--output-topic" && i+1<argc) {topic=argv[++i];}
      else if (arg=="--max-age" && i+1<argc) {
        const std::string value=argv[++i];std::size_t used=0;age=std::stod(value,&used);
        if (used!=value.size()) {throw std::invalid_argument("invalid max age");}
      }
      else if (arg=="--help") {std::cout<<"Read-only native relay --input FILE [--output-topic TOPIC] [--max-age SECONDS]\n";return 0;}
      else {throw std::invalid_argument("unknown or incomplete argument: "+arg);}
    }
    if (input.empty() || !std::isfinite(age) || age<=0.) {throw std::invalid_argument("invalid input path or max age");}
    rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<LocalLowRelay>(input,topic,age));
    rclcpp::shutdown();return 0;
  } catch (const std::exception & e) {std::cerr<<e.what()<<'\n';return 2;}
}
