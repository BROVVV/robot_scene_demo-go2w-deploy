// SPDX-License-Identifier: Apache-2.0
#ifndef GO2W_NAV2_CONTROLLER_COMPAT__CONTROLLER_PARAMETERS_HPP_
#define GO2W_NAV2_CONTROLLER_COMPAT__CONTROLLER_PARAMETERS_HPP_

#include <cmath>
#include <mutex>
#include <string>
#include <vector>
#include "rclcpp/parameter.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "go2w_nav2_controller_compat/controller_patience.hpp"

namespace go2w_foxy_controller
{
// As in the Humble server, core settings cannot change during an action;
// plugin-scoped speed updates remain handled by the plugin's own callback.
class ControllerParameters
{
public:
  std::mutex & actionMutex() {return action_mutex_;}

  rcl_interfaces::msg::SetParametersResult validate(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    bool core = false;
    for (const auto & parameter : parameters) {
      core = core || parameter.get_name().find('.') == std::string::npos;
    }
    std::unique_lock<std::mutex> lock(action_mutex_, std::defer_lock);
    if (core && !lock.try_lock()) {
      result.successful = false;
      result.reason = "Unable to dynamically change Parameters while the controller is currently running";
      return result;
    }
    for (const auto & parameter : parameters) {
      const auto & name = parameter.get_name();
      if (name == "publish_zero_velocity") {
        if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
          result.successful = false;
          result.reason = "invalid controller parameter: " + name;
          return result;
        }
        continue;
      }
      if (name != "failure_tolerance" && name != "controller_frequency" &&
        name != "min_x_velocity_threshold" && name != "min_y_velocity_threshold" &&
        name != "min_theta_velocity_threshold")
      {
        continue;
      }
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
        result.successful = false;
      } else {
        const double value = parameter.as_double();
        result.successful = name == "failure_tolerance" ? validFailureTolerance(value) :
          (std::isfinite(value) && (name == "controller_frequency" ? value > 0.0 : value >= 0.0));
      }
      if (!result.successful) {
        result.reason = "invalid controller parameter: " + name;
        return result;
      }
    }
    // Do not mutate cached values in a pre-commit callback. The server reads
    // committed values under actionMutex when beginning each distinct action.
    return result;
  }

private:
  std::mutex action_mutex_;
};
}  // namespace go2w_foxy_controller
#endif  // GO2W_NAV2_CONTROLLER_COMPAT__CONTROLLER_PARAMETERS_HPP_
