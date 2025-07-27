#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <map>
#include <string>

namespace articubot_gpio
{

class ArticubotGPIOHardware : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override
  {
    info_ = info;
    RCLCPP_INFO(rclcpp::get_logger("ArticubotGPIO"), "Hardware interface initialized");
    // Initialize GPIO pins here (e.g. setup wiringPi, pigpio, or sysfs GPIO)
    return hardware_interface::CallbackReturn::OK;
  }

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (const auto & joint : info_.joints) {
      state_interfaces.emplace_back(
        hardware_interface::StateInterface(joint.name, hardware_interface::HW_IF_VELOCITY, &velocity_states_[joint.name]));
    }
    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (const auto & joint : info_.joints) {
      command_interfaces.emplace_back(
        hardware_interface::CommandInterface(joint.name, hardware_interface::HW_IF_VELOCITY, &velocity_commands_[joint.name]));
    }
    return command_interfaces;
  }

  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    RCLCPP_INFO(rclcpp::get_logger("ArticubotGPIO"), "Hardware interface activated");
    return hardware_interface::CallbackReturn::OK;
  }

  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    RCLCPP_INFO(rclcpp::get_logger("ArticubotGPIO"), "Hardware interface deactivated");
    return hardware_interface::CallbackReturn::OK;
  }

  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    // TODO: Read encoder counts or motor velocity from GPIO or hardware interface and update velocity_states_
    // For example:
    // velocity_states_["left_wheel"] = read_encoder_left();
    // velocity_states_["right_wheel"] = read_encoder_right();
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    // TODO: Write velocity commands to motor driver GPIO pins (e.g. PWM signals)
    // For example:
    // set_pwm_left(velocity_commands_["left_wheel"]);
    // set_pwm_right(velocity_commands_["right_wheel"]);
    return hardware_interface::return_type::OK;
  }

private:
  hardware_interface::HardwareInfo info_;
  std::map<std::string, double> velocity_states_;
  std::map<std::string, double> velocity_commands_;
};

}  // namespace articubot_gpio

PLUGINLIB_EXPORT_CLASS(articubot_gpio::ArticubotGPIOHardware, hardware_interface::SystemInterface)
