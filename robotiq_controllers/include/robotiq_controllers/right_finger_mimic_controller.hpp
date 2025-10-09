#ifndef ROBOTIQ_CONTROLLERS__RIGHT_FINGER_MIMIC_CONTROLLER_HPP_
#define ROBOTIQ_CONTROLLERS__RIGHT_FINGER_MIMIC_CONTROLLER_HPP_

#include <memory>
#include <string>

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace robotiq_controllers
{

class RightFingerMimicController : public controller_interface::ControllerInterface
{
public:
  RightFingerMimicController();

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::string main_joint_name_;
  std::string finger_joint_name_;
  double mimic_multiplier_;
  double mimic_offset_;
  double max_finger_position_;
};

}  // namespace robotiq_controllers

#endif  // ROBOTIQ_CONTROLLERS__RIGHT_FINGER_MIMIC_CONTROLLER_HPP_