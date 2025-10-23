#include "robotiq_controllers/right_finger_mimic_controller.hpp"
#include <algorithm>
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace robotiq_controllers
{

RightFingerMimicController::RightFingerMimicController()
: controller_interface::ControllerInterface()
{
}

controller_interface::CallbackReturn RightFingerMimicController::on_init()
{
  try {
    auto_declare<std::string>("main_joint", "hande_finger_distance");
    auto_declare<std::string>("finger_joint", "hande_right_finger_joint");
    auto_declare<double>("mimic_multiplier", -0.5);
    auto_declare<double>("mimic_offset", 0.025);
    auto_declare<double>("max_finger_position", 0.025);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception during init: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RightFingerMimicController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  main_joint_name_ = get_node()->get_parameter("main_joint").as_string();
  finger_joint_name_ = get_node()->get_parameter("finger_joint").as_string();
  mimic_multiplier_ = get_node()->get_parameter("mimic_multiplier").as_double();
  mimic_offset_ = get_node()->get_parameter("mimic_offset").as_double();
  max_finger_position_ = get_node()->get_parameter("max_finger_position").as_double();

  RCLCPP_DEBUG(get_node()->get_logger(), "RIGHT Finger Mimic: main='%s', finger='%s', mult=%.3f, offset=%.4f",
    main_joint_name_.c_str(), finger_joint_name_.c_str(), mimic_multiplier_, mimic_offset_);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration 
RightFingerMimicController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.push_back(finger_joint_name_ + "/" + hardware_interface::HW_IF_POSITION);
  return config;
}

controller_interface::InterfaceConfiguration 
RightFingerMimicController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.push_back(main_joint_name_ + "/" + hardware_interface::HW_IF_POSITION);
  return config;
}

controller_interface::CallbackReturn RightFingerMimicController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (command_interfaces_.size() != 1) {
    RCLCPP_ERROR(get_node()->get_logger(), "Expected 1 command interface, got %zu", command_interfaces_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  if (state_interfaces_.size() != 1) {
    RCLCPP_ERROR(get_node()->get_logger(), "Expected 1 state interface, got %zu", state_interfaces_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  double main_pos = state_interfaces_[0].get_value();
  double finger_pos = std::clamp(mimic_multiplier_ * main_pos + mimic_offset_, 0.0, max_finger_position_);
  command_interfaces_[0].set_value(finger_pos);

  RCLCPP_DEBUG(get_node()->get_logger(), "RIGHT Finger activated: main=%.5f -> finger=%.5f", main_pos, finger_pos);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RightFingerMimicController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type RightFingerMimicController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  double main_pos = state_interfaces_[0].get_value();
  double finger_pos = std::clamp(mimic_multiplier_ * main_pos + mimic_offset_, 0.0, max_finger_position_);
  command_interfaces_[0].set_value(finger_pos);

  RCLCPP_DEBUG_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
    "RIGHT: main=%.5f -> finger=%.5f", main_pos, finger_pos);

  return controller_interface::return_type::OK;
}

}  // namespace robotiq_controllers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(robotiq_controllers::RightFingerMimicController, controller_interface::ControllerInterface)