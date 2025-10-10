#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/gazebo/components/Joint.hh>
#include <ignition/gazebo/components/JointPosition.hh>
#include <ignition/gazebo/components/Name.hh>
#include <rclcpp/rclcpp.hpp>

namespace ignition
{
namespace gazebo
{
namespace systems
{
  class MimicJointPlugin :
    public System,
    public ISystemConfigure,
    public ISystemPreUpdate
  {
  public:
    MimicJointPlugin() : multiplier_(1.0), offset_(0.0) {}

    void Configure(const Entity &_entity,
                   const std::shared_ptr<const sdf::Element> &_sdf,
                   EntityComponentManager &_ecm,
                   EventManager &/*_eventMgr*/) override
    {
      model_ = Model(_entity);

      if (!_sdf->HasElement("main_joint") || !_sdf->HasElement("mimic_joint"))
      {
        ignerr << "[MimicJointPlugin] Missing <main_joint> or <mimic_joint>\n";
        return;
      }

      main_joint_name_ = _sdf->Get<std::string>("main_joint");
      mimic_joint_name_ = _sdf->Get<std::string>("mimic_joint");

      if (_sdf->HasElement("multiplier"))
        multiplier_ = _sdf->Get<double>("multiplier");
      if (_sdf->HasElement("offset"))
        offset_ = _sdf->Get<double>("offset");

      main_joint_ = model_.JointByName(_ecm, main_joint_name_);
      mimic_joint_ = model_.JointByName(_ecm, mimic_joint_name_);

      if (main_joint_ == kNullEntity || mimic_joint_ == kNullEntity)
      {
        ignerr << "[MimicJointPlugin] Could not find joints: "
               << main_joint_name_ << " or " << mimic_joint_name_ << "\n";
        return;
      }

      ignmsg << "[MimicJointPlugin] Loaded: " << main_joint_name_ << " → "
             << mimic_joint_name_ << " mult=" << multiplier_
             << " offset=" << offset_ << "\n";
    }

    void PreUpdate(const UpdateInfo &/*_info*/,
                   EntityComponentManager &_ecm) override
    {
      if (main_joint_ == kNullEntity || mimic_joint_ == kNullEntity)
        return;

      // Get main joint position
      auto main_pos_comp = _ecm.Component<components::JointPosition>(main_joint_);
      if (!main_pos_comp || main_pos_comp->Data().empty())
        return;

      double main_pos = main_pos_comp->Data()[0];
      double mimic_pos = multiplier_ * main_pos + offset_;

      // Set mimic joint position
      auto mimic_pos_comp = _ecm.Component<components::JointPosition>(mimic_joint_);
      if (mimic_pos_comp)
      {
        *mimic_pos_comp = components::JointPosition({mimic_pos});
      }
    }

  private:
    Model model_{kNullEntity};
    Entity main_joint_{kNullEntity};
    Entity mimic_joint_{kNullEntity};
    std::string main_joint_name_;
    std::string mimic_joint_name_;
    double multiplier_;
    double offset_;
  };
}
}
}

// Register the plugin with an alias
IGNITION_ADD_PLUGIN(
  ignition::gazebo::systems::MimicJointPlugin,
  ignition::gazebo::System,
  ignition::gazebo::systems::MimicJointPlugin::ISystemConfigure,
  ignition::gazebo::systems::MimicJointPlugin::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(
  ignition::gazebo::systems::MimicJointPlugin,
  "MimicJointPlugin")