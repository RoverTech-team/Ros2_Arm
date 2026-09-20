/*
 * Gazebo Harmonic plugin for a rotary switch joint.
 * Applies a constant friction torque and publishes the
 * current joint angle to a ROS 2 topic.
 */

#include <gz/sim/System.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Model.hh>
#include <gz/plugin/Register.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <string>

namespace gz_sim
{

class RotarySwitchPlugin :
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate,
  public gz::sim::ISystemUpdate,
  public gz::sim::ISystemPostUpdate
{
public:
  RotarySwitchPlugin() = default;
  ~RotarySwitchPlugin() override = default;

  void Configure(const gz::sim::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &_ecm,
                 gz::sim::EventManager &_eventMgr) override
  {
    jointName_ = _sdf->Get<std::string>("joint");
    frictionTorque_ = _sdf->Get<double>("friction_torque");

    model_ = gz::sim::Model(_entity);
    joint_ = gz::sim::Joint(model_.JointByName(_ecm, jointName_));
    joint_.EnablePositionCheck(_ecm);
    joint_.EnableVelocityCheck(_ecm);

    if (!rclcpp::ok()) {
      int argc = 0;
      char const * const * argv = nullptr;
      rclcpp::init(argc, argv);
    }
    rosNode_ = std::make_shared<rclcpp::Node>("rotary_switch_" + jointName_);
    anglePub_ = rosNode_->create_publisher<std_msgs::msg::Float64>(
        "/panel/switch/" + jointName_ + "/angle", 10);

    gzmsg << "RotarySwitchPlugin loaded: " << jointName_
          << " friction=" << frictionTorque_
          << std::endl;
  }

  void PreUpdate(const gz::sim::UpdateInfo &_info,
                 gz::sim::EntityComponentManager &_ecm) override
  {
    if (!joint_.Valid(_ecm))
      return;

    auto velOpt = joint_.Velocity(_ecm);
    if (!velOpt)
      return;
    double vel = (*velOpt)[0];
    double force = (vel > 0.0 ? -1.0 : 1.0) * frictionTorque_;

    joint_.SetForce(_ecm, std::vector<double>{force});

    auto posOpt = joint_.Position(_ecm);
    if (!posOpt)
      return;
    double pos = (*posOpt)[0];
    auto msg = std_msgs::msg::Float64();
    msg.data = pos;
    anglePub_->publish(msg);
  }

  void Update(const gz::sim::UpdateInfo &_info,
              gz::sim::EntityComponentManager &_ecm) override
  {
  }

  void PostUpdate(const gz::sim::UpdateInfo &_info,
                  const gz::sim::EntityComponentManager &_ecm) override
  {
  }

private:
  std::string jointName_;
  double frictionTorque_ = 0.1;
  gz::sim::Model model_;
  gz::sim::Joint joint_;

  std::shared_ptr<rclcpp::Node> rosNode_;
  std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64>> anglePub_;
};

GZ_ADD_PLUGIN(
  RotarySwitchPlugin,
  ::gz::sim::System,
  RotarySwitchPlugin::ISystemConfigure,
  RotarySwitchPlugin::ISystemPreUpdate,
  RotarySwitchPlugin::ISystemUpdate,
  RotarySwitchPlugin::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(RotarySwitchPlugin, "gz_sim::RotarySwitchPlugin")

}
