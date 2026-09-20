/*
 * Gazebo Harmonic plugin for a lever switch joint.
 * Applies a detent force holding the joint at off/on positions
 * and publishes the current joint position to a ROS 2 topic.
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

class LeverSwitchPlugin :
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate,
  public gz::sim::ISystemUpdate,
  public gz::sim::ISystemPostUpdate
{
public:
  LeverSwitchPlugin() = default;
  ~LeverSwitchPlugin() override = default;

  void Configure(const gz::sim::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &_ecm,
                 gz::sim::EventManager &_eventMgr) override
  {
    jointName_ = _sdf->Get<std::string>("joint");
    offPosition_ = _sdf->Get<double>("off_position");
    onPosition_ = _sdf->Get<double>("on_position");
    detentStiffness_ = _sdf->Get<double>("detent_stiffness");

    model_ = gz::sim::Model(_entity);
    joint_ = gz::sim::Joint(model_.JointByName(_ecm, jointName_));
    joint_.EnablePositionCheck(_ecm);

    if (!rclcpp::ok()) {
      int argc = 0;
      char const * const * argv = nullptr;
      rclcpp::init(argc, argv);
    }
    rosNode_ = std::make_shared<rclcpp::Node>("lever_switch_" + jointName_);
    positionPub_ = rosNode_->create_publisher<std_msgs::msg::Float64>(
        "/panel/switch/" + jointName_ + "/position", 10);

    gzmsg << "LeverSwitchPlugin loaded: " << jointName_
          << " OFF=" << offPosition_ << " ON=" << onPosition_
          << " stiffness=" << detentStiffness_ << std::endl;
  }

  void PreUpdate(const gz::sim::UpdateInfo &_info,
                 gz::sim::EntityComponentManager &_ecm) override
  {
    if (!joint_.Valid(_ecm))
      return;

    auto posOpt = joint_.Position(_ecm);
    if (!posOpt)
      return;
    double pos = (*posOpt)[0];

    double target = (pos < (offPosition_ + onPosition_) / 2.0)
                      ? offPosition_ : onPosition_;
    double force = detentStiffness_ * (target - pos);

    joint_.SetForce(_ecm, std::vector<double>{force});

    auto msg = std_msgs::msg::Float64();
    msg.data = pos;
    positionPub_->publish(msg);
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
  double offPosition_ = 0.0;
  double onPosition_ = 1.0;
  double detentStiffness_ = 100.0;
  gz::sim::Model model_;
  gz::sim::Joint joint_;

  std::shared_ptr<rclcpp::Node> rosNode_;
  std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64>> positionPub_;
};

GZ_ADD_PLUGIN(
  LeverSwitchPlugin,
  ::gz::sim::System,
  LeverSwitchPlugin::ISystemConfigure,
  LeverSwitchPlugin::ISystemPreUpdate,
  LeverSwitchPlugin::ISystemUpdate,
  LeverSwitchPlugin::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(LeverSwitchPlugin, "gz_sim::LeverSwitchPlugin")

}
