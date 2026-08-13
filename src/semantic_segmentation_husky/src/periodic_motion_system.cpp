#include <chrono>
#include <cmath>
#include <memory>

#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/plugin/Register.hh>
#include <sdf/Element.hh>

namespace semantic_segmentation_husky
{
class PeriodicMotionSystem final
    : public ignition::gazebo::System,
      public ignition::gazebo::ISystemConfigure,
      public ignition::gazebo::ISystemPreUpdate
{
public:
  void Configure(const ignition::gazebo::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 ignition::gazebo::EntityComponentManager &_ecm,
                 ignition::gazebo::EventManager &) override
  {
    this->model_ = ignition::gazebo::Model(_entity);

    const auto *pose = _ecm.Component<ignition::gazebo::components::Pose>(_entity);
    if (!this->model_.Valid(_ecm) || pose == nullptr)
    {
      this->valid_ = false;
      return;
    }

    this->origin_ = pose->Data();
    if (_sdf->HasElement("offset"))
      this->offset_ = _sdf->Get<ignition::math::Vector3d>("offset");
    if (_sdf->HasElement("period"))
      this->period_ = std::max(0.1, _sdf->Get<double>("period"));
    if (_sdf->HasElement("phase"))
      this->phase_ = _sdf->Get<double>("phase");
  }

  void PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                 ignition::gazebo::EntityComponentManager &_ecm) override
  {
    if (!this->valid_ || _info.paused)
      return;

    const double sim_time =
        std::chrono::duration<double>(_info.simTime).count() + this->phase_;
    const double angle = 2.0 * M_PI * sim_time / this->period_;
    const double progress = 0.5 * (1.0 - std::cos(angle));

    ignition::math::Pose3d command = this->origin_;
    command.Pos() += this->offset_ * progress;
    this->model_.SetWorldPoseCmd(_ecm, command);
  }

private:
  ignition::gazebo::Model model_{ignition::gazebo::kNullEntity};
  ignition::math::Pose3d origin_;
  ignition::math::Vector3d offset_{0.0, 0.0, 0.0};
  double period_{20.0};
  double phase_{0.0};
  bool valid_{true};
};
}  // namespace semantic_segmentation_husky

IGNITION_ADD_PLUGIN(
    semantic_segmentation_husky::PeriodicMotionSystem,
    ignition::gazebo::System,
    ignition::gazebo::ISystemConfigure,
    ignition::gazebo::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(
    semantic_segmentation_husky::PeriodicMotionSystem,
    "semantic_segmentation_husky::PeriodicMotionSystem")
