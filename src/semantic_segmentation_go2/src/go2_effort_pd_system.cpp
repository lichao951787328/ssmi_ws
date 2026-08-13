#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/components/JointForceCmd.hh>
#include <ignition/gazebo/components/JointPosition.hh>
#include <ignition/gazebo/components/JointVelocity.hh>
#include <ignition/msgs/double.pb.h>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>
#include <sdf/Element.hh>

namespace semantic_segmentation_go2
{
class Go2EffortPdSystem final
    : public ignition::gazebo::System,
      public ignition::gazebo::ISystemConfigure,
      public ignition::gazebo::ISystemPreUpdate
{
  struct JointControl
  {
    std::string name;
    ignition::gazebo::Entity entity{ignition::gazebo::kNullEntity};
    double target{0.0};
    double kp{40.0};
    double kd{1.0};
    double effortLimit{23.7};
  };

public:
  void Configure(const ignition::gazebo::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 ignition::gazebo::EntityComponentManager &_ecm,
                 ignition::gazebo::EventManager &) override
  {
    this->model_ = ignition::gazebo::Model(_entity);
    if (!this->model_.Valid(_ecm))
      return;

    auto jointElem = _sdf->FindElement("joint");
    while (jointElem)
    {
      JointControl joint;
      joint.name = jointElem->Get<std::string>("name");
      joint.target = jointElem->Get<double>("initial_position", joint.target).first;
      joint.kp = jointElem->Get<double>("kp", joint.kp).first;
      joint.kd = jointElem->Get<double>("kd", joint.kd).first;
      joint.effortLimit = jointElem->Get<double>("effort_limit", joint.effortLimit).first;
      joint.entity = this->model_.JointByName(_ecm, joint.name);
      if (joint.entity != ignition::gazebo::kNullEntity)
      {
        const std::size_t index = this->joints_.size();
        this->joints_.push_back(joint);
        const std::string topic = "/model/" + this->model_.Name(_ecm) +
            "/joint/" + joint.name + "/cmd_pos";
        std::function<void(const ignition::msgs::Double &)> callback =
            [this, index](const ignition::msgs::Double &_msg)
        {
          std::lock_guard<std::mutex> lock(this->mutex_);
          if (std::isfinite(_msg.data()))
            this->joints_[index].target = _msg.data();
        };
        this->node_.Subscribe<ignition::msgs::Double>(topic, callback);
      }
      jointElem = jointElem->GetNextElement("joint");
    }
  }

  void PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                 ignition::gazebo::EntityComponentManager &_ecm) override
  {
    if (_info.paused)
      return;

    std::lock_guard<std::mutex> lock(this->mutex_);
    for (const auto &joint : this->joints_)
    {
      const auto *position =
          _ecm.Component<ignition::gazebo::components::JointPosition>(joint.entity);
      const auto *velocity =
          _ecm.Component<ignition::gazebo::components::JointVelocity>(joint.entity);
      if (!position || !velocity || position->Data().empty() || velocity->Data().empty())
        continue;

      // Same convention as UnitreeJointController. The current RL policy
      // always sends dq_des=0 and tau_ff=0, so those terms are explicit here.
      const double q = position->Data()[0];
      const double dq = velocity->Data()[0];
      const double effort = std::clamp(
          joint.kp * (joint.target - q) + joint.kd * (0.0 - dq),
          -joint.effortLimit, joint.effortLimit);

      auto *command =
          _ecm.Component<ignition::gazebo::components::JointForceCmd>(joint.entity);
      if (!command)
      {
        _ecm.CreateComponent(joint.entity,
            ignition::gazebo::components::JointForceCmd({effort}));
      }
      else
      {
        if (command->Data().empty())
          command->Data().push_back(effort);
        else
          command->Data()[0] = effort;
      }
    }
  }

private:
  ignition::gazebo::Model model_{ignition::gazebo::kNullEntity};
  ignition::transport::Node node_;
  std::vector<JointControl> joints_;
  std::mutex mutex_;
};
}  // namespace semantic_segmentation_go2

IGNITION_ADD_PLUGIN(
    semantic_segmentation_go2::Go2EffortPdSystem,
    ignition::gazebo::System,
    ignition::gazebo::ISystemConfigure,
    ignition::gazebo::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(
    semantic_segmentation_go2::Go2EffortPdSystem,
    "semantic_segmentation_go2::Go2EffortPdSystem")
