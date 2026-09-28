#include <cmath>
#include <memory>
#include <string>

#include <gz/common/Console.hh>
#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <sdf/Element.hh>

namespace grande::simulation {

class PassiveDisturbancePlugin final : public gz::sim::System,
                                       public gz::sim::ISystemConfigure,
                                       public gz::sim::ISystemPreUpdate {
 public:
  void Configure(const gz::sim::Entity& entity,
                 const std::shared_ptr<const sdf::Element>& sdf,
                 gz::sim::EntityComponentManager& ecm,
                 gz::sim::EventManager&) override {
    this->model_ = gz::sim::Model(entity);
    const std::string body_name = ReadString(sdf, "bodyName", "base_link");
    const auto link_entity = this->model_.LinkByName(ecm, body_name);
    if (link_entity == gz::sim::kNullEntity) {
      gzerr << "passive disturbance could not find link " << body_name << '\n';
      return;
    }
    this->link_ = gz::sim::Link(link_entity);

    this->force_mean_ = ReadVector3(sdf, "forceMean");
    this->force_amplitude_ = ReadVector3(sdf, "forceAmplitude");
    this->torque_mean_ = ReadVector3(sdf, "torqueMean");
    this->torque_amplitude_ = ReadVector3(sdf, "torqueAmplitude");
    this->period_sec_ = ReadDouble(sdf, "periodSec", 12.0);
    this->phase_rad_ = ReadDouble(sdf, "phaseRad", 0.0);
    this->configured_ = true;
  }

  void PreUpdate(const gz::sim::UpdateInfo& info,
                 gz::sim::EntityComponentManager& ecm) override {
    if (!this->configured_ || info.paused || !this->link_.Valid(ecm)) {
      return;
    }

    const double time_sec =
        std::chrono::duration<double>(info.simTime).count();
    double wave = 0.0;
    if (this->period_sec_ > 1e-6) {
      constexpr double kPi = 3.14159265358979323846;
      wave = std::sin((2.0 * kPi * time_sec / this->period_sec_) +
                      this->phase_rad_);
    }

    const auto pose = this->link_.WorldPose(ecm);
    if (!pose) {
      return;
    }
    const auto rotation = pose->Rot();
    const auto force_world = rotation.RotateVector(
        this->force_mean_ + (this->force_amplitude_ * wave));
    const auto torque_world = rotation.RotateVector(
        this->torque_mean_ + (this->torque_amplitude_ * wave));
    this->link_.AddWorldWrench(ecm, force_world, torque_world);
  }

 private:
  static std::string ReadString(
      const std::shared_ptr<const sdf::Element>& sdf,
      const std::string& name, const std::string& fallback) {
    return sdf && sdf->HasElement(name) ? sdf->Get<std::string>(name)
                                        : fallback;
  }

  static double ReadDouble(const std::shared_ptr<const sdf::Element>& sdf,
                           const std::string& name, double fallback) {
    return sdf && sdf->HasElement(name) ? sdf->Get<double>(name) : fallback;
  }

  static gz::math::Vector3d ReadVector3(
      const std::shared_ptr<const sdf::Element>& sdf,
      const std::string& name) {
    return sdf && sdf->HasElement(name)
               ? sdf->Get<gz::math::Vector3d>(name)
               : gz::math::Vector3d::Zero;
  }

  gz::sim::Model model_;
  gz::sim::Link link_;
  gz::math::Vector3d force_mean_;
  gz::math::Vector3d force_amplitude_;
  gz::math::Vector3d torque_mean_;
  gz::math::Vector3d torque_amplitude_;
  double period_sec_{12.0};
  double phase_rad_{0.0};
  bool configured_{false};
};

}  // namespace grande::simulation

GZ_ADD_PLUGIN(grande::simulation::PassiveDisturbancePlugin,
              gz::sim::System,
              gz::sim::ISystemConfigure,
              gz::sim::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(grande::simulation::PassiveDisturbancePlugin,
                    "grande::simulation::PassiveDisturbancePlugin")
