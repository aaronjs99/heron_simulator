#include <algorithm>
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
#include <gz/sim/World.hh>
#include <gz/sim/components/Inertial.hh>
#include <gz/sim/components/World.hh>
#include <sdf/Element.hh>

namespace grande::simulation {

class SurfaceHydrodynamicsPlugin final : public gz::sim::System,
                                         public gz::sim::ISystemConfigure,
                                         public gz::sim::ISystemPreUpdate {
 public:
  void Configure(const gz::sim::Entity& entity,
                 const std::shared_ptr<const sdf::Element>& sdf,
                 gz::sim::EntityComponentManager& ecm,
                 gz::sim::EventManager&) override {
    this->model_ = gz::sim::Model(entity);
    this->link_name_ = ReadString(sdf, "bodyName", "base_footprint");
    const auto link_entity = this->model_.LinkByName(ecm, this->link_name_);
    if (link_entity == gz::sim::kNullEntity) {
      gzerr << "surface hydrodynamics could not find link "
            << this->link_name_ << '\n';
      return;
    }
    this->link_ = gz::sim::Link(link_entity);
    this->link_.EnableVelocityChecks(ecm);

    this->equilibrium_z_ = ReadDouble(sdf, "equilibriumZ", 0.0);
    this->heave_stiffness_ = ReadDouble(sdf, "heaveStiffness", 900.0);
    this->heave_damping_ = ReadDouble(sdf, "heaveDamping", 180.0);
    this->roll_stiffness_ = ReadDouble(sdf, "rollStiffness", 35.0);
    this->pitch_stiffness_ = ReadDouble(sdf, "pitchStiffness", 35.0);
    this->linear_damping_ = ReadVector3(
        sdf, "linearDamping", gz::math::Vector3d(25.0, 24.0, 150.0));
    this->quadratic_damping_ = ReadVector3(
        sdf, "quadraticDamping", gz::math::Vector3d(5.0, 5.0, 15.0));
    this->angular_damping_ = ReadVector3(
        sdf, "angularDamping", gz::math::Vector3d(20.0, 20.0, 10.0));
    this->angular_quadratic_damping_ = ReadVector3(
        sdf, "angularQuadraticDamping", gz::math::Vector3d(8.0, 8.0, 8.0));

    for (const auto link_entity_id : this->model_.Links(ecm)) {
      const auto* inertial =
          ecm.Component<gz::sim::components::Inertial>(link_entity_id);
      if (inertial) {
        this->mass_kg_ += inertial->Data().MassMatrix().Mass();
      }
    }
    if (this->mass_kg_ <= 0.0) {
      gzerr << "surface hydrodynamics requires positive model mass\n";
      return;
    }

    this->configured_ = true;
  }

  void PreUpdate(const gz::sim::UpdateInfo& info,
                 gz::sim::EntityComponentManager& ecm) override {
    if (!this->configured_ || info.paused || !this->link_.Valid(ecm)) {
      return;
    }

    if (!this->world_.Valid(ecm)) {
      ecm.Each<gz::sim::components::World>(
          [this](const gz::sim::Entity& entity,
                 const gz::sim::components::World*) {
            this->world_ = gz::sim::World(entity);
            return false;
          });
    }
    if (!this->world_.Valid(ecm)) {
      return;
    }

    const auto pose = this->link_.WorldPose(ecm);
    const auto velocity_world = this->link_.WorldLinearVelocity(ecm);
    const auto angular_world = this->link_.WorldAngularVelocity(ecm);
    const auto gravity = this->world_.Gravity(ecm);
    if (!pose || !velocity_world || !angular_world || !gravity) {
      return;
    }

    const auto rotation = pose->Rot();
    const auto velocity_body = rotation.RotateVectorReverse(*velocity_world);
    const auto angular_body = rotation.RotateVectorReverse(*angular_world);
    const double weight_n = std::max(0.0, -this->mass_kg_ * gravity->Z());
    const double hydrostatic_n = std::max(
        0.0,
        std::min(2.0 * weight_n,
                 weight_n + this->heave_stiffness_ *
                                (this->equilibrium_z_ - pose->Pos().Z()) -
                     this->heave_damping_ * velocity_world->Z()));
    this->link_.AddWorldForce(ecm, gz::math::Vector3d(0.0, 0.0, hydrostatic_n));

    const gz::math::Vector3d drag_body(
        Drag(velocity_body.X(), this->linear_damping_.X(),
             this->quadratic_damping_.X()),
        Drag(velocity_body.Y(), this->linear_damping_.Y(),
             this->quadratic_damping_.Y()),
        0.0);
    const auto euler = rotation.Euler();
    const gz::math::Vector3d torque_body(
        -this->roll_stiffness_ * euler.X() +
            Drag(angular_body.X(), this->angular_damping_.X(),
                 this->angular_quadratic_damping_.X()),
        -this->pitch_stiffness_ * euler.Y() +
            Drag(angular_body.Y(), this->angular_damping_.Y(),
                 this->angular_quadratic_damping_.Y()),
        Drag(angular_body.Z(), this->angular_damping_.Z(),
             this->angular_quadratic_damping_.Z()));
    this->link_.AddWorldWrench(ecm, rotation.RotateVector(drag_body),
                               rotation.RotateVector(torque_body));
  }

 private:
  static double Drag(double velocity, double linear, double quadratic) {
    return -(linear * velocity + quadratic * std::abs(velocity) * velocity);
  }

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
      const std::string& name, const gz::math::Vector3d& fallback) {
    return sdf && sdf->HasElement(name)
               ? sdf->Get<gz::math::Vector3d>(name)
               : fallback;
  }

  gz::sim::Model model_;
  gz::sim::Link link_;
  gz::sim::World world_;
  std::string link_name_;
  double mass_kg_{0.0};
  double equilibrium_z_{0.0};
  double heave_stiffness_{900.0};
  double heave_damping_{180.0};
  double roll_stiffness_{35.0};
  double pitch_stiffness_{35.0};
  gz::math::Vector3d linear_damping_;
  gz::math::Vector3d quadratic_damping_;
  gz::math::Vector3d angular_damping_;
  gz::math::Vector3d angular_quadratic_damping_;
  bool configured_{false};
};

}  // namespace grande::simulation

GZ_ADD_PLUGIN(grande::simulation::SurfaceHydrodynamicsPlugin,
              gz::sim::System,
              gz::sim::ISystemConfigure,
              gz::sim::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(grande::simulation::SurfaceHydrodynamicsPlugin,
                    "grande::simulation::SurfaceHydrodynamicsPlugin")
