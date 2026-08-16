#include <algorithm>
#include <cmath>
#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include <string>

namespace gazebo {
class SurfaceHydrodynamicsPlugin : public ModelPlugin {
public:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override {
    this->model_ = model;
    this->link_name_ = this->ReadString(sdf, "bodyName", "base_footprint");
    this->link_ = this->model_->GetLink(this->link_name_);
    if (!this->link_) {
      gzerr << "surface hydrodynamics could not find link " << this->link_name_ << "\n";
      return;
    }

    this->equilibrium_z_ = this->ReadDouble(sdf, "equilibriumZ", 0.0);
    this->heave_stiffness_ = this->ReadDouble(sdf, "heaveStiffness", 900.0);
    this->heave_damping_ = this->ReadDouble(sdf, "heaveDamping", 180.0);
    this->roll_stiffness_ = this->ReadDouble(sdf, "rollStiffness", 35.0);
    this->pitch_stiffness_ = this->ReadDouble(sdf, "pitchStiffness", 35.0);
    this->linear_damping_ = this->ReadVector3(sdf, "linearDamping", ignition::math::Vector3d(25.0, 24.0, 150.0));
    this->quadratic_damping_ = this->ReadVector3(sdf, "quadraticDamping", ignition::math::Vector3d(5.0, 5.0, 15.0));
    this->angular_damping_ = this->ReadVector3(sdf, "angularDamping", ignition::math::Vector3d(20.0, 20.0, 10.0));
    this->angular_quadratic_damping_ =
        this->ReadVector3(sdf, "angularQuadraticDamping", ignition::math::Vector3d(8.0, 8.0, 8.0));

    this->mass_kg_ = 0.0;
    for (const auto& link : this->model_->GetLinks()) {
      if (link && link->GetInertial()) {
        this->mass_kg_ += link->GetInertial()->Mass();
      }
    }
    if (this->mass_kg_ <= 0.0) {
      gzerr << "surface hydrodynamics requires positive model mass\n";
      return;
    }

    this->update_connection_ =
        event::Events::ConnectWorldUpdateBegin([this](const common::UpdateInfo&) { this->Update(); });
  }

private:
  static double Drag(double velocity, double linear, double quadratic) {
    return -(linear * velocity + quadratic * std::abs(velocity) * velocity);
  }

private:
  void Update() {
    if (!this->link_) {
      return;
    }
    const auto pose = this->link_->WorldPose();
    const auto velocity_world = this->link_->WorldLinearVel();
    const auto velocity_body = this->link_->RelativeLinearVel();
    const auto angular_body = this->link_->RelativeAngularVel();
    const auto gravity = this->model_->GetWorld()->Gravity();

    const double weight_n = std::max(0.0, -this->mass_kg_ * gravity.Z());
    const double hydrostatic_n = std::max(
        0.0, std::min(2.0 * weight_n, weight_n + this->heave_stiffness_ * (this->equilibrium_z_ - pose.Pos().Z()) -
                                          this->heave_damping_ * velocity_world.Z()));
    this->link_->AddForce(ignition::math::Vector3d(0.0, 0.0, hydrostatic_n));

    this->link_->AddRelativeForce(ignition::math::Vector3d(
        this->Drag(velocity_body.X(), this->linear_damping_.X(), this->quadratic_damping_.X()),
        this->Drag(velocity_body.Y(), this->linear_damping_.Y(), this->quadratic_damping_.Y()), 0.0));

    const auto euler = pose.Rot().Euler();
    this->link_->AddRelativeTorque(ignition::math::Vector3d(
        -this->roll_stiffness_ * euler.X() +
            this->Drag(angular_body.X(), this->angular_damping_.X(), this->angular_quadratic_damping_.X()),
        -this->pitch_stiffness_ * euler.Y() +
            this->Drag(angular_body.Y(), this->angular_damping_.Y(), this->angular_quadratic_damping_.Y()),
        this->Drag(angular_body.Z(), this->angular_damping_.Z(), this->angular_quadratic_damping_.Z())));
  }

private:
  std::string ReadString(const sdf::ElementPtr& sdf, const std::string& name, const std::string& fallback) const {
    return sdf->HasElement(name) ? sdf->Get<std::string>(name) : fallback;
  }

private:
  double ReadDouble(const sdf::ElementPtr& sdf, const std::string& name, double fallback) const {
    return sdf->HasElement(name) ? sdf->Get<double>(name) : fallback;
  }

private:
  ignition::math::Vector3d ReadVector3(const sdf::ElementPtr& sdf, const std::string& name,
                                       const ignition::math::Vector3d& fallback) const {
    return sdf->HasElement(name) ? sdf->Get<ignition::math::Vector3d>(name) : fallback;
  }

private:
  physics::ModelPtr model_;

private:
  physics::LinkPtr link_;

private:
  std::string link_name_;

private:
  double mass_kg_ = 0.0;

private:
  double equilibrium_z_ = 0.0;

private:
  double heave_stiffness_ = 900.0;

private:
  double heave_damping_ = 180.0;

private:
  double roll_stiffness_ = 35.0;

private:
  double pitch_stiffness_ = 35.0;

private:
  ignition::math::Vector3d linear_damping_;

private:
  ignition::math::Vector3d quadratic_damping_;

private:
  ignition::math::Vector3d angular_damping_;

private:
  ignition::math::Vector3d angular_quadratic_damping_;

private:
  event::ConnectionPtr update_connection_;
};

GZ_REGISTER_MODEL_PLUGIN(SurfaceHydrodynamicsPlugin)
}  // namespace gazebo
