#include <cmath>
#include <string>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>

namespace gazebo
{
class PassiveDisturbancePlugin : public ModelPlugin
{
  public: PassiveDisturbancePlugin() = default;

  public: void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    this->model_ = model;

    if (!sdf->HasElement("bodyName"))
    {
      gzerr << "passive disturbance plugin missing <bodyName>\n";
      return;
    }
    const std::string body_name =
        sdf->GetElement("bodyName")->Get<std::string>();
    this->link_ = this->model_->GetLink(body_name);
    if (!this->link_)
    {
      gzerr << "passive disturbance plugin could not find link "
            << body_name << "\n";
      return;
    }

    this->force_mean_ = this->ReadVector3(sdf, "forceMean");
    this->force_amplitude_ = this->ReadVector3(sdf, "forceAmplitude");
    this->torque_mean_ = this->ReadVector3(sdf, "torqueMean");
    this->torque_amplitude_ = this->ReadVector3(sdf, "torqueAmplitude");
    this->period_sec_ = this->ReadDouble(sdf, "periodSec", 12.0);
    this->phase_rad_ = this->ReadDouble(sdf, "phaseRad", 0.0);

    this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
        [this](const common::UpdateInfo& info) { this->Update(info); });
  }

  private: ignition::math::Vector3d ReadVector3(
      const sdf::ElementPtr& sdf,
      const std::string& name) const
  {
    if (!sdf->HasElement(name))
    {
      return ignition::math::Vector3d::Zero;
    }
    return sdf->Get<ignition::math::Vector3d>(name);
  }

  private: double ReadDouble(
      const sdf::ElementPtr& sdf,
      const std::string& name,
      double fallback) const
  {
    if (!sdf->HasElement(name))
    {
      return fallback;
    }
    return sdf->Get<double>(name);
  }

  private: void Update(const common::UpdateInfo& info)
  {
    if (!this->link_)
    {
      return;
    }

    const double t = info.simTime.Double();
    double wave = 0.0;
    if (this->period_sec_ > 1e-6)
    {
      static const double kPi = 3.14159265358979323846;
      wave = std::sin((2.0 * kPi * t / this->period_sec_) + this->phase_rad_);
    }

    this->link_->AddRelativeForce(
        this->force_mean_ + (this->force_amplitude_ * wave));
    this->link_->AddRelativeTorque(
        this->torque_mean_ + (this->torque_amplitude_ * wave));
  }

  private: physics::ModelPtr model_;
  private: physics::LinkPtr link_;
  private: ignition::math::Vector3d force_mean_;
  private: ignition::math::Vector3d force_amplitude_;
  private: ignition::math::Vector3d torque_mean_;
  private: ignition::math::Vector3d torque_amplitude_;
  private: double period_sec_ = 12.0;
  private: double phase_rad_ = 0.0;
  private: event::ConnectionPtr update_connection_;
};

GZ_REGISTER_MODEL_PLUGIN(PassiveDisturbancePlugin)
}  // namespace gazebo
