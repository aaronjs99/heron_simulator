#include <chrono>
#include <memory>
#include <mutex>
#include <string>

#include <gz/common/Console.hh>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/wrench.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>

namespace grande::simulation {

class RelativeForcePlugin final : public gz::sim::System,
                                  public gz::sim::ISystemConfigure,
                                  public gz::sim::ISystemPreUpdate {
 public:
  void Configure(const gz::sim::Entity& entity,
                 const std::shared_ptr<const sdf::Element>& sdf,
                 gz::sim::EntityComponentManager& ecm,
                 gz::sim::EventManager&) override {
    this->model_ = gz::sim::Model(entity);
    const std::string body_name = ReadString(sdf, "bodyName", "");
    if (body_name.empty()) {
      gzerr << "relative force plugin is missing <bodyName>\n";
      return;
    }
    const auto link_entity = this->model_.LinkByName(ecm, body_name);
    if (link_entity == gz::sim::kNullEntity) {
      gzerr << "relative force plugin could not find link " << body_name
            << '\n';
      return;
    }
    this->link_ = gz::sim::Link(link_entity);

    this->topic_name_ = ReadString(sdf, "topicName", "");
    if (this->topic_name_.empty() || this->topic_name_.front() != '/') {
      gzerr << "relative force plugin requires an absolute <topicName>\n";
      return;
    }
    this->command_timeout_sec_ = ReadDouble(sdf, "commandTimeout", 0.5);
    if (this->command_timeout_sec_ <= 0.0) {
      gzerr << "relative force plugin requires a positive <commandTimeout>\n";
      return;
    }

    this->watchdog_pub_ = this->node_.Advertise<gz::msgs::Boolean>(
        this->topic_name_ + "/watchdog_active");
    if (!this->watchdog_pub_) {
      gzerr << "relative force plugin could not advertise watchdog topic "
            << this->topic_name_ << "/watchdog_active\n";
      return;
    }
    if (!this->node_.Subscribe(this->topic_name_,
                               &RelativeForcePlugin::OnWrench, this)) {
      gzerr << "relative force plugin could not subscribe to "
            << this->topic_name_ << '\n';
      return;
    }
    this->configured_ = true;
  }

  void PreUpdate(const gz::sim::UpdateInfo& info,
                 gz::sim::EntityComponentManager& ecm) override {
    if (!this->configured_ || info.paused || !this->link_.Valid(ecm)) {
      return;
    }

    gz::msgs::Wrench command;
    bool received_command = false;
    std::chrono::steady_clock::time_point last_command_time;
    {
      std::lock_guard<std::mutex> lock(this->mutex_);
      command = this->wrench_;
      received_command = this->received_command_;
      last_command_time = this->last_command_time_;
    }

    const auto now = std::chrono::steady_clock::now();
    const bool watchdog_active =
        !received_command ||
        std::chrono::duration<double>(now - last_command_time).count() >
            this->command_timeout_sec_;
    if (watchdog_active) {
      command.Clear();
    }

    if (!this->watchdog_state_published_ ||
        watchdog_active != this->watchdog_active_ ||
        now - this->last_status_publish_ >= std::chrono::seconds(1)) {
      gz::msgs::Boolean status;
      status.set_data(watchdog_active);
      this->watchdog_pub_.Publish(status);
      this->watchdog_active_ = watchdog_active;
      this->watchdog_state_published_ = true;
      this->last_status_publish_ = now;
    }

    const auto pose = this->link_.WorldPose(ecm);
    if (!pose) {
      return;
    }
    const gz::math::Vector3d force_body(command.force().x(),
                                        command.force().y(),
                                        command.force().z());
    const gz::math::Vector3d torque_body(command.torque().x(),
                                         command.torque().y(),
                                         command.torque().z());
    const auto rotation = pose->Rot();
    this->link_.AddWorldWrench(ecm, rotation.RotateVector(force_body),
                               rotation.RotateVector(torque_body));
  }

 private:
  void OnWrench(const gz::msgs::Wrench& message) {
    std::lock_guard<std::mutex> lock(this->mutex_);
    this->wrench_ = message;
    this->last_command_time_ = std::chrono::steady_clock::now();
    this->received_command_ = true;
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

  gz::sim::Model model_;
  gz::sim::Link link_;
  gz::transport::Node node_;
  gz::transport::Node::Publisher watchdog_pub_;
  std::string topic_name_;
  std::mutex mutex_;
  gz::msgs::Wrench wrench_;
  std::chrono::steady_clock::time_point last_command_time_{};
  std::chrono::steady_clock::time_point last_status_publish_{};
  double command_timeout_sec_{0.5};
  bool received_command_{false};
  bool watchdog_active_{true};
  bool watchdog_state_published_{false};
  bool configured_{false};
};

}  // namespace grande::simulation

GZ_ADD_PLUGIN(grande::simulation::RelativeForcePlugin,
              gz::sim::System,
              gz::sim::ISystemConfigure,
              gz::sim::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(grande::simulation::RelativeForcePlugin,
                    "grande::simulation::RelativeForcePlugin")
