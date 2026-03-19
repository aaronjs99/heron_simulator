#include <string>

#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <geometry_msgs/Wrench.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <ros/subscribe_options.h>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>

namespace gazebo
{
class RelativeForcePlugin : public ModelPlugin
{
  public: RelativeForcePlugin() = default;

  public: ~RelativeForcePlugin() override
  {
    this->update_connection_.reset();
    this->queue_.clear();
    this->queue_.disable();
    if (this->rosnode_ != nullptr)
    {
      this->rosnode_->shutdown();
    }
    this->callback_queue_thread_.join();
    delete this->rosnode_;
    this->rosnode_ = nullptr;
  }

  public: void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    this->model_ = model;

    this->robot_namespace_ = "";
    if (sdf->HasElement("robotNamespace"))
    {
      this->robot_namespace_ =
          sdf->GetElement("robotNamespace")->Get<std::string>() + "/";
    }

    if (!sdf->HasElement("bodyName"))
    {
      ROS_FATAL_NAMED(
          "relative_force",
          "relative force plugin missing <bodyName>, cannot proceed");
      return;
    }
    this->link_name_ = sdf->GetElement("bodyName")->Get<std::string>();

    this->link_ = this->model_->GetLink(this->link_name_);
    if (!this->link_)
    {
      ROS_FATAL_NAMED(
          "relative_force",
          "relative force plugin error: link named %s does not exist",
          this->link_name_.c_str());
      return;
    }

    if (!sdf->HasElement("topicName"))
    {
      ROS_FATAL_NAMED(
          "relative_force",
          "relative force plugin missing <topicName>, cannot proceed");
      return;
    }
    this->topic_name_ = sdf->GetElement("topicName")->Get<std::string>();

    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM_NAMED(
          "relative_force",
          "A ROS node for Gazebo has not been initialized, unable to load plugin. "
          "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' first.");
      return;
    }

    this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);

    ros::SubscribeOptions so =
        ros::SubscribeOptions::create<geometry_msgs::Wrench>(
            this->topic_name_,
            1,
            boost::bind(&RelativeForcePlugin::UpdateObjectForce, this, _1),
            ros::VoidPtr(),
            &this->queue_);
    this->sub_ = this->rosnode_->subscribe(so);

    this->callback_queue_thread_ =
        boost::thread(boost::bind(&RelativeForcePlugin::QueueThread, this));

    this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&RelativeForcePlugin::UpdateChild, this));
  }

  private: void UpdateObjectForce(
      const geometry_msgs::Wrench::ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(this->lock_);
    this->wrench_msg_ = *msg;
  }

  private: void UpdateChild()
  {
    boost::mutex::scoped_lock lock(this->lock_);
    const ignition::math::Vector3d force(
        this->wrench_msg_.force.x,
        this->wrench_msg_.force.y,
        this->wrench_msg_.force.z);
    const ignition::math::Vector3d torque(
        this->wrench_msg_.torque.x,
        this->wrench_msg_.torque.y,
        this->wrench_msg_.torque.z);

    this->link_->AddRelativeForce(force);
    this->link_->AddRelativeTorque(torque);
  }

  private: void QueueThread()
  {
    static const double timeout = 0.01;
    while (this->rosnode_ != nullptr && this->rosnode_->ok())
    {
      this->queue_.callAvailable(ros::WallDuration(timeout));
    }
  }

  private: physics::ModelPtr model_;
  private: physics::LinkPtr link_;
  private: ros::NodeHandle* rosnode_ = nullptr;
  private: ros::Subscriber sub_;
  private: boost::mutex lock_;
  private: std::string topic_name_;
  private: std::string link_name_;
  private: std::string robot_namespace_;
  private: ros::CallbackQueue queue_;
  private: boost::thread callback_queue_thread_;
  private: geometry_msgs::Wrench wrench_msg_;
  private: event::ConnectionPtr update_connection_;
};

GZ_REGISTER_MODEL_PLUGIN(RelativeForcePlugin)
}  // namespace gazebo
