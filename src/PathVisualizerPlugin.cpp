#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreVector3.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/rendering/Scene.hh>
#include <gazebo/rendering/Visual.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <mutex>
#include <string>
#include <vector>

namespace gazebo {
class PathVisualizerPlugin : public VisualPlugin {
public:
  PathVisualizerPlugin()
      : visual_(nullptr),
        nh_(nullptr),
        scene_(nullptr),
        local_path_object_(nullptr),
        global_path_object_(nullptr),
        scene_node_(nullptr),
        robot_visual_(nullptr),
        new_path_received_(false),
        new_global_path_received_(false),
        initialized_(false),
        warned_waiting_for_ros_(false),
        logged_first_local_msg_(false),
        logged_first_global_msg_(false),
        logged_first_local_draw_(false),
        logged_first_global_draw_(false),
        local_frame_mode_known_(false),
        local_frame_mode_robot_local_(false),
        start_wall_time_(ros::WallTime::now()) {}

  virtual ~PathVisualizerPlugin() {
    if (nh_) {
      nh_->shutdown();
      delete nh_;
    }
  }

  void Load(rendering::VisualPtr _parent, sdf::ElementPtr /*_sdf*/) override {
    this->visual_ = _parent;
    this->update_connection_ =
        event::Events::ConnectPreRender(std::bind(&PathVisualizerPlugin::OnUpdate, this));
    this->TryInitialize();
  }

private:
  void TryInitialize() {
    if (this->initialized_ || !this->visual_) {
      return;
    }

    if (!ros::isInitialized()) {
      if (!this->warned_waiting_for_ros_) {
        ROS_DEBUG_STREAM(
            "[PathVisualizerPlugin] Gazebo ROS is not ready yet; delaying plugin initialization.");
        this->warned_waiting_for_ros_ = true;
      }
      return;
    }

    this->nh_ = new ros::NodeHandle("~");
    this->scene_ = this->visual_->GetScene();
    if (!this->scene_) {
      return;
    }

    Ogre::SceneManager* scene_manager = this->scene_->OgreSceneManager();
    if (!scene_manager) {
      return;
    }

    static int instance_id = 0;
    const std::string suffix = "_" + std::to_string(instance_id++);

    this->scene_node_ = this->visual_->GetSceneNode()->createChildSceneNode();

    this->local_path_object_ = scene_manager->createManualObject("LocalPathBand" + suffix);
    this->local_path_object_->setDynamic(true);
    this->local_path_object_->setBoundingBox(Ogre::AxisAlignedBox::BOX_INFINITE);
    this->scene_node_->attachObject(this->local_path_object_);

    this->global_path_object_ = scene_manager->createManualObject("GlobalPathLine" + suffix);
    this->global_path_object_->setDynamic(true);
    this->global_path_object_->setBoundingBox(Ogre::AxisAlignedBox::BOX_INFINITE);
    this->scene_node_->attachObject(this->global_path_object_);

    this->sub_ = this->nh_->subscribe(
        "/move_base/TebLocalPlannerROS/local_plan", 1, &PathVisualizerPlugin::PathCallback, this);
    this->global_sub_ = this->nh_->subscribe(
        "/move_base/GlobalPlanner/plan", 1, &PathVisualizerPlugin::GlobalPathCallback, this);
    this->global_fallback_sub_ = this->nh_->subscribe(
        "/move_base/NavfnROS/plan", 1, &PathVisualizerPlugin::GlobalPathCallback, this);

    this->initialized_ = true;
    this->start_wall_time_ = ros::WallTime::now();
    ROS_INFO(
        "[PathVisualizerPlugin] LOADED! local=/move_base/TebLocalPlannerROS/local_plan "
        "global=/move_base/GlobalPlanner/plan|/move_base/NavfnROS/plan");
  }

  void PathCallback(const nav_msgs::Path::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(this->mutex_);
    this->latest_path_ = *msg;
    this->new_path_received_ = true;
    if (!this->logged_first_local_msg_) {
      this->logged_first_local_msg_ = true;
      ROS_INFO_STREAM("[PathVisualizerPlugin] first local path received frame="
                      << (msg->header.frame_id.empty() ? "<empty>" : msg->header.frame_id)
                      << " poses=" << msg->poses.size());
    }
  }

  void GlobalPathCallback(const nav_msgs::Path::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(this->mutex_);
    this->global_path_ = *msg;
    this->new_global_path_received_ = true;
    if (!this->logged_first_global_msg_) {
      this->logged_first_global_msg_ = true;
      ROS_INFO_STREAM("[PathVisualizerPlugin] first global path received frame="
                      << (msg->header.frame_id.empty() ? "<empty>" : msg->header.frame_id)
                      << " poses=" << msg->poses.size());
    }
  }

  void OnUpdate() {
    if (!this->initialized_) {
      this->TryInitialize();
      return;
    }

    if (this->local_path_object_) {
      this->local_path_object_->clear();
    }
    if (this->global_path_object_) {
      this->global_path_object_->clear();
    }

    nav_msgs::Path local_path_copy;
    nav_msgs::Path global_path_copy;
    {
      std::lock_guard<std::mutex> lock(this->mutex_);
      local_path_copy = this->latest_path_;
      global_path_copy = this->global_path_;
      this->new_path_received_ = false;
      this->new_global_path_received_ = false;
    }

    this->WarnIfMissingPaths();

    ignition::math::Pose3d robot_pose_world = ignition::math::Pose3d::Zero;
    const bool robot_found = this->ResolveRobotPose(robot_pose_world);
    const auto anchor_pose = this->visual_->WorldPose();

    std::vector<ignition::math::Vector3d> local_world_points =
        this->BuildLocalWorldPoints(local_path_copy, robot_found, robot_pose_world);
    if (local_world_points.size() >= 2) {
      this->DrawLocalBand(local_world_points, anchor_pose);
      if (!this->logged_first_local_draw_) {
        this->logged_first_local_draw_ = true;
        ROS_INFO_STREAM("[PathVisualizerPlugin] rendering local plan band with "
                        << local_world_points.size() << " points");
      }
    }

    std::vector<ignition::math::Vector3d> global_world_points =
        this->BuildGlobalWorldPoints(global_path_copy, robot_found, robot_pose_world);
    if (global_world_points.size() >= 2) {
      this->DrawGlobalLine(global_world_points, anchor_pose);
      if (!this->logged_first_global_draw_) {
        this->logged_first_global_draw_ = true;
        ROS_INFO_STREAM("[PathVisualizerPlugin] rendering global path line with "
                        << global_world_points.size() << " points");
      }
    }
  }

  void WarnIfMissingPaths() {
    const double elapsed = (ros::WallTime::now() - this->start_wall_time_).toSec();
    if (elapsed < 10.0) {
      return;
    }
    if (!this->logged_first_local_msg_) {
      ROS_WARN_THROTTLE(10.0,
                        "[PathVisualizerPlugin] still waiting for usable local plan on "
                        "/move_base/TebLocalPlannerROS/local_plan");
    }
    if (!this->logged_first_global_msg_) {
      ROS_WARN_THROTTLE(10.0,
                        "[PathVisualizerPlugin] still waiting for usable global path on "
                        "/move_base/GlobalPlanner/plan or /move_base/NavfnROS/plan");
    }
  }

  bool ResolveRobotPose(ignition::math::Pose3d& robot_pose_world) {
    if (!this->robot_visual_ && this->scene_) {
      this->robot_visual_ = this->scene_->GetVisual("heron");
      if (!this->robot_visual_) this->robot_visual_ = this->scene_->GetVisual("heron::base_link::visual");
      if (!this->robot_visual_) this->robot_visual_ = this->scene_->GetVisual("heron::base_link");
      if (!this->robot_visual_) this->robot_visual_ = this->scene_->GetVisual("heron::base_footprint::visual");
      if (!this->robot_visual_) this->robot_visual_ = this->scene_->GetVisual("heron::base_footprint");
      if (!this->robot_visual_) this->robot_visual_ = this->scene_->GetVisual("heron_sim::heron::base_link::visual");
    }

    if (!this->robot_visual_) {
      return false;
    }

    robot_pose_world = this->robot_visual_->WorldPose();
    return true;
  }

  std::vector<ignition::math::Vector3d> BuildLocalWorldPoints(
      const nav_msgs::Path& path,
      bool robot_found,
      const ignition::math::Pose3d& robot_pose_world) {
    std::vector<ignition::math::Vector3d> points;
    if (path.poses.empty()) {
      return points;
    }

    const bool robot_local = this->TreatAsRobotLocal(path, robot_found, robot_pose_world);
    if (robot_found) {
      ignition::math::Vector3d robot_point = robot_pose_world.Pos();
      robot_point.Z() = 0.0;
      this->AppendIfDistinct(points, robot_point, 0.05);
    }

    for (const auto& pose_stamped : path.poses) {
      ignition::math::Vector3d point(
          pose_stamped.pose.position.x,
          pose_stamped.pose.position.y,
          pose_stamped.pose.position.z);
      if (robot_local && robot_found) {
        point = robot_pose_world.Rot() * point + robot_pose_world.Pos();
      }
      point.Z() = 0.0;
      this->AppendIfDistinct(points, point, 0.03);
    }
    return points;
  }

  std::vector<ignition::math::Vector3d> BuildGlobalWorldPoints(
      const nav_msgs::Path& path,
      bool robot_found,
      const ignition::math::Pose3d& robot_pose_world) {
    std::vector<ignition::math::Vector3d> points;
    if (path.poses.empty()) {
      return points;
    }

    if (robot_found) {
      ignition::math::Vector3d robot_point = robot_pose_world.Pos();
      robot_point.Z() = 0.0;
      this->AppendIfDistinct(points, robot_point, 0.05);
    }

    for (const auto& pose_stamped : path.poses) {
      ignition::math::Vector3d point(
          pose_stamped.pose.position.x,
          pose_stamped.pose.position.y,
          0.0);
      this->AppendIfDistinct(points, point, 0.03);
    }
    return points;
  }

  bool TreatAsRobotLocal(
      const nav_msgs::Path& path,
      bool robot_found,
      const ignition::math::Pose3d& robot_pose_world) {
    std::string frame = path.header.frame_id;
    bool robot_local = false;

    if (frame.find("base_link") != std::string::npos ||
        frame.find("base_footprint") != std::string::npos) {
      robot_local = true;
    } else if (frame == "map" || frame == "odom" || frame == "world") {
      robot_local = false;
    } else if (!path.poses.empty() && robot_found) {
      ignition::math::Vector3d p0(
          path.poses.front().pose.position.x,
          path.poses.front().pose.position.y,
          path.poses.front().pose.position.z);
      const double dist_to_origin = p0.Length();
      const double dist_to_robot = p0.Distance(robot_pose_world.Pos());
      robot_local = (dist_to_origin < 1.0 && dist_to_robot > 2.0);
    }

    if (!this->local_frame_mode_known_ || this->local_frame_mode_robot_local_ != robot_local) {
      this->local_frame_mode_known_ = true;
      this->local_frame_mode_robot_local_ = robot_local;
      ROS_INFO_STREAM("[PathVisualizerPlugin] treating local plan frame="
                      << (frame.empty() ? "<empty>" : frame)
                      << " as " << (robot_local ? "robot-local" : "world/map"));
    }

    return robot_local;
  }

  void DrawLocalBand(
      const std::vector<ignition::math::Vector3d>& world_points,
      const ignition::math::Pose3d& anchor_pose) {
    if (!this->local_path_object_ || world_points.size() < 2) {
      return;
    }

    this->local_path_object_->begin(
        "Gazebo/BlueTransparentOverlay", Ogre::RenderOperation::OT_TRIANGLE_STRIP);

    const double z_offset = 0.30;
    const double half_width = 0.92;

    for (size_t i = 0; i < world_points.size(); ++i) {
      ignition::math::Vector3d p_curr = world_points[i];

      ignition::math::Vector3d p_next;
      if (i + 1 < world_points.size()) {
        p_next = world_points[i + 1];
      } else if (i > 0) {
        p_next = p_curr + (p_curr - world_points[i - 1]);
      } else {
        p_next = p_curr + ignition::math::Vector3d(1, 0, 0);
      }

      ignition::math::Vector3d dir = p_next - p_curr;
      if (dir.Length() < 1e-6) {
        dir = ignition::math::Vector3d(1, 0, 0);
      } else {
        dir.Normalize();
      }
      ignition::math::Vector3d perp(-dir.Y(), dir.X(), 0.0);

      ignition::math::Vector3d left_world = p_curr + perp * half_width;
      ignition::math::Vector3d right_world = p_curr - perp * half_width;

      ignition::math::Vector3d left_local =
          anchor_pose.Rot().Inverse() * (left_world - anchor_pose.Pos());
      ignition::math::Vector3d right_local =
          anchor_pose.Rot().Inverse() * (right_world - anchor_pose.Pos());

      this->local_path_object_->position(left_local.X(), left_local.Y(), left_local.Z() + z_offset);
      this->local_path_object_->colour(0.30f, 0.66f, 1.0f, 0.30f);

      this->local_path_object_->position(right_local.X(), right_local.Y(), right_local.Z() + z_offset);
      this->local_path_object_->colour(0.30f, 0.66f, 1.0f, 0.30f);
    }

    this->local_path_object_->end();
  }

  void DrawGlobalLine(
      const std::vector<ignition::math::Vector3d>& world_points,
      const ignition::math::Pose3d& anchor_pose) {
    if (!this->global_path_object_ || world_points.size() < 2) {
      return;
    }

    this->global_path_object_->begin(
        "Gazebo/BlueTransparentOverlay", Ogre::RenderOperation::OT_TRIANGLE_STRIP);

    const double z_offset = 0.40;
    const double half_width = 0.12;

    for (size_t i = 0; i < world_points.size(); ++i) {
      ignition::math::Vector3d p_curr = world_points[i];
      ignition::math::Vector3d p_next;
      if (i + 1 < world_points.size()) {
        p_next = world_points[i + 1];
      } else if (i > 0) {
        p_next = p_curr + (p_curr - world_points[i - 1]);
      } else {
        p_next = p_curr + ignition::math::Vector3d(1, 0, 0);
      }

      ignition::math::Vector3d dir = p_next - p_curr;
      if (dir.Length() < 1e-6) {
        dir = ignition::math::Vector3d(1, 0, 0);
      } else {
        dir.Normalize();
      }
      ignition::math::Vector3d perp(-dir.Y(), dir.X(), 0.0);

      ignition::math::Vector3d left_world = p_curr + perp * half_width;
      ignition::math::Vector3d right_world = p_curr - perp * half_width;

      ignition::math::Vector3d left_local =
          anchor_pose.Rot().Inverse() * (left_world - anchor_pose.Pos());
      ignition::math::Vector3d right_local =
          anchor_pose.Rot().Inverse() * (right_world - anchor_pose.Pos());

      this->global_path_object_->position(left_local.X(), left_local.Y(), left_local.Z() + z_offset);
      this->global_path_object_->colour(0.12f, 0.56f, 1.0f, 0.95f);

      this->global_path_object_->position(right_local.X(), right_local.Y(), right_local.Z() + z_offset);
      this->global_path_object_->colour(0.12f, 0.56f, 1.0f, 0.95f);
    }

    this->global_path_object_->end();
  }

  void AppendIfDistinct(
      std::vector<ignition::math::Vector3d>& points,
      const ignition::math::Vector3d& point,
      double min_distance) {
    if (points.empty() || points.back().Distance(point) > min_distance) {
      points.push_back(point);
    }
  }

  rendering::VisualPtr visual_;
  ros::NodeHandle* nh_;
  ros::Subscriber sub_;
  ros::Subscriber global_sub_;
  ros::Subscriber global_fallback_sub_;
  rendering::ScenePtr scene_;
  Ogre::ManualObject* local_path_object_;
  Ogre::ManualObject* global_path_object_;
  Ogre::SceneNode* scene_node_;
  rendering::VisualPtr robot_visual_;
  event::ConnectionPtr update_connection_;

  std::mutex mutex_;
  nav_msgs::Path latest_path_;
  nav_msgs::Path global_path_;
  bool new_path_received_;
  bool new_global_path_received_;
  bool initialized_;
  bool warned_waiting_for_ros_;
  bool logged_first_local_msg_;
  bool logged_first_global_msg_;
  bool logged_first_local_draw_;
  bool logged_first_global_draw_;
  bool local_frame_mode_known_;
  bool local_frame_mode_robot_local_;
  ros::WallTime start_wall_time_;
};

GZ_REGISTER_VISUAL_PLUGIN(PathVisualizerPlugin)
}  // namespace gazebo
