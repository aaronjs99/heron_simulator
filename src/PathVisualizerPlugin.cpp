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
#include <mutex>
#include <vector>

namespace gazebo {
class PathVisualizerPlugin : public VisualPlugin {
public:
  PathVisualizerPlugin()
      : visual_(nullptr),
        nh_(nullptr),
        scene_(nullptr),
        manual_object_(nullptr),
        global_path_object_(nullptr),
        preview_path_object_(nullptr),
        scene_node_(nullptr),
        robot_visual_(nullptr),
        new_path_received_(false),
        new_global_path_received_(false),
        new_preview_path_received_(false),
        initialized_(false),
        warned_waiting_for_ros_(false) {}
  virtual ~PathVisualizerPlugin() {
    if (nh_) {
      nh_->shutdown();
      delete nh_;
    }
  }

  void Load(rendering::VisualPtr _parent, sdf::ElementPtr _sdf) {
    this->visual_ = _parent;
    this->update_connection_ =
        event::Events::ConnectPreRender(std::bind(&PathVisualizerPlugin::OnUpdate, this));

    this->TryInitialize();
  }

private:
  void TryInitialize() {
    if (this->initialized_) return;
    if (!this->visual_) return;

    if (!ros::isInitialized()) {
      if (!this->warned_waiting_for_ros_) {
        ROS_DEBUG_STREAM("[PathVisualizerPlugin] Gazebo ROS is not ready yet; delaying plugin initialization.");
        this->warned_waiting_for_ros_ = true;
      }
      return;
    }

    this->nh_ = new ros::NodeHandle("~");

    // Get Gazebo Scene
    this->scene_ = this->visual_->GetScene();
    if (!this->scene_) return;
    Ogre::SceneManager* scene_manager = this->scene_->OgreSceneManager();
    if (!scene_manager) return;

    static int instance_id = 0;
    const std::string suffix = "_" + std::to_string(instance_id++);

    // Create ManualObject for Ribbon (Local Plan)
    this->scene_node_ = this->visual_->GetSceneNode()->createChildSceneNode();
    this->manual_object_ = scene_manager->createManualObject("PathRibbon_MO" + suffix);
    this->manual_object_->setDynamic(true);
    // CRITICAL: Infinite BBox to prevent culling
    this->manual_object_->setBoundingBox(Ogre::AxisAlignedBox::BOX_INFINITE);
    this->scene_node_->attachObject(this->manual_object_);

    // Create ManualObject for Global Plan (thin line)
    this->global_path_object_ = scene_manager->createManualObject("GlobalPath_MO" + suffix);
    this->global_path_object_->setDynamic(true);
    this->global_path_object_->setBoundingBox(Ogre::AxisAlignedBox::BOX_INFINITE);
    this->scene_node_->attachObject(this->global_path_object_);

    // Create ManualObject for the higher-level mission preview path.
    this->preview_path_object_ = scene_manager->createManualObject("PreviewPath_MO" + suffix);
    this->preview_path_object_->setDynamic(true);
    this->preview_path_object_->setBoundingBox(Ogre::AxisAlignedBox::BOX_INFINITE);
    this->scene_node_->attachObject(this->preview_path_object_);

    // Sub to nav path (Local Plan)
    this->sub_ =
        this->nh_->subscribe("/move_base/TebLocalPlannerROS/local_plan", 1, &PathVisualizerPlugin::PathCallback, this);

    // Sub to the global planner output. Support both current and legacy topic names.
    this->global_sub_ = this->nh_->subscribe(
        "/move_base/GlobalPlanner/plan", 1, &PathVisualizerPlugin::GlobalPathCallback, this);
    this->global_fallback_sub_ = this->nh_->subscribe(
        "/move_base/NavfnROS/plan", 1, &PathVisualizerPlugin::GlobalPathCallback, this);
    this->preview_sub_ =
        this->nh_->subscribe("/oracle/viz_path", 1, &PathVisualizerPlugin::PreviewPathCallback, this);

    this->initialized_ = true;
    ROS_INFO("[PathVisualizerPlugin] LOADED! preview=/oracle/viz_path local=/move_base/TebLocalPlannerROS/local_plan global=/move_base/GlobalPlanner/plan|/move_base/NavfnROS/plan");
  }

  void PathCallback(const nav_msgs::Path::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(this->mutex_);
    this->latest_path_ = *msg;
    this->new_path_received_ = true;
  }

  void GlobalPathCallback(const nav_msgs::Path::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(this->mutex_);
    this->global_path_ = *msg;
    this->new_global_path_received_ = true;
  }

  void PreviewPathCallback(const nav_msgs::Path::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(this->mutex_);
    this->preview_path_ = *msg;
    this->new_preview_path_received_ = true;
  }

  void OnUpdate() {
    if (!this->initialized_) {
      this->TryInitialize();
      return;
    }

    if (this->new_path_received_) {
      // ROS_WARN_STREAM_THROTTLE(5.0, "Updating Ribbon...");
      this->new_path_received_ = false;
    }

    this->manual_object_->clear();

    // --- 1. Frame Detection ---
    ignition::math::Pose3d robot_pose_world = ignition::math::Pose3d::Zero;
    bool robot_found = false;

    if (!this->robot_visual_ && this->scene_) {
      this->robot_visual_ = this->scene_->GetVisual("heron");
      if (!this->robot_visual_) this->robot_visual_ = this->scene_->GetVisual("heron::base_link::visual");
      if (!this->robot_visual_) this->robot_visual_ = this->scene_->GetVisual("heron::base_link");
      if (!this->robot_visual_) this->robot_visual_ = this->scene_->GetVisual("heron::base_footprint::visual");
      if (!this->robot_visual_) this->robot_visual_ = this->scene_->GetVisual("heron::base_footprint");
      if (!this->robot_visual_) this->robot_visual_ = this->scene_->GetVisual("heron_sim::heron::base_link::visual");
    }

    if (this->robot_visual_) {
      robot_pose_world = this->robot_visual_->WorldPose();
      robot_found = true;
    }

    auto anchor_pose = this->visual_->WorldPose();

    bool path_is_local = false;
    if (this->latest_path_.poses.size() >= 2) {
      const auto& first_pose = this->latest_path_.poses[0].pose;
      ignition::math::Vector3d p0(first_pose.position.x, first_pose.position.y, first_pose.position.z);

      if (robot_found) {
        double dist_to_origin = p0.Length();
        double dist_to_robot = p0.Distance(robot_pose_world.Pos());
        if (dist_to_origin < 1.0 && dist_to_robot > 2.0) {
          path_is_local = true;
          ROS_WARN_STREAM_THROTTLE(5.0, "[PathVisualizerPlugin] Frame Mismatch! Treating as LOCAL. Robot: "
                                            << robot_pose_world.Pos() << ", Path P0: " << p0);
        }
      }

      // --- 2. Geometry Generation ---
      // Use Gazebo-native overlay materials. The previous RViz-style material
      // name was not available in Gazebo, which could leave the path invisible.
      this->manual_object_->begin("Gazebo/LightBlueLaser", Ogre::RenderOperation::OT_TRIANGLE_STRIP);

      const double z_offset = 0.30;
      const double half_width = 0.55;

      for (size_t i = 0; i < this->latest_path_.poses.size(); ++i) {
        ignition::math::Vector3d p_curr = GetWorldPos(i, path_is_local, robot_found, robot_pose_world);

        ignition::math::Vector3d p_next;
        if (i + 1 < this->latest_path_.poses.size()) {
          p_next = GetWorldPos(i + 1, path_is_local, robot_found, robot_pose_world);
        } else if (i > 0) {
          p_next = p_curr + (p_curr - GetWorldPos(i - 1, path_is_local, robot_found, robot_pose_world));
        } else {
          p_next = p_curr + ignition::math::Vector3d(1, 0, 0);
        }

        ignition::math::Vector3d dir = (p_next - p_curr).Normalize();
        ignition::math::Vector3d perp(-dir.Y(), dir.X(), 0);

        auto to_local = [&](ignition::math::Vector3d world_pt) {
          return anchor_pose.Rot().Inverse() * (world_pt - anchor_pose.Pos());
        };

        ignition::math::Vector3d v_left_world = p_curr + perp * half_width;
        ignition::math::Vector3d v_right_world = p_curr - perp * half_width;

        auto v_left = to_local(v_left_world);
        auto v_right = to_local(v_right_world);

        this->manual_object_->position(v_left.X(), v_left.Y(), v_left.Z() + z_offset);
        this->manual_object_->colour(0.08, 0.82, 1.0, 0.96);

        this->manual_object_->position(v_right.X(), v_right.Y(), v_right.Z() + z_offset);
        this->manual_object_->colour(0.08, 0.82, 1.0, 0.96);
      }

      this->manual_object_->end();
    }

    // --- 3. Global Path (Narrow Dark Blue Ribbon - 15% of local width) ---
    this->global_path_object_->clear();

    if (this->global_path_.poses.size() >= 2) {
      this->global_path_object_->begin("Gazebo/BlueTransparentOverlay", Ogre::RenderOperation::OT_TRIANGLE_STRIP);

      const double global_z_offset = 0.38;
      const double global_half_width = 0.18;

      // Transform from world to local anchor frame
      auto to_local = [&](ignition::math::Vector3d world_pt) {
        return anchor_pose.Rot().Inverse() * (world_pt - anchor_pose.Pos());
      };

      for (size_t i = 0; i < this->global_path_.poses.size(); ++i) {
        const auto& msg_pose = this->global_path_.poses[i].pose;
        ignition::math::Vector3d p_curr(msg_pose.position.x, msg_pose.position.y, 0.0);

        // Calculate direction for perpendicular
        ignition::math::Vector3d p_next;
        if (i + 1 < this->global_path_.poses.size()) {
          const auto& next_pose = this->global_path_.poses[i + 1].pose;
          p_next = ignition::math::Vector3d(next_pose.position.x, next_pose.position.y, 0.0);
        } else if (i > 0) {
          const auto& prev_pose = this->global_path_.poses[i - 1].pose;
          ignition::math::Vector3d p_prev(prev_pose.position.x, prev_pose.position.y, 0.0);
          p_next = p_curr + (p_curr - p_prev);
        } else {
          p_next = p_curr + ignition::math::Vector3d(1, 0, 0);
        }

        ignition::math::Vector3d dir = (p_next - p_curr).Normalize();
        ignition::math::Vector3d perp(-dir.Y(), dir.X(), 0);

        ignition::math::Vector3d v_left_world = p_curr + perp * global_half_width;
        ignition::math::Vector3d v_right_world = p_curr - perp * global_half_width;

        auto v_left = to_local(v_left_world);
        auto v_right = to_local(v_right_world);

        this->global_path_object_->position(v_left.X(), v_left.Y(), global_z_offset);
        this->global_path_object_->colour(0.02, 0.30, 0.98, 1.0);

        this->global_path_object_->position(v_right.X(), v_right.Y(), global_z_offset);
        this->global_path_object_->colour(0.02, 0.30, 0.98, 1.0);
      }

      this->global_path_object_->end();
    }

    // --- 4. High-level mission preview path (gold ribbon) ---
    this->preview_path_object_->clear();

    if (this->preview_path_.poses.size() >= 2) {
      this->preview_path_object_->begin("Gazebo/Gold", Ogre::RenderOperation::OT_TRIANGLE_STRIP);

      const double preview_z_offset = 0.52;
      const double preview_half_width = 0.30;

      auto to_local = [&](ignition::math::Vector3d world_pt) {
        return anchor_pose.Rot().Inverse() * (world_pt - anchor_pose.Pos());
      };

      for (size_t i = 0; i < this->preview_path_.poses.size(); ++i) {
        const auto& msg_pose = this->preview_path_.poses[i].pose;
        ignition::math::Vector3d p_curr(msg_pose.position.x, msg_pose.position.y, 0.0);

        ignition::math::Vector3d p_next;
        if (i + 1 < this->preview_path_.poses.size()) {
          const auto& next_pose = this->preview_path_.poses[i + 1].pose;
          p_next = ignition::math::Vector3d(next_pose.position.x, next_pose.position.y, 0.0);
        } else if (i > 0) {
          const auto& prev_pose = this->preview_path_.poses[i - 1].pose;
          ignition::math::Vector3d p_prev(prev_pose.position.x, prev_pose.position.y, 0.0);
          p_next = p_curr + (p_curr - p_prev);
        } else {
          p_next = p_curr + ignition::math::Vector3d(1, 0, 0);
        }

        ignition::math::Vector3d dir = (p_next - p_curr).Normalize();
        ignition::math::Vector3d perp(-dir.Y(), dir.X(), 0);

        ignition::math::Vector3d v_left_world = p_curr + perp * preview_half_width;
        ignition::math::Vector3d v_right_world = p_curr - perp * preview_half_width;

        auto v_left = to_local(v_left_world);
        auto v_right = to_local(v_right_world);

        this->preview_path_object_->position(v_left.X(), v_left.Y(), preview_z_offset);
        this->preview_path_object_->colour(1.0, 0.78, 0.10, 0.98);

        this->preview_path_object_->position(v_right.X(), v_right.Y(), preview_z_offset);
        this->preview_path_object_->colour(1.0, 0.78, 0.10, 0.98);
      }

      this->preview_path_object_->end();
    }
  }

  // Helper to get World position
  ignition::math::Vector3d GetWorldPos(size_t i, bool is_local, bool robot_found,
                                       const ignition::math::Pose3d& robot_pose) {
    const auto& msg_pose = this->latest_path_.poses[i].pose;
    ignition::math::Vector3d p_msg(msg_pose.position.x, msg_pose.position.y, msg_pose.position.z);

    if (is_local && robot_found) {
      return robot_pose.Rot() * p_msg + robot_pose.Pos();
    }
    return p_msg;
  }

  rendering::VisualPtr visual_;
  ros::NodeHandle* nh_;
  ros::Subscriber sub_;
  rendering::ScenePtr scene_;
  Ogre::ManualObject* manual_object_;
  Ogre::SceneNode* scene_node_;
  rendering::VisualPtr robot_visual_;
  event::ConnectionPtr update_connection_;

  std::mutex mutex_;
  nav_msgs::Path latest_path_;
  nav_msgs::Path global_path_;
  bool new_path_received_;
  bool new_global_path_received_;
  ros::Subscriber global_sub_;
  ros::Subscriber global_fallback_sub_;
  ros::Subscriber preview_sub_;
  Ogre::ManualObject* global_path_object_;
  Ogre::ManualObject* preview_path_object_;
  bool initialized_;
  bool warned_waiting_for_ros_;
  nav_msgs::Path preview_path_;
  bool new_preview_path_received_;
};

// Register this plugin with the simulator
GZ_REGISTER_VISUAL_PLUGIN(PathVisualizerPlugin)
}  // namespace gazebo
