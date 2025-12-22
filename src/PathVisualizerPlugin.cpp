#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/rendering/Visual.hh>
#include <gazebo/rendering/Scene.hh>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreVector3.h>
#include <vector>
#include <mutex>

namespace gazebo
{
  class PathVisualizerPlugin : public VisualPlugin
  {
    public:
      PathVisualizerPlugin() : visual_(nullptr), scene_(nullptr), manual_object_(nullptr), scene_node_(nullptr), robot_visual_(nullptr), new_path_received_(false) {}
      virtual ~PathVisualizerPlugin() 
      {
         if (nh_) {
           nh_->shutdown();
           delete nh_;
         }
      }

      void Load(rendering::VisualPtr _parent, sdf::ElementPtr _sdf)
      {
        this->visual_ = _parent;
        
        if (!ros::isInitialized())
        {
          ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized");
          return;
        }

        this->nh_ = new ros::NodeHandle("~");

        // Get Gazebo Scene
        this->scene_ = this->visual_->GetScene();
        Ogre::SceneManager *scene_manager = this->scene_->OgreSceneManager();

        // Create ManualObject for Ribbon
        this->scene_node_ = this->visual_->GetSceneNode()->createChildSceneNode();
        this->manual_object_ = scene_manager->createManualObject("PathRibbon_MO");
        this->manual_object_->setDynamic(true);
        // CRITICAL: Infinite BBox to prevent culling
        this->manual_object_->setBoundingBox(Ogre::AxisAlignedBox::BOX_INFINITE);
        this->scene_node_->attachObject(this->manual_object_);
        
        // Sub to nav path (Local Plan)
        this->sub_ = this->nh_->subscribe("/move_base/TebLocalPlannerROS/local_plan", 1, 
            &PathVisualizerPlugin::PathCallback, this);

        this->update_connection_ = event::Events::ConnectPreRender(
            std::bind(&PathVisualizerPlugin::OnUpdate, this));

        ROS_INFO("[PathVisualizerPlugin] LOADED! Ribbon Mode (ManualObject).");
      }

    private: 
        void PathCallback(const nav_msgs::Path::ConstPtr& msg)
        {
            std::lock_guard<std::mutex> lock(this->mutex_);
            this->latest_path_ = *msg;
            this->new_path_received_ = true;
        }

        void OnUpdate()
        {
            if (this->new_path_received_) {
                 // ROS_WARN_STREAM_THROTTLE(5.0, "Updating Ribbon...");
                 this->new_path_received_ = false;
            }

            this->manual_object_->clear();

            if (this->latest_path_.poses.size() < 2) return;

            // --- 1. Frame Detection ---
            ignition::math::Pose3d robot_pose_world = ignition::math::Pose3d::Zero;
            bool robot_found = false;
            
            if (!this->robot_visual_ && this->scene_) {
                 this->robot_visual_ = this->scene_->GetVisual("heron");
                 if (!this->robot_visual_) this->robot_visual_ = this->scene_->GetVisual("heron::base_link::visual");
                 if (!this->robot_visual_) this->robot_visual_ = this->scene_->GetVisual("heron::base_link");
                 if (!this->robot_visual_) this->robot_visual_ = this->scene_->GetVisual("heron_sim::heron::base_link::visual");
            }

            if (this->robot_visual_) {
                robot_pose_world = this->robot_visual_->WorldPose();
                robot_found = true;
            }

            auto anchor_pose = this->visual_->WorldPose(); 
            
            bool path_is_local = false;
            const auto& first_pose = this->latest_path_.poses[0].pose;
            ignition::math::Vector3d p0(first_pose.position.x, first_pose.position.y, first_pose.position.z);
            
            if (robot_found) {
                double dist_to_origin = p0.Length();
                double dist_to_robot = p0.Distance(robot_pose_world.Pos());
                if (dist_to_origin < 1.0 && dist_to_robot > 2.0) {
                    path_is_local = true;
                    ROS_WARN_STREAM_THROTTLE(5.0, "[PathVisualizerPlugin] Frame Mismatch! Treating as LOCAL. Robot: " << robot_pose_world.Pos() << ", Path P0: " << p0);
                }
            }

            // --- 2. Geometry Generation ---
            // Begin Triangle Strip
            this->manual_object_->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_TRIANGLE_STRIP);
            
            double z_offset = 0.05;
            double half_width = 0.5; // Total width 1.0m

            for (size_t i = 0; i < this->latest_path_.poses.size(); ++i)
            {
                // Get Current Position
                ignition::math::Vector3d p_curr = GetWorldPos(i, path_is_local, robot_found, robot_pose_world);
                
                // Get Next Position for Tangent
                ignition::math::Vector3d p_next;
                if (i + 1 < this->latest_path_.poses.size()) {
                    p_next = GetWorldPos(i+1, path_is_local, robot_found, robot_pose_world);
                } else if (i > 0) {
                    // Last point, use previous tangent
                    p_next = p_curr + (p_curr - GetWorldPos(i-1, path_is_local, robot_found, robot_pose_world));
                } else {
                    p_next = p_curr + ignition::math::Vector3d(1, 0, 0);
                }

                // Calculate Tangent and Perpendicular
                ignition::math::Vector3d dir = (p_next - p_curr).Normalize();
                ignition::math::Vector3d perp(-dir.Y(), dir.X(), 0); // 2D perpendicular (XY plane)
                
                // Transform Points to Local Anchor Frame
                auto to_local = [&](ignition::math::Vector3d world_pt) {
                    return anchor_pose.Rot().Inverse() * (world_pt - anchor_pose.Pos());
                };

                ignition::math::Vector3d v_left_world = p_curr + perp * half_width;
                ignition::math::Vector3d v_right_world = p_curr - perp * half_width;

                auto v_left = to_local(v_left_world);
                auto v_right = to_local(v_right_world);
                
                // Add Vertices (Triangle Strip Order: Left, Right, Left, Right...)
                // Left
                this->manual_object_->position(v_left.X(), v_left.Y(), v_left.Z() + z_offset);
                this->manual_object_->colour(0.0, 0.4, 1.0, 0.2); // R 0.0 G 0.4 B 1.0, A 0.2 (Translucent)
                
                // Right
                this->manual_object_->position(v_right.X(), v_right.Y(), v_right.Z() + z_offset);
                this->manual_object_->colour(0.0, 0.4, 1.0, 0.2);
            }
            
            this->manual_object_->end();
        }

        // Helper to get World position
        ignition::math::Vector3d GetWorldPos(size_t i, bool is_local, bool robot_found, const ignition::math::Pose3d& robot_pose)
        {
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
      bool new_path_received_;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_VISUAL_PLUGIN(PathVisualizerPlugin)
}
