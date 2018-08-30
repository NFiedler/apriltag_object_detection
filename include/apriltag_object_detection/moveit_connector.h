#ifndef APRILTAG_OBJECT_DETECTION
#define APRILTAG_OBJECT_DETECTION

#include <ros/ros.h>
#include <map>
#include <vector>
#include <algorithm>
#include <visualization_msgs/MarkerArray.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <apriltag_object_detection/marker_to_collision_object.h>
#include <pr2_phantom/State.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>

class MoveitConnector {
public:
  MoveitConnector();
  ~MoveitConnector(){};

  void objectsCallback(const visualization_msgs::MarkerArray &msg);
  void stateCallback(const pr2_phantom::State &msg);
  void planPickCallback(const geometry_msgs::PointStamped &msg);
  void planPlaceCallback(const geometry_msgs::PointStamped &msg);

private:
  ros::NodeHandle nh_;
  std::map<std::string, ros::Timer> collision_object_remove_timers_;
  std::vector<std::string> pci_collision_objects_;
  uint object_lifetime_secs_ = 10;
  bool update_enabled_;

  tf::TransformListener collision_object_transform_listener_;
  ros::Subscriber object_sub_; // subscribes to the apriltag objects
  ros::Subscriber state_sub_; // subscribes to the state of the pick-place node
  ros::Subscriber plan_pick_sub_;
  ros::Subscriber plan_place_sub_;
  moveit::planning_interface::PlanningSceneInterface *psi_;
  void remove_collision_object(std::string object_id);
  void enable_updates();
  void disable_updates();

  visualization_msgs::MarkerArray marker_array_;
};

#endif
