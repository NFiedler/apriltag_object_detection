#ifndef APRILTAG_OBJECT_DETECTION
#define APRILTAG_OBJECT_DETECTION

#include <ros/ros.h>
#include <map>
#include <vector>
#include <visualization_msgs/MarkerArray.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <apriltag_object_detection/marker_to_collision_object.h>

class MoveitConnector {
public:
  MoveitConnector();
  ~MoveitConnector(){};

  void objectsCallback(const visualization_msgs::MarkerArray &msg);

private:
  ros::NodeHandle nh_;
  std::map<std::string, ros::Timer> collision_object_remove_timers_;
  uint object_lifetime_secs_ = 10;


  ros::Subscriber object_sub_;
  moveit::planning_interface::PlanningSceneInterface *psi_;
  void remove_collision_object(std::string object_id);

  visualization_msgs::MarkerArray marker_array_;
};

#endif
