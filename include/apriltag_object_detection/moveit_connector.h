#ifndef APRILTAG_OBJECT_DETECTION
#define APRILTAG_OBJECT_DETECTION

#include <apriltags2_ros/AprilTagDetectionArray.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

class MoveitConnector {
public:
  MoveitConnector();
  ~MoveitConnector(){};

  void objectsCallback(const visualization_msgs::MarkerArray &msg);

private:
  ros::NodeHandle nh_;

  ros::Subscriber object_sub_;
  moveit::planning_interface::PlanningSceneInterface *psi_;

  visualization_msgs::MarkerArray marker_array_;
};

#endif
