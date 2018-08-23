

#include <apriltag_object_detection/moveit_connector.h>


MoveitConnector::MoveitConnector() : nh_() {
  psi_ = new moveit::planning_interface::PlanningSceneInterface();
  
  object_sub_ = nh_.subscribe(
      "apriltag_objects", 1, &MoveitConnector::objectsCallback, this);
}

void MoveitConnector::objectsCallback(const visualization_msgs::MarkerArray &msg){
  
} 

int main(int argc, char **argv) {
  ros::init(argc, argv, "MoveitConnector");

  MoveitConnector moveit_connector;

  ros::spin();
  return 0;
};
