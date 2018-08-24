#include <apriltag_object_detection/moveit_connector.h>

MoveitConnector::MoveitConnector() : nh_() {
  psi_ = new moveit::planning_interface::PlanningSceneInterface();

  object_sub_ = nh_.subscribe(
      "apriltag_objects", 1, &MoveitConnector::objectsCallback, this);
}

void MoveitConnector::objectsCallback(const visualization_msgs::MarkerArray &msg){
  moveit_msgs::CollisionObject collision_object;
  for(int i = 0; i < msg.markers.size(); i++) {
    if(!markerMsgToCollisionObjectMsg(msg.markers[i], "/odom_combined", collision_object)) {
      ROS_ERROR_STREAM("Marker could not get transformed to a CollisionObject!");
      continue;
    }
    if(!psi_->getAttachedObjects(std::vector<std::string>({static_cast<std::string>(collision_object.id)})).empty()){ // just checking if the object is an attached collision object
      ROS_DEBUG_STREAM("Object detection detected attached object: " << collision_object.id);
      continue;
    }
    // TODO: use MOVE, when the object exists already. (Only the pose in the CollisionObject)
    collision_object.operation = collision_object.ADD;
    if(!psi_->applyCollisionObject(collision_object)) {
      ROS_ERROR_STREAM("Failed to add CollisionObject to planning scene!");
      continue;
    }
  }

}

int main(int argc, char **argv) {
  ros::init(argc, argv, "MoveitConnector");

  MoveitConnector moveit_connector;

  ros::spin();
  return 0;
};
