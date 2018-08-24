#include <apriltag_object_detection/moveit_connector.h>

MoveitConnector::MoveitConnector() : nh_() {
  psi_ = new moveit::planning_interface::PlanningSceneInterface();

  object_sub_ = nh_.subscribe(
      "apriltag_objects", 1, &MoveitConnector::objectsCallback, this);
}

void MoveitConnector::objectsCallback(const visualization_msgs::MarkerArray &msg){
  moveit_msgs::CollisionObject collision_object;
  std::string collision_object_id;
  for(uint i = 0; i < msg.markers.size(); i++) {
    if(!markerMsgToCollisionObjectMsg(msg.markers[i], "/odom_combined", collision_object)) {
      ROS_ERROR_STREAM("Marker could not get transformed to a CollisionObject!");
      continue;
    }
    collision_object_id = static_cast<std::string>(collision_object.id);
    if ( collision_object_remove_timers_.find(collision_object_id) == collision_object_remove_timers_.end() ) {
      collision_object_remove_timers_.insert(std::pair<std::string, ros::Timer>(collision_object_id, nh_.createTimer(ros::Duration(object_lifetime_secs_), [this, id = collision_object_id](const ros::TimerEvent& evt){remove_collision_object(id);}, true)));
    } else {
      collision_object_remove_timers_[collision_object_id].setPeriod(ros::Duration(object_lifetime_secs_), true); // resetting period to reset the timer
    }
    if(!psi_->getAttachedObjects(std::vector<std::string>({collision_object_id})).empty()){ // just checking if the object is an attached collision object
      ROS_DEBUG_STREAM("Object detection detected attached object: " << collision_object.id);
      if (std::find(pci_collision_objects_.begin(), pci_collision_objects_.end(), collision_object_id) != pci_collision_objects_.end()) {
        pci_collision_objects_.erase(std::remove(pci_collision_objects_.begin(), pci_collision_objects_.end(), collision_object_id), pci_collision_objects_.end()); // removing objects when attached
      }
      continue;
    }
    if (std::find(pci_collision_objects_.begin(), pci_collision_objects_.end(), collision_object_id) != pci_collision_objects_.end()) {
      collision_object.operation = collision_object.MOVE; // move object when it is known
    } else {
      collision_object.operation = collision_object.ADD;
      pci_collision_objects_.push_back(collision_object_id); // add object to known objects
    }
    if (!psi_->applyCollisionObject(collision_object)) {
      ROS_ERROR_STREAM("Failed to add CollisionObject to planning scene!");
      continue;
    }
  }

}

void MoveitConnector::remove_collision_object(std::string object_id){
  ROS_DEBUG_STREAM("Removing object \"" << object_id << "\" from planning scene because it was too old.");
  psi_->removeCollisionObjects(std::vector<std::string>({object_id}));
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "MoveitConnector");

  MoveitConnector moveit_connector;

  ros::spin();
  return 0;
};
