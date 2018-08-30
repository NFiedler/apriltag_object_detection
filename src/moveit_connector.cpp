#include <apriltag_object_detection/moveit_connector.h>

MoveitConnector::MoveitConnector() : nh_() {
  psi_ = new moveit::planning_interface::PlanningSceneInterface();
  update_enabled_ = true;

  object_sub_ = nh_.subscribe(
      "apriltag_objects", 1, &MoveitConnector::objectsCallback, this);
  state_sub_ = nh_.subscribe(
      "pr2_phantom/state", 1, &MoveitConnector::stateCallback, this);
  plan_pick_sub_ = nh_.subscribe(
          "pr2_phantom/plan_pick", 1, &MoveitConnector::planPickCallback, this);
  plan_place_sub_ = nh_.subscribe(
          "pr2_phantom/plan_place", 1, &MoveitConnector::planPlaceCallback, this);
}

void MoveitConnector::stateCallback(const pr2_phantom::State &msg){
  // enabling updates when state is IDLE (after successful movement), PLANNING_FAILED (no current plan and no current attempt to plan) or ERROR (Error in the execution of a plan -> a new plan is needed)
  if (msg.val == pr2_phantom::State::IDLE || msg.val == pr2_phantom::State::PLANNING_FAILED || msg.val == pr2_phantom::State::ERROR) {
    enable_updates();
  }
}

void MoveitConnector::planPickCallback(const geometry_msgs::PointStamped &msg) {
  disable_updates();
}
void MoveitConnector::planPlaceCallback(const geometry_msgs::PointStamped &msg) {
  disable_updates();
}

void MoveitConnector::objectsCallback(const visualization_msgs::MarkerArray &msg){
  if (!update_enabled_)  // do not react to any messages
  {
      return;
  }
  moveit_msgs::CollisionObject collision_object;
  std::string collision_object_id;
  for(uint i = 0; i < msg.markers.size(); i++) {
    if(!markerMsgToCollisionObjectMsg(msg.markers[i], "/odom_combined", collision_object, &collision_object_transform_listener_)) {
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
      // collision_object.operation = collision_object.MOVE; // move object when it is known
      collision_object.operation = collision_object.ADD;  // temporary fix to avoid error messages due to shapes in the CollisionObject msg
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
  pci_collision_objects_.erase(std::remove(pci_collision_objects_.begin(), pci_collision_objects_.end(), object_id), pci_collision_objects_.end()); // removing object from known objects
}

void MoveitConnector::enable_updates(){
  update_enabled_ = true;
  // do this properly
  psi_->removeCollisionObjects(pci_collision_objects_); // deletes every object which might be in the planning scene, including the table.
  ROS_INFO("Planning scene updates enabled.");
}

void MoveitConnector::disable_updates(){
  update_enabled_ = false;  // disabling the addition of new objects to the planning scene
  for (auto& timer : collision_object_remove_timers_) {  // stopping the collision object remove timers
      timer.second.stop();
  }
  ROS_INFO("Planning scene updates disabled.");
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "MoveitConnector");

  MoveitConnector moveit_connector;

  ros::spin();
  return 0;
};
