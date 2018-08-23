
#include <apriltag_object_detection/moveit_connector.h>

bool markerMsgToCollisionObjectMsg(
    visualization_msgs::Marker marker, std::string frame_id,
    moveit_msgs::CollisionObject &collision_object) {
  moveit_msgs::CollisionObject result;

  // header
  result.header = marker.header;
  result.header.frame_id = frame_id;

  // id
  result.id = marker.id;

  // primitive
  shape_msgs::SolidPrimitive primitive;

  if (marker.type == visualization_msgs::Marker::CUBE) {
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = marker.scale.x;
    primitive.dimensions[1] = marker.scale.y;
    primitive.dimensions[2] = marker.scale.z;
  } else {
    ROS_ERROR("Marker type not supported");
    return false;
  }

  result.primitives.push_back(primitive);

  // primitive pose
  tf::StampedTransform marker_transform;
  tf::Transform object_transform;

  tf::TransformListener listener;
  try {
    listener.waitForTransform(frame_id, marker.header.frame_id, ros::Time(0),
                              ros::Duration(5.0));
    listener.lookupTransform(frame_id, marker.header.frame_id, ros::Time(0),
                             marker_transform);
  } catch (tf::TransformException ex) {
    ROS_ERROR("%s", ex.what());
    return false;
  }

  tf::poseMsgToTF(marker.pose, object_transform);

  geometry_msgs::Pose pose;
  tf::poseTFToMsg(marker_transform * object_transform, pose);

  result.primitive_poses.push_back(pose);

  // operation
  result.operation = moveit_msgs::CollisionObject::ADD;

  collision_object = result;

  return true;
}

MoveitConnector::MoveitConnector() : nh_() {
  psi_ = new moveit::planning_interface::PlanningSceneInterface();

  object_sub_ = nh_.subscribe(
      "apriltag_objects", 1, &MoveitConnector::objectsCallback, this);
}

void MoveitConnector::objectsCallback(const visualization_msgs::MarkerArray &msg){
  moveit_msgs::CollisionObject collision_object;
  for(int i = 0; i < msg.markers.size(); i++) {
    // TODO: do not update attached objects
    if(!markerMsgToCollisionObjectMsg(msg.markers[i], "/odom_combined", collision_object)) {
        ROS_ERROR_STREAM("Marker could not get transformed to a CollisionObject!");
        continue;
    }
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
