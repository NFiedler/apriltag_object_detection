#include <apriltag_object_detection/marker_to_collision_object.h>


bool markerMsgToCollisionObjectMsg(
    visualization_msgs::Marker marker,
    moveit_msgs::CollisionObject &collision_object,
    tf::TransformListener *listener) {
  return markerMsgToCollisionObjectMsg(marker, "world", collision_object, listener);
}

bool markerMsgToCollisionObjectMsg(
    visualization_msgs::Marker marker, std::string frame_id,
    moveit_msgs::CollisionObject &collision_object,
    tf::TransformListener *listener) {
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

  try {
    listener->waitForTransform(frame_id, marker.header.frame_id, marker.header.stamp,
                              ros::Duration(0.5));
    listener->lookupTransform(frame_id, marker.header.frame_id, marker.header.stamp,
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
