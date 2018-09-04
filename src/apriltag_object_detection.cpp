#include <apriltag_object_detection/apriltag_object_detection.h>

#include <tf/transform_datatypes.h>

#include <XmlRpcException.h>
#include <XmlRpcValue.h>

ApriltagObjectDetection::ApriltagObjectDetection() : nh_() {
  parseYaml();
  apriltag_sub_ = nh_.subscribe(
      "tag_detections", 1, &ApriltagObjectDetection::aprilTagsCallback, this);
  marker_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>("apriltag_objects", 0);
}

void ApriltagObjectDetection::parseYaml() {
  ros::NodeHandle ph("~");

  XmlRpc::XmlRpcValue objects;
  ph.getParam("objects", objects);

  visualization_msgs::Marker marker;

  marker.action = visualization_msgs::Marker::ADD;
  marker.lifetime = ros::Duration(0.2);
  marker.pose.orientation.w = 1;
  marker.color.a = 0.8;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;

  try {
    XmlRpc::XmlRpcValue &val = objects;

    for (int i = 0; i < objects.size(); i++) {
      // Id
      marker.id = objects[i]["object"]["tag_id"];

      // parent frame
      std::string marker_name = objects[i]["object"]["name"];
      marker.header.frame_id = marker_name;

      // Object type
      std::string marker_type = objects[i]["object"]["type"];
      if (marker_type == "CUBE")
        marker.type = visualization_msgs::Marker::CUBE;
      else if (marker_type == "SPHERE")
        marker.type = visualization_msgs::Marker::SPHERE;
      else if (marker_type == "CYLINDER")
        marker.type = visualization_msgs::Marker::CYLINDER;
      else if (marker_type == "MESH") {
        marker.type = visualization_msgs::Marker::MESH_RESOURCE;
        if (val["mesh"].valid())
          marker.mesh_resource = static_cast<std::string>(val["mesh"]);
        else {
          ROS_WARN_STREAM("Incomplete definition of pushable object "
                          << marker.id << ".");
          ROS_WARN_STREAM(
              "Object type is MESH but 'mesh_resource' is not specified!");
        }
      } else {
        ROS_WARN_STREAM("Found unknown marker type '"
                        << marker_type << "' in definition of object "
                        << marker.id);
      }

      // Tag offset
      geometry_msgs::Pose tag_offset;
      marker.pose.position.x = objects[i]["object"]["origin"][0];
      marker.pose.position.y = objects[i]["object"]["origin"][1];
      marker.pose.position.z = objects[i]["object"]["origin"][2];

      marker.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(
          objects[i]["object"]["rpy"][0], objects[i]["object"]["rpy"][1],
          objects[i]["object"]["rpy"][2]);

      // Scale
      marker.scale.x = objects[i]["object"]["scale"]["x"];
      marker.scale.y = objects[i]["object"]["scale"]["y"];
      marker.scale.z = objects[i]["object"]["scale"]["z"];

      marker_array_.markers.push_back(marker);
    }
  } catch (XmlRpc::XmlRpcException &e) {
    ROS_WARN("%s", e.getMessage().c_str());
  }
}

void ApriltagObjectDetection::aprilTagsCallback(
    const apriltags2_ros::AprilTagDetectionArray &msg) {
  visualization_msgs::MarkerArray marker_array;

  for (int i = 0; i < msg.detections.size(); i++) {
    for (int j = 0; j < marker_array_.markers.size(); j++) {
      if (msg.detections[i].id[0] == marker_array_.markers[j].id) {
        marker_array_.markers[j].header.stamp = ros::Time::now();
        marker_array.markers.push_back(marker_array_.markers[j]);
      }
    }
  }
  marker_pub_.publish(marker_array);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "ApriltagObjectDetection");

  ApriltagObjectDetection apriltag_object_detection;

  ros::spin();
  return 0;
};
