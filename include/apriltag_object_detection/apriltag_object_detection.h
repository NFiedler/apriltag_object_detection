#ifndef APRILTAG_OBJECT_DETECTION
#define APRILTAG_OBJECT_DETECTION

#include <apriltags2_ros/AprilTagDetectionArray.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

class ApriltagObjectDetection {
public:
  ApriltagObjectDetection();
  ~ApriltagObjectDetection(){};

  void parseYaml();
  void aprilTagsCallback(const apriltags2_ros::AprilTagDetectionArray &msg);

private:
  ros::NodeHandle nh_;

  ros::Subscriber apriltag_sub_;
  ros::Publisher marker_pub_;

  visualization_msgs::MarkerArray marker_array_;
};

#endif
