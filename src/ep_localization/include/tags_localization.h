//
// Created by qiayuan on 6/18/20.
//

#ifndef SRC_EP_ROS_SRC_EP_LOCALIZATION_INCLUDE_TAGS_LOCALIZATION_H_
#define SRC_EP_ROS_SRC_EP_LOCALIZATION_INCLUDE_TAGS_LOCALIZATION_H_

#include <apriltag_ros/AprilTagDetectionArray.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <string>

template<typename T, int filter_size>
class MovingAverage {
 private:
  T sum = 0;

  T buffer[filter_size];
  int pointer = 0;
  int size = 0;

 public:
  MovingAverage() { memset(buffer, 0, sizeof(T) * filter_size); }
  T filter(T val) {
    //push
    sum = sum - buffer[pointer] + val;
    buffer[pointer] = val;
    pointer = (pointer + 1) % filter_size;

    if (size < filter_size) size++;

    return sum / size;
  }
};

class TagsLocalization {
 public:
  TagsLocalization();
  void FuseDetectedTags(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg);
  void tagCB(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg);

 private:
  tf::Transform map2odom_;
  std::string camera_frame_;

  unsigned int tags_buffer_size_{};
  MovingAverage<double, 5> filter_x, filter_y, filter_yaw_x, filter_yaw_y;

  ros::Subscriber tag_sub_;
  tf::TransformListener tf_listener_;
  tf::TransformBroadcaster tf_broadcaster_;
};

inline double yawFromQuaternion(const tf::Quaternion &quat) {
  tf::Matrix3x3 m_orient(quat);

  double roll, pitch, yaw;
  m_orient.getRPY(roll, pitch, yaw);

  return yaw;
}

inline std::string to_string(int x) {
  std::stringstream ss;
  ss << x;
  return ss.str();
}

#endif //SRC_EP_ROS_SRC_EP_LOCALIZATION_INCLUDE_TAGS_LOCALIZATION_H_
