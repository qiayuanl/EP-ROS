//
// Created by qiayuan on 6/18/20.
//
#include "tags_localization.h"

TagsLocalization::TagsLocalization() {
  ros::NodeHandle nh_private;
  tag_sub_ = nh_private.subscribe
      <apriltag_ros::AprilTagDetectionArray>("tag_detections",
                                             100,
                                             &TagsLocalization::tagCB,
                                             this);
  nh_private.param<std::string>("camera_frame", camera_frame_, "ep_camera_frame");
  nh_private.param<std::int32_t>("tags_buff", reinterpret_cast<int &>(tags_buffer_size_), 42);

}

void TagsLocalization::FuseDetectedTags(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg) {
  //If no detection
  if (msg->detections.empty()) return;

  double loc_x = 0;
  double loc_y = 0;
  double loc_yaw_x = 0;
  double loc_yaw_y = 0;
  double loc_weight_sum = 0;

  //Lookup Base -> Cam Transform
  tf::StampedTransform base2cam;
  try {
    tf_listener_.lookupTransform("base_link", camera_frame_, ros::Time(0), base2cam);
  }
  catch (tf::TransformException &ex) {
    ROS_ERROR("%s", ex.what());
  }

  //Loop all detections
  for (const auto &detection : msg->detections) {
    unsigned long tag_id;
    tf::Vector3 tag2cam_v;
    tf::Quaternion tag2cam_q;

    tag_id = static_cast<unsigned long>(detection.id[0]);
    tf::pointMsgToTF(detection.pose.pose.pose.position, tag2cam_v);
    tf::quaternionMsgToTF(detection.pose.pose.pose.orientation, tag2cam_q);

    //Transform Coordinate
    tf::Vector3 base2cam_v = base2cam.getOrigin();
    tf::Matrix3x3 base2cam_r(base2cam.getRotation());
    tf::Matrix3x3 tag2cam_r(tag2cam_q);
    tf::Matrix3x3 tag2base_r = tag2cam_r * (base2cam_r.transpose());
    tf::Vector3 tag2base_v = tag2cam_v - tag2base_r * base2cam_v;

    //Tag To Base
    tf::Quaternion tag2base_q;
    tf::Transform tag2base;

    tag2base_r.getRotation(tag2base_q);
    tag2base.setRotation(tag2base_q);
    tag2base.setOrigin(tag2base_v);

    //Lookup map -> base Transform
    tf::StampedTransform map2tag;
    try {
      tf_listener_.lookupTransform("map", "tag_" + to_string((int) tag_id),
                                   ros::Time(0), map2tag);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s", ex.what());
    }

    //Base to map
    tf::Transform map2base;
    map2base = map2tag * tag2base;

    //Send current tag location
    tf_broadcaster_.sendTransform(tf::StampedTransform(map2base,
                                                       ros::Time::now(),
                                                       "map",
                                                       "not_fuse_base" + to_string((int) tag_id)));

    double cur_base_x = map2base.getOrigin().getX();
    double cur_base_y = map2base.getOrigin().getY();
    double cur_base_z = map2base.getOrigin().getZ();
    double cur_base_yaw = yawFromQuaternion(map2base.getRotation());

    //Ignore Flying base (>0.2m)
    if (std::abs(cur_base_z) < 0.5) {
      //Calc Weight
      double cur_Weight = 1 / tag2cam_v.length();

      //Sum Up Position Estimate
      loc_x += cur_Weight * cur_base_x;
      loc_y += cur_Weight * cur_base_y;
      loc_yaw_x += cur_Weight * cos(cur_base_yaw);
      loc_yaw_y += cur_Weight * sin(cur_base_yaw);
      loc_weight_sum += cur_Weight;
    }
  }

  if (fabs(loc_weight_sum) < 1e-8) return;

  //get weighted pose (map -> base)
  tf::Transform map2base;
  map2base.setOrigin(tf::Vector3(
      filter_x.filter(loc_x / loc_weight_sum),
      filter_y.filter(loc_x / loc_weight_sum),
      0
  ));
  map2base.setRotation(tf::createQuaternionFromYaw(atan2(
      filter_yaw_y.filter(loc_yaw_y / loc_weight_sum),
      filter_yaw_x.filter(loc_yaw_x / loc_weight_sum)
  )));

  //get base -> odom
  tf::StampedTransform base_odom_tf;
  try {
    tf_listener_.lookupTransform("base_link", "odom", ros::Time(0), base_odom_tf);
  }
  catch (tf::TransformException &ex) {
    ROS_ERROR("%s", ex.what());
  }

  //set map -> odom
  map2odom_ = map2base * base_odom_tf;

  tf_broadcaster_.sendTransform(tf::StampedTransform(map2odom_, ros::Time::now(), "map", "odom"));
}

void TagsLocalization::tagCB(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg) {
  FuseDetectedTags(msg);
}