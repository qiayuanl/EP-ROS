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
  tf::StampedTransform cam2base;
  try {
    tf_listener_.lookupTransform(camera_frame_, "base_link", ros::Time(0), cam2base);
  }
  catch (tf::TransformException &ex) {
    ROS_ERROR("%s", ex.what());
  }

  //Loop all detections
  for (const auto &detection : msg->detections) {
    unsigned long tag_id;
    tf::Vector3 cam2tag_v;
    tf::Quaternion cam2tag_q;

    tag_id = static_cast<unsigned long>(detection.id[0]);
    tf::pointMsgToTF(detection.pose.pose.pose.position, cam2tag_v);
    tf::quaternionMsgToTF(detection.pose.pose.pose.orientation, cam2tag_q);
    tf::Transform cam2tag;
    cam2tag.setOrigin(cam2tag_v);
    cam2tag.setRotation(cam2tag_q);
    //Send current tag location
    //
    tf_broadcaster_.sendTransform(tf::StampedTransform(cam2tag.inverse(),
                                                       ros::Time::now(),
                                                       "tag_" + to_string((int) tag_id),
                                                       "cam"));

    tf::Transform tag2base;
    tag2base = cam2tag.inverse() * cam2base;

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
    if (std::abs(cur_base_z) < 0.2) {
      //Calc Weight
      double cur_Weight = 1 / cam2tag_v.length();

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
      filter_y.filter(loc_y / loc_weight_sum),
      0
  ));
  map2base.setRotation(tf::createQuaternionFromYaw(atan2(
      filter_yaw_y.filter(loc_yaw_y / loc_weight_sum),
      filter_yaw_x.filter(loc_yaw_x / loc_weight_sum)
  )));

  //Send current tag location
  tf_broadcaster_.sendTransform(tf::StampedTransform(map2base,
                                                     ros::Time::now(),
                                                     "map",
                                                     "fused_base"));
  //get base -> odom
  tf::StampedTransform base2odom;
  try {
    tf_listener_.lookupTransform("base_link", "odom", ros::Time(0), base2odom);
  }
  catch (tf::TransformException &ex) {
    ROS_ERROR("%s", ex.what());
  }

  //set map -> odom
  map2odom_ = map2base * base2odom;

  tf_broadcaster_.sendTransform(tf::StampedTransform(map2odom_, ros::Time::now(), "map", "odom"));
}

void TagsLocalization::tagCB(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg) {
  FuseDetectedTags(msg);
}