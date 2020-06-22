//
// Created by qiayuan on 6/18/20.
//

#include "tags_localization.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "tags_localization");

  ros::NodeHandle nh;

  //Create Nodes
  TagsLocalization tags_localization;

  ros::spin();

  //Release Nodes
  return 0;
}