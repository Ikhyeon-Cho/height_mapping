#include "height_mapping_ros/HeightMapGeneratorNode.h"
#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "height_map_generator_node");
  HeightMapGeneratorNode node;
  ros::spin();
  return 0;
} 