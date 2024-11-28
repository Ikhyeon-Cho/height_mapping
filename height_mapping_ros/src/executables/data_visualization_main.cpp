#include "height_mapping_ros/DataVisualizationNode.h"
#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "data_visualization_node");
  DataVisualizationNode node;
  ros::spin();
  return 0;
} 