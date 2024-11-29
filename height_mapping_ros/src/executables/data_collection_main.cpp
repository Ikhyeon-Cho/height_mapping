#include "height_mapping_ros/DataCollectionNode.h"
#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "data_collection_node");
  DataCollectionNode node;

  ros::spin();

  return 0;
}
