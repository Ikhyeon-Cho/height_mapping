/*
 * global_mapping_node.cpp
 *
 *  Created on: Mar 20, 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include <ros/ros.h>
#include "height_mapping/GlobalMapping.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "global_mapping");
  ros::NodeHandle nh;

  std::string point_type{ nh.param<std::string>("sensor_processor/pointType", "laser") };  // By default, laser
                                                                                           // pointcloud is used
  if (point_type == "color")
  {
    GlobalMapping<pcl::PointXYZRGB> node;
    ros::spin();
  }
  else if (point_type == "laser")
  {
    GlobalMapping<pcl::PointXYZI> node;
    ros::spin();
  }
  else
  {
    std::cout << "\033[33m[HeightMapping::GlobalMapping]: Invalid Point Type!! Supported point types: color, "
                 "laser\033[0m"
              << std::endl;
    return -1;
  }

  return 0;
}