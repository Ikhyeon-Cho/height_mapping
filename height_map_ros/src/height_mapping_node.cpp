/*
 * height_mapping_node.cpp
 *
 *  Created on: Aug 17, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include <ros/ros.h>
#include "height_map_ros/HeightMapping.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "height_mapping");
  ros::NodeHandle nh("~");

  HeightMapping height_mapping_node;

  ros::spin();

  return 0;
}