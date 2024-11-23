/*
 * height_mapping_main.cpp
 *
 *  Created on: Aug 17, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include <ros/ros.h>
#include "height_mapping_ros/HeightMappingNode.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "height_mapping");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  HeightMappingNode node(nh, pnh);

  ros::spin();

  return 0;
}