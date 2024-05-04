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
  ros::init(argc, argv, "height_mapping_global");

  GlobalHeightMapping node;

  ros::spin();

  return 0;
}