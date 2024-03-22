/*
 * height_map_loader_node.cpp
 *
 *  Created on: Mar 21, 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include <ros/ros.h>
#include "heightmap_loader/HeightMapLoader.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "heightmap_loader");

  HeightMapLoader heightmap_loader_node;

  ros::spin();

  return 0;
}