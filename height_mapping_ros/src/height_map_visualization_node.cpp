/*
 * height_map_visualization_node.cpp
 *
 *  Created on: Feb 12, 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include <ros/ros.h>
#include "height_mapping_ros/HeightMapVisualization.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "height_map_visualization");

  HeightMapVisualization height_map_visualization_node;

  ros::spin();

  return 0;
}