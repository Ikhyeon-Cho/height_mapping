/*
 * height_map_saver_node.cpp
 *
 *  Created on: Mar 22, 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "heightmap_saver/HeightMapSaver.h"
#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "heightmap_saver");

  HeightMapSaver heightmap_saver_node;

  ros::spin();

  return 0;
}