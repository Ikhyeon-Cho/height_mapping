/*
 * ElevationMapRosConverter.h
 *
 *  Created on: Dec 2, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef ELEVATION_MAP_ROS_CONVERTER_H
#define ELEVATION_MAP_ROS_CONVERTER_H

#include "terrain_mapping/ElevationMap.h"
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>

class ElevationMapRosConverter
{
public:
  static void toMapBoundaryMsg(const ElevationMap& map, visualization_msgs::Marker& msg); 

  static void toOccupancyGridMsg(const ElevationMap& map, nav_msgs::OccupancyGrid& msg);
};

#endif