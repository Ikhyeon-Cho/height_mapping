/*
 *  HeightMapMsgs.h
 *  Created on: Dec 2, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAP_MSGS_H
#define HEIGHT_MAP_MSGS_H

#include <height_map_core/HeightMap.h>

#include <grid_map_ros/GridMapRosConverter.hpp>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>

class HeightMapMsgs
{
public:
  static void toMapBoundary(const grid_map::GridMap& map, visualization_msgs::Marker& msg);

  static void toOccupancyGrid(const grid_map::GridMap& map, nav_msgs::OccupancyGrid& msg);
};

#endif  // HEIGHT_MAP_MSGS_H