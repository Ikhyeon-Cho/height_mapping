/*
 * ElevationMapRosConverter.cpp
 *
 *  Created on: Dec 2, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "terrain_mapping/ElevationMapRosConverter.h"

void ElevationMapRosConverter::toMapBoundaryMsg(const ElevationMap& map, visualization_msgs::Marker& msg)
{
  msg.ns = "elevation_map";
  msg.lifetime = ros::Duration();
  msg.action = visualization_msgs::Marker::ADD;
  msg.type = visualization_msgs::Marker::LINE_STRIP;

  msg.scale.x = 0.1;
  msg.color.a = 1.0;
  msg.color.r = 0.0;
  msg.color.g = 1.0;
  msg.color.b = 0.0;

  msg.header.frame_id = map.getFrameId();
  msg.header.stamp = ros::Time::now();

  float length_x_half = map.getLength().x() / 2.0;
  float length_y_half = map.getLength().y() / 2.0;

  msg.points.resize(5);
  msg.points[0].x = map.getPosition().x() + length_x_half;
  msg.points[0].y = map.getPosition().y() + length_x_half;

  msg.points[1].x = map.getPosition().x() + length_x_half;
  msg.points[1].y = map.getPosition().y() - length_x_half;

  msg.points[2].x = map.getPosition().x() - length_x_half;
  msg.points[2].y = map.getPosition().y() - length_x_half;

  msg.points[3].x = map.getPosition().x() - length_x_half;
  msg.points[3].y = map.getPosition().y() + length_x_half;

  msg.points[4] = msg.points[0];
}

void ElevationMapRosConverter::toOccupancyGridMsg(const ElevationMap& map, nav_msgs::OccupancyGrid& msg)
{
  const auto& elevation_layer = map.getElevationLayer();
  auto fill_NAN_with_min = elevation_layer.array().isNaN().select(std::numeric_limits<float>::lowest(), elevation_layer);
  auto fill_NAN_with_max = elevation_layer.array().isNaN().select(std::numeric_limits<float>::max(), elevation_layer);

  float min = fill_NAN_with_max.minCoeff();
  float max = fill_NAN_with_min.maxCoeff();

  grid_map::GridMapRosConverter::toOccupancyGrid(map, "elevation", min, max, msg);
}