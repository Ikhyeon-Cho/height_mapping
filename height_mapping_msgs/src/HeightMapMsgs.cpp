/*
 * HeightMapMsgs.cpp
 *
 *  Created on: Dec 2, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "height_mapping_msgs/HeightMapMsgs.h"

void HeightMapMsgs::toMapRegion(const grid_map::HeightMap &map,
                                visualization_msgs::Marker &msg) {
  msg.ns = "height_map";
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

  float length_x_half = (map.getLength().x() - 0.5 * map.getResolution()) / 2.0;
  float length_y_half = (map.getLength().y() - 0.5 * map.getResolution()) / 2.0;

  msg.points.resize(5);
  msg.points[0].x = map.getPosition().x() + length_x_half;
  msg.points[0].y = map.getPosition().y() + length_x_half;
  msg.points[0].z = 0;

  msg.points[1].x = map.getPosition().x() + length_x_half;
  msg.points[1].y = map.getPosition().y() - length_x_half;
  msg.points[1].z = 0;

  msg.points[2].x = map.getPosition().x() - length_x_half;
  msg.points[2].y = map.getPosition().y() - length_x_half;
  msg.points[2].z = 0;

  msg.points[3].x = map.getPosition().x() - length_x_half;
  msg.points[3].y = map.getPosition().y() + length_x_half;
  msg.points[3].z = 0;

  msg.points[4] = msg.points[0];
}

void HeightMapMsgs::toOccupancyGrid(const grid_map::HeightMap &map,
                                    nav_msgs::OccupancyGrid &msg) {
  const auto &elevation_grid = map[map.getHeightLayer()];

  // NAN processing for finding min and max
  auto fill_NAN_with_min = elevation_grid.array().isNaN().select(
      std::numeric_limits<float>::lowest(), elevation_grid);
  auto fill_NAN_with_max = elevation_grid.array().isNaN().select(
      std::numeric_limits<float>::max(), elevation_grid);

  // Find min and max
  float min = fill_NAN_with_max.minCoeff();
  float max = fill_NAN_with_min.maxCoeff();

  grid_map::GridMapRosConverter::toOccupancyGrid(map, map.getHeightLayer(), min,
                                                 max, msg);
}