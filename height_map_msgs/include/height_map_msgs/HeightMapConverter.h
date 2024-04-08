/*
 *  HeightMapConverter.h
 *  Created on: Mar 21, 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAP_CONVERTER_H
#define HEIGHT_MAP_CONVERTER_H

#include <height_map_core/HeightMap.h>
#include <grid_map_cv/GridMapCvConverter.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <sensor_msgs/point_cloud2_iterator.h>

class HeightMapConverter
{
  const static int VALID_PIXEL_MIN = 100;  // lower than this value is an invalid pixel (unexplored region)
  const static int VALID_PIXEL_MAX = 255;

public:
  static bool fromGrayImage(const cv::Mat& img, float min_val, float max_val, grid_map::HeightMap& map,
                            const std::string& layer);

  static bool toGrayImage(const grid_map::HeightMap& map, const std::string& layer, cv::Mat& img, float min_value,
                          float max_value);

  static void fromPointCloud2(const sensor_msgs::PointCloud2& cloud, grid_map::HeightMap& map);

  static void toPointCloud2(const grid_map::HeightMap& map, sensor_msgs::PointCloud2& cloud);

  static void toPointCloud2(const grid_map::HeightMap& map, const std::vector<std::string>& layers,
                           sensor_msgs::PointCloud2& cloud);

  static void toPointCloud2(const grid_map::HeightMap& map, const std::vector<std::string>& layers,
                           const std::vector<grid_map::Index>& measured_indices, sensor_msgs::PointCloud2& cloud);
};

// Helper functions
void unpackRgb(uint32_t rgb_packed, uint8_t& r, uint8_t& g, uint8_t& b);

#endif  // HEIGHT_MAP_CONVERTER_H