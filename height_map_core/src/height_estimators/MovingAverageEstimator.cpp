/*
 * MovingAverageEstimator.cpp
 *
 *  Created on: Apr 2, 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "height_map_core/height_estimators/MovingAverageEstimator.h"

namespace height_map
{
void MovingAverageEstimator::estimate(grid_map::HeightMap& map, const pcl::PointCloud<pcl::PointXYZ>& cloud)
{
  if (hasEmptyCloud(cloud))
    return;

  if (cloud.header.frame_id != map.getFrameId())
  {
    std::cout << "[HeightEstimator]: Frame ID mismatch - pointcloud is in a different frame! \n";
    return;
  }

  auto& height_matrix = map.getHeightMatrix();

  grid_map::Index cell_index;
  grid_map::Position cell_position;
  for (const auto& point : cloud)
  {
    // Skip if the point is out of the map
    cell_position << point.x, point.y;
    if (!map.getIndex(cell_position, cell_index))
      continue;

    auto& height = height_matrix(cell_index(0), cell_index(1));

    // Initialize the height and variance if it is NaN
    if (map.isEmptyAt(cell_index))
    {
      height = point.z;
      continue;
    }

    ExponentialWeightedMovingAverageUpdate(height, point.z);
  }
}

void MovingAverageEstimator::estimate(grid_map::HeightMap& map, const pcl::PointCloud<pcl::PointXYZI>& cloud)
{
  if (hasEmptyCloud(cloud))
    return;

  if (cloud.header.frame_id != map.getFrameId())
  {
    std::cout << "[HeightEstimator]: Frame ID mismatch - pointcloud is in a different frame! \n";
    return;
  }

  map.addLayer("intensity");

  auto& height_matrix = map.getHeightMatrix();
  auto& intensity_matrix = map["intensity"];

  grid_map::Index cell_index;
  grid_map::Position cell_position;
  for (const auto& point : cloud)
  {
    // Skip if the point is out of the map
    cell_position << point.x, point.y;
    if (!map.getIndex(cell_position, cell_index))
      continue;

    auto& height = height_matrix(cell_index(0), cell_index(1));
    auto& intensity = intensity_matrix(cell_index(0), cell_index(1));

    // Initialize the height and variance if it is NaN
    if (map.isEmptyAt(cell_index))
    {
      height = point.z;
      intensity = point.intensity;
      continue;
    }
    ExponentialWeightedMovingAverageUpdate(height, point.z);
    ExponentialWeightedMovingAverageUpdate(intensity, point.intensity);
  }
}

void MovingAverageEstimator::estimate(grid_map::HeightMap& map, const pcl::PointCloud<pcl::PointXYZRGB>& cloud)
{
  if (hasEmptyCloud(cloud))
    return;

  if (cloud.header.frame_id != map.getFrameId())
  {
    std::cout << "[HeightEstimator]: Frame ID mismatch - pointcloud is in a different frame! \n";
    return;
  }

  map.addLayer("r");
  map.addLayer("g");
  map.addLayer("b");
  map.addLayer("color");

  auto& height_matrix = map.getHeightMatrix();
  auto& red_matrix = map["r"];
  auto& green_matrix = map["g"];
  auto& blue_matrix = map["b"];
  auto& color_matrix = map["color"];

  grid_map::Index cell_index;
  grid_map::Position cell_position;
  for (const auto& point : cloud)
  {
    // Skip if the point is out of the map
    cell_position << point.x, point.y;
    if (!map.getIndex(cell_position, cell_index))
      continue;

    auto& height = height_matrix(cell_index(0), cell_index(1));
    auto& red = red_matrix(cell_index(0), cell_index(1));
    auto& green = green_matrix(cell_index(0), cell_index(1));
    auto& blue = blue_matrix(cell_index(0), cell_index(1));
    auto& color = color_matrix(cell_index(0), cell_index(1));

    // Initialize the height and variance if it is NaN
    if (map.isEmptyAt(cell_index))
    {
      height = point.z;
      red = point.r;
      green = point.g;
      blue = point.b;
      grid_map::colorVectorToValue(point.getRGBVector3i(), color);

      continue;
    }

    ExponentialWeightedMovingAverageUpdate(height, point.z);
    ExponentialWeightedMovingAverageUpdate(red, point.r);
    ExponentialWeightedMovingAverageUpdate(green, point.g);
    ExponentialWeightedMovingAverageUpdate(blue, point.b);
    grid_map::colorVectorToValue(Eigen::Vector3i(red, green, blue), color);
  }
}

}  // namespace height_map