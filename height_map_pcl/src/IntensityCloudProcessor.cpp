/*
 * IntensityCloudProcessor.cpp
 *
 *  Created on: Apr 3, 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "height_map_pcl/processors/IntensityCloudProcessor.h"

namespace height_map
{
std::pair<bool, pcl::PointCloud<pcl::PointXYZI>::Ptr> IntensityCloudProcessor::gridDownsampling(
    const pcl::PointCloud<pcl::PointXYZI>& cloud, const grid_map::HeightMap& input_map)
{
  // Check if the cloud is valid
  if (!isEmpty(cloud))
    return { false, nullptr };

  // Initialize the map geometry
  if (!is_map_initialized_)
  {
    map_.setGeometry(input_map.getLength(), input_map.getResolution());
    map_.setFrameId(input_map.getFrameId());
    map_.addLayer("intensity");
    is_map_initialized_ = true;
  }

  // Check if the frame id matches
  if (cloud.header.frame_id != input_map.getFrameId())
  {
    std::cout << "[IntensityCloudProcessor]: Frame ID mismatch - pointcloud is in a different frame! \n";
    return { false, nullptr };
  }

  // reset the map values
  map_.clear(map_.getHeightLayer());
  map_.clear("intensity");

  // update the map position
  map_.move(input_map.getPosition());

  auto& height_matrix = map_.getHeightMatrix();
  auto& intensity_matrix = map_["intensity"];

  // Create a vector to keep track of measured grid indices.
  std::vector<grid_map::Index> measured_index_list;
  grid_map::Index cell_index;
  grid_map::Position cell_position;
  for (const auto& point : cloud)
  {
    // Check whether point is inside the map
    cell_position << point.x, point.y;
    if (!map_.getIndex(cell_position, cell_index))
      continue;

    // First grid height measuerment
    if (map_.isEmptyAt(map_.getHeightLayer(), cell_index))
    {
      height_matrix(cell_index(0), cell_index(1)) = point.z;
      measured_index_list.push_back(cell_index);
    }
    // Drop the existing point if the new point has higher height
    else if (point.z > height_matrix(cell_index(0), cell_index(1)))
    {
      height_matrix(cell_index(0), cell_index(1)) = point.z;
    }
    // Update the intensity value
    intensity_matrix(cell_index(0), cell_index(1)) = point.intensity;

  }  // pointcloud loop ends

  // push back the downsampled points that were saved in the grid
  auto cloud_downsampled = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  cloud_downsampled->header = cloud.header;
  cloud_downsampled->reserve(measured_index_list.size());

  grid_map::Position3 cell_position3;
  pcl::PointXYZI point;
  for (const auto& cell_index : measured_index_list)
  {
    map_.getPosition3(map_.getHeightLayer(), cell_index, cell_position3);

    point.x = cell_position3.x();
    point.y = cell_position3.y();
    point.z = cell_position3.z();
    point.intensity = intensity_matrix(cell_index(0), cell_index(1));

    cloud_downsampled->push_back(point);
  }

  return { true, cloud_downsampled };
}

std::pair<bool, pcl::PointCloud<pcl::PointXYZI>::Ptr> IntensityCloudProcessor::removeInconsistentPoints(
    const pcl::PointCloud<pcl::PointXYZI>& cloud_input, const grid_map::HeightMap& map)
{
  if (!isEmpty(cloud_input))
    return { false, nullptr };

  // Check if the frame id matches
  if (cloud_input.header.frame_id != map.getFrameId())
  {
    std::cout << "[IntensityCloudProcessor]: Frame ID mismatch - pointcloud is in a different frame! \n";
    return { false, nullptr };
  }

  auto cloud_consistent = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  cloud_consistent->header = cloud_input.header;
  cloud_consistent->reserve(cloud_input.size());

  const auto& height_matrix = map.getHeightMatrix();

  grid_map::Index cell_index;
  grid_map::Position cell_position;
  for (const auto& point : cloud_input)
  {
    // Check whether point is inside the map
    cell_position << point.x, point.y;
    if (!map.getIndex(cell_position, cell_index))
      continue;

    bool has_valid_height = std::isfinite(height_matrix(cell_index(0), cell_index(1)));
    if (has_valid_height)
    {
      auto height_diff = point.z - height_matrix(cell_index(0), cell_index(1));
      bool inconsistent = height_diff > 0.5;
      if (inconsistent)
        continue;
    }
    cloud_consistent->push_back(point);
  }

  return { true, cloud_consistent };
}
}  // namespace height_map