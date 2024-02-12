/*
 * HeightMap.cpp
 *
 *  Created on: Aug 17, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "height_map_core/HeightMap.h"

namespace grid_map
{
HeightMap::HeightMap(double length_x, double length_y, double grid_resolution)
  : GridMap({ "elevation", "variance", "n_point", "sample_mean", "sample_variance", "n_sample" })
{
  setFrameId("map");
  setGeometry(grid_map::Length(length_x, length_y), grid_resolution);
  setBasicLayers({ "elevation", "variance" });

  add("max_height");
  add("point_variance");
}

HeightMap::HeightMap() : HeightMap(20, 20, 0.1)
{
}

HeightMap::HeightMap(const std::vector<std::string>& layers) : grid_map::GridMap(layers)
{
}

void HeightMap::update(const pcl::PointCloud<pcl::PointXYZI>& pointcloud)
{
  if (pointcloud.header.frame_id != getFrameId())
  {
    std::cout << " [HeightMap] Warning: Frame ID mismatch - pointcloud is in a different frame! \n";
    return;
  }
  if (pointcloud.empty())
  {
    std::cout << " [HeightMap] Warning: Skipping map update - point cloud is empty! \n";
    return;
  }

  // 1. Remove duplicated points in grid
  auto downsampled_cloudPtr = getDownsampledCloudAtGrid(pointcloud);

  // 2. update grid cell height estimates
  updateElevation(*downsampled_cloudPtr);

  // 3. height sample variance : for visualization
  updateSampleVariance(*downsampled_cloudPtr);
}

pcl::PointCloud<pcl::PointXYZI>::Ptr HeightMap::getDownsampledCloudAtGrid(const pcl::PointCloud<PointXYZR>& pointcloud)
{
  const std::string maxHeight_layer("max_height");
  const std::string pointVariance_layer("point_variance");

  // Note: Add() function and erase() function makes this function slow. Be careful!
  clear(maxHeight_layer);
  clear(pointVariance_layer);

  auto& maxHeight_layer_data = get(maxHeight_layer);
  auto& pointVariance_layer_data = get(pointVariance_layer);

  // Create a set to keep track of unique grid indices.
  std::vector<grid_map::Index> measured_index_list;
  grid_map::Index index;
  for (const auto& point : pointcloud)
  {
    // Check whether point is inside the map
    if (!getIndex(grid_map::Position(point.x, point.y), index))
      continue;

    // First grid height measuerment
    if (isEmptyAt(maxHeight_layer, index))
    {
      maxHeight_layer_data(index(0), index(1)) = point.z;
      pointVariance_layer_data(index(0), index(1)) = point.intensity;
      measured_index_list.push_back(index);
    }
    else if (point.z > maxHeight_layer_data(index(0), index(1)))
    {
      maxHeight_layer_data(index(0), index(1)) = point.z;
      pointVariance_layer_data(index(0), index(1)) = point.intensity;
    }
  }  // pointcloud loop ends

  // Get overrided point height (per grid) >> saved in downsampled_cloud
  pcl::PointCloud<PointXYZR>::Ptr downsampled_cloud = boost::make_shared<pcl::PointCloud<PointXYZR>>();
  downsampled_cloud->header = pointcloud.header;
  downsampled_cloud->reserve(measured_index_list.size());

  // declare outside of for loop : for faster speed
  // Eigen constructors are pretty slow
  grid_map::Position3 grid_position3D;
  PointXYZR point;
  for (const auto& index : measured_index_list)
  {
    getPosition3(maxHeight_layer, index, grid_position3D);

    point.x = grid_position3D.x();
    point.y = grid_position3D.y();
    point.z = grid_position3D.z();
    point.intensity = pointVariance_layer_data(index(0), index(1));

    downsampled_cloud->push_back(point);
  }

  return downsampled_cloud;
}

void HeightMap::updateElevation(const pcl::PointCloud<PointXYZR>& pointcloud)
{
  const std::string elevation_layer("elevation");
  const std::string variance_layer("variance");
  const std::string nPoint_layer("n_point");
  auto& elevation_layer_data = getElevationLayer();
  auto& variance_layer_data = getVarianceLayer();
  auto& nPoint_layer_data = getNumMeasuredPointsLayer();

  grid_map::Index index;
  for (const auto& point : pointcloud)
  {
    const auto& point_variance = point.intensity;

    // Check whether point is inside the map
    if (!getIndex(grid_map::Position(point.x, point.y), index))
      continue;

    // First grid height measuerment
    if (isEmptyAt(index))
    {
      elevation_layer_data(index(0), index(1)) = point.z;         // at(elevation_layer, index) = point.z
      variance_layer_data(index(0), index(1)) = point.intensity;  // at(variance_layer, index) = point.intensity
      nPoint_layer_data(index(0), index(1)) = 1;                  // at(nPoint_layer, index) = 1
      continue;
    }

    auto& elevation = elevation_layer_data(index(0), index(1));
    auto& variance = variance_layer_data(index(0), index(1));
    auto& n_point = nPoint_layer_data(index(0), index(1));

    // reject dynamic obstacle >> TODO: use mahalanobis distance
    if (std::abs(elevation - point.z) > 0.2)
      continue;

    // univariate Kalman measurement update
    elevation = (elevation * point_variance + point.z * variance) / (variance + point_variance);
    variance = variance * point_variance / (variance + point_variance);
    n_point += 1;
  }
}

// Later Can be moved inside of downsampling function for efficiency.
// Currently implemented with seperate function for modularization
void HeightMap::updateSampleVariance(const pcl::PointCloud<PointXYZR>& pointcloud)
{
  auto& sampleMean_layer_data = get("sample_mean");
  auto& sampleVariance_layer_data = get("sample_variance");
  auto& numSample_layer_data = get("n_sample");

  grid_map::Index index;
  for (const auto& point : pointcloud)
  {
    // Check whether point is inside the map
    if (!getIndex(grid_map::Position(point.x, point.y), index))
      continue;

    // First grid height measuerment
    if (isEmptyAt("n_sample", index))
    {
      numSample_layer_data(index(0), index(1)) = 1;         // at("n_sample", index) = 1
      sampleMean_layer_data(index(0), index(1)) = point.z;  // at("sample_mean", index) = point.z
      sampleVariance_layer_data(index(0), index(1)) = 0;    // at("sample_variance", index) = 0;
      continue;
    }

    // alias
    auto& n_sample = numSample_layer_data(index(0), index(1));
    auto& sample_maen = sampleMean_layer_data(index(0), index(1));
    auto& sample_variance = sampleVariance_layer_data(index(0), index(1));

    // recursive update of mean and variance:
    // https://math.stackexchange.com/questions/374881/recursive-formula-for-variance
    n_sample += 1;
    auto prev_sample_mean = sample_maen;
    sample_maen = sample_maen + (point.z - sample_maen) / n_sample;
    sample_variance = sample_variance + std::pow(prev_sample_mean, 2) - std::pow(sample_maen, 2) +
                      (std::pow(point.z, 2) - sample_variance - std::pow(prev_sample_mean, 2)) / n_sample;
  }
}

void HeightMap::smoothing()
{
  for (grid_map::GridMapIterator iterator(*this); !iterator.isPastEnd(); ++iterator)
  {
    const auto& thisGrid = *iterator;
    if (!isValid(thisGrid))
      continue;

    grid_map::Position thisGridPosition;
    if (!getPosition(thisGrid, thisGridPosition))
      continue;

    // Smooth only unreliable area
    const auto& unreliable = at("unreliable", thisGrid);
    if (!std::isfinite(unreliable))
      continue;

    int n_sum = 0;
    double elevation_sum = 0;
    for (grid_map::CircleIterator subiter(*this, thisGridPosition, 0.3); !subiter.isPastEnd(); ++subiter)
    {
      grid_map::Position3 nearGridPosition3;
      if (!getPosition3("elevation", *subiter, nearGridPosition3))
        continue;

      if ((nearGridPosition3.head<2>() - thisGridPosition).norm() < 0.05)
        continue;

      elevation_sum += nearGridPosition3.z();
      ++n_sum;
    }
    if (n_sum == 0)
      continue;

    auto& elevation = at("elevation", thisGrid);
    const auto& elevation_bottom = at("height_ground", thisGrid);
    // Smooth only potentially ground cell
    if (elevation - elevation_bottom > 0.15)
      continue;

    elevation = elevation_sum / n_sum;
  }
}

const grid_map::GridMap::Matrix& HeightMap::getElevationLayer() const
{
  return get("elevation");
}

grid_map::GridMap::Matrix& HeightMap::getElevationLayer()
{
  return get("elevation");
}

const grid_map::GridMap::Matrix& HeightMap::getVarianceLayer() const
{
  return get("variance");
}

grid_map::GridMap::Matrix& HeightMap::getVarianceLayer()
{
  return get("variance");
}

const grid_map::GridMap::Matrix& HeightMap::getNumMeasuredPointsLayer() const
{
  return get("n_point");
}

grid_map::GridMap::Matrix& HeightMap::getNumMeasuredPointsLayer()
{
  return get("n_point");
}

bool HeightMap::isEmptyAt(const grid_map::Index& index) const
{
  return !isValid(index);
}

bool HeightMap::isEmptyAt(const std::string& layer, const grid_map::Index& index) const
{
  return !std::isfinite(at(layer, index));
}

// // Note: Only for local map
// void HeightMap::rayCasting(const Position3 &robotPosition3)
// {
//     std::cout << "raycast test" << std::endl;
//     // add("max_height");

//     // Check the cell at robot position is valid
//     Index grid_at_robot;
//     Position robotPosition2D(robotPosition3(0), robotPosition3(1));
//     if (!getIndex(robotPosition2D, grid_at_robot))
//         return;

//     // for (GridMapIterator iter(*this); !iter.isPastEnd(); ++iter)
//     Index start_index(75, 75);
//     Index search_region(151, 151);
//     SubmapIterator sub_iter(*this, grid_at_robot, search_region);
//     int count_ray = 0;

//     for (sub_iter; !sub_iter.isPastEnd(); ++sub_iter)
//     {
//         const auto &grid = *sub_iter;

//         // Check elevation is valid
//         if (!isValid(grid))
//             continue;

//         const auto &groundHeight = at("height_ground", grid);

//         if (std::isnan(groundHeight))
//             continue;

//         Position point;
//         getPosition(*sub_iter, point);
//         float ray_diff_x = point.x() - robotPosition2D.x();
//         float ray_diff_y = point.y() - robotPosition2D.y();
//         float distance_to_point = std::sqrt(ray_diff_x * ray_diff_x + ray_diff_y * ray_diff_y);
//         // if (!(distance_to_point > 0))
//         //     continue;

//         // Ray Casting
//         ++count_ray;

//         for (LineIterator rayiter(*this, grid_at_robot, grid); !rayiter.isPastEnd(); ++rayiter)
//         {
//             Position cell_position;
//             getPosition(*rayiter, cell_position);
//             const float cell_diff_x = cell_position.x() - robotPosition2D.x();
//             const float cell_diff_y = cell_position.y() - robotPosition2D.y();
//             const float distance_to_cell = distance_to_point - std::sqrt(cell_diff_x * cell_diff_x + cell_diff_y *
//             cell_diff_y); const float max_height = groundHeight + (robotPosition3.z() - groundHeight) /
//             distance_to_point * distance_to_cell; auto &cell_max_height = at("max_height", grid);

//             if (std::isnan(cell_max_height) || cell_max_height > max_height)
//                 cell_max_height = max_height;
//         }
//     }

//     // List of cells to be removed
//     std::vector<Position> cellsToRemove;
//     SubmapIterator sub_iter2(*this, grid_at_robot, search_region);
//     int count = 0;
//     for (sub_iter2; !sub_iter2.isPastEnd(); ++sub_iter2)
//     {
//         const auto &grid = *sub_iter2;

//         // Check elevation is valid
//         if (!isValid(grid))
//             continue;

//         const auto &elevation = at("elevation", grid);
//         const auto &variance = at("variance", grid);
//         const auto &max_height = at("max_height", grid);
//         if (!std::isnan(max_height) && elevation > max_height)
//         {
//             Position cell_position;
//             getPosition(grid, cell_position);
//             cellsToRemove.push_back(cell_position);

//             ++count;
//         }
//     }
//     std::cout << count << std::endl;
//     std::cout << count_ray << std::endl;

//     // Elevation Removal
//     for (const auto &cell_position : cellsToRemove)
//     {
//         Index grid;
//         if (!getIndex(cell_position, grid))
//             continue;

//         if (isValid(grid))
//         {
//             at("elevation", grid) = NAN;
//             at("variance", grid) = NAN;
//         }
//     }
// }
}  // namespace grid_map
