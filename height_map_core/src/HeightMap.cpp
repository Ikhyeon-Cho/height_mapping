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
{
  // Add basic layers
  add(layer_height_);
  add(layer_variance_);

  setFrameId("map");
  setGeometry(grid_map::Length(length_x, length_y), grid_resolution);
  setBasicLayers({ layer_height_ });
}

void HeightMap::update(const pcl::PointCloud<pcl::PointXYZI>& pointcloud)
{
  // Add intensity layer if not exists
  if (!exists(layer_intensity_))
    add(layer_intensity_);

  if (pointcloud.empty())
  {
    std::cout << " [ HeightMap] Warning: Skipping map update - point cloud is empty! \n";
    return;
  }

  if (pointcloud.header.frame_id != getFrameId())
  {
    std::cout << " [ HeightMap] Warning: Frame ID mismatch - pointcloud is in a different frame! \n";
    return;
  }

  // Access to data matrix -> this is faster than calling at() function for each point
  auto& data_elevation = getHeightMatrix();
  auto& data_variance = get(layer_variance_);
  auto& data_intensity = get(layer_intensity_);

  //////////////////////////////////////////////////
  ////////////// 1. Kalman Filter //////////////////
  //////////////////////////////////////////////////
  // We assume that the point with large distance //
  //    is less reliable for height estimation    //
  //////////////////////////////////////////////////
  //////////////////////////////////////////////////

  // // Only check the measured grid indices for efficiency
  // grid_map::Index index;
  // for (const auto& point : pointcloud)
  // {
  //   // Skip if point is outside of the map
  //   if (!getIndex(grid_map::Position(point.x, point.y), index))
  //     continue;

  //   auto point_variance = std::sqrt(point.x * point.x + point.y * point.y);

  //   auto& elevation = data_elevation(index(0), index(1));
  //   auto& variance = data_variance(index(0), index(1));
  //   auto& intensity = data_intensity(index(0), index(1));

  //   // Initialize height attributes
  //   if (isEmptyAt(index))
  //   {
  //     elevation = point.z;              // at(layer_elevation, index) = point.z
  //     variance = point_variance;  // at(layer_variance, index) = point_variance
  //     intensity = point.intensity;      // at(layer_intensity, index) = point.intensity

  //     continue;
  //   }

  //   doKF(elevation, variance, point.z, point_variance);
  //   doKF(intensity, variance, point.intensity, point_variance);

  ////////////////////////////////////////////////
  ////////////////////////////////////////////////
  ///// 2. Kalman Filter + Consistency Check /////
  ////////////////////////////////////////////////
  ////////////////////////////////////////////////

  // if (!exists(layer_height_diff_))
  // {
  //   add(layer_height_diff_);
  //   add(layer_min_height_);
  //   add(layer_max_height_);
  // }

  // auto& data_min_z = get(layer_min_height_);
  // auto& data_max_z = get(layer_max_height_);
  // auto& data_diff_z = get(layer_height_diff_);

  // // Only check the measured grid indices for efficiency
  // grid_map::Index index;
  // for (const auto& point : pointcloud)
  // {
  //   // Skip if point is outside of the map
  //   if (!getIndex(grid_map::Position(point.x, point.y), index))
  //     continue;

  //   auto point_variance = std::sqrt(point.x * point.x + point.y * point.y);

  //   auto& elevation = data_elevation(index(0), index(1));
  //   auto& variance = data_variance(index(0), index(1));
  //   auto& intensity = data_intensity(index(0), index(1));
  //   auto& min_z = data_min_z(index(0), index(1));
  //   auto& max_z = data_max_z(index(0), index(1));
  //   auto& diff_z = data_diff_z(index(0), index(1));

  //   // Initialize height attributes
  //   if (isEmptyAt(index))
  //   {
  //     elevation = point.z;          // at(layer_elevation, index) = point.z
  //     variance = point_variance;    // at(layer_variance, index) = point_variance
  //     intensity = point.intensity;  // at(layer_intensity, index) = point.intensity
  //     min_z = point.z;
  //     max_z = point.z;
  //     diff_z = 0;

  //     continue;
  //   }

  //   min_z = std::min(min_z, point.z);
  //   max_z = std::max(max_z, point.z);
  //   diff_z = max_z - min_z;

  //   if (diff_z > 0.2)
  //   {
  //     variance = 1e+9;
  //     max_z = min_z = point.z;
  //     diff_z = 0;
  //   }

  //   doKF(elevation, variance, point.z, point_variance);
  //   doKF(intensity, variance, point.intensity, point_variance);

  ////////////////////////////////////////////////
  ////////////////////////////////////////////////
  /////////////// 3. EWMA Filter /////////////////
  ////////////////////////////////////////////////
  ////////////////////////////////////////////////

  // // Only check the measured grid indices for efficiency
  // grid_map::Index index;
  // for (const auto& point : pointcloud)
  // {
  //   // Skip if point is outside of the map
  //   if (!getIndex(grid_map::Position(point.x, point.y), index))
  //     continue;

  //   auto point_variance = std::sqrt(point.x * point.x + point.y * point.y);

  //   auto& elevation = data_elevation(index(0), index(1));
  //   auto& variance = data_variance(index(0), index(1));
  //   auto& intensity = data_intensity(index(0), index(1));

  //   // Initialize height attributes
  //   if (isEmptyAt(index))
  //   {
  //     elevation = point.z;          // at(layer_elevation, index) = point.z
  //     variance = point_variance;    // at(layer_variance, index) = point_variance
  //     intensity = point.intensity;  // at(layer_intensity, index) = point.intensity

  //     continue;
  //   }

  //   doEWMA(elevation, point.z, 0.8);
  //   doEWMA(variance, point_variance, 0.8);
  //   doEWMA(intensity, point.intensity, 0.8);

  ////////////////////////////////////////////////
  ////////////////////////////////////////////////
  /////////////// 4. Mean Filter /////////////////
  ////////////////////////////////////////////////
  ////////////////////////////////////////////////

  if (!exists(layer_measured_num_))
    add(layer_measured_num_);

  auto& data_n = get(layer_measured_num_);

  if (!exists(layer_height_diff_))
  {
    add(layer_height_diff_);
    add(layer_min_height_);
    add(layer_max_height_);
  }

  auto& data_min_z = get(layer_min_height_);
  auto& data_max_z = get(layer_max_height_);
  auto& data_diff_z = get(layer_height_diff_);

  // Only check the measured grid indices for efficiency
  grid_map::Index index;
  for (const auto& point : pointcloud)
  {
    // Skip if point is outside of the map
    if (!getIndex(grid_map::Position(point.x, point.y), index))
      continue;

    auto& elevation = data_elevation(index(0), index(1));
    auto& variance = data_variance(index(0), index(1));
    auto& intensity = data_intensity(index(0), index(1));
    auto& n_sample = data_n(index(0), index(1));
    auto& min_z = data_min_z(index(0), index(1));
    auto& max_z = data_max_z(index(0), index(1));
    auto& diff_z = data_diff_z(index(0), index(1));

    // Initialize height attributes
    if (isEmptyAt(index))
    {
      elevation = point.z;          // at(layer_elevation, index) = point.z
      variance = 0;                 // at(layer_variance, index) = point_variance
      intensity = point.intensity;  // at(layer_intensity, index) = point.intensity
      n_sample = 1;                 // at(layer_measured_num_, index) = 1
      min_z = point.z;
      max_z = point.z;
      diff_z = 0;

      continue;
    }

    // Update statistics: mu, sigma2, n
    // recursive update of mean and variance:
    // https://math.stackexchange.com/questions/374881/recursive-formula-for-variance
    n_sample += 1;
    auto prev_elevation = elevation;
    elevation = elevation + (point.z - elevation) / n_sample;
    variance = variance + std::pow(prev_elevation, 2) - std::pow(elevation, 2) +
               (std::pow(point.z, 2) - variance - std::pow(prev_elevation, 2)) / n_sample;

    // For dynamic obstacle
    min_z = std::min(min_z, point.z);
    max_z = std::max(max_z, point.z);
    diff_z = point.z - min_z;

    if (diff_z > 0.3)
    {
      // reset the elevation 
      elevation = point.z;
      variance = 0;
      n_sample = 1;

      max_z = min_z = point.z;
      // diff_z = 0;
    }
  }
}

void HeightMap::doKF(float& mu, float& sigma2, float point_z, float point_sigma2)
{
  mu = (mu * point_sigma2 + point_z * sigma2) / (sigma2 + point_sigma2);
  sigma2 = sigma2 * point_sigma2 / (sigma2 + point_sigma2);
}

void HeightMap::doEWMA(float& mu, float point_z, float alpha)
{
  mu = (1 - alpha) * mu + alpha * point_z;
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
      if (!getPosition3(layer_height_, *subiter, nearGridPosition3))
        continue;

      if ((nearGridPosition3.head<2>() - thisGridPosition).norm() < 0.05)
        continue;

      elevation_sum += nearGridPosition3.z();
      ++n_sum;
    }
    if (n_sum == 0)
      continue;

    auto& elevation = at(layer_height_, thisGrid);
    const auto& elevation_bottom = at("height_ground", thisGrid);
    // Smooth only potentially ground cell
    if (elevation - elevation_bottom > 0.15)
      continue;

    elevation = elevation_sum / n_sum;
  }
}

const std::string& HeightMap::getHeightLayer() const
{
  return layer_height_;
}

const grid_map::GridMap::Matrix& HeightMap::getHeightMatrix() const
{
  return get(layer_height_);
}

grid_map::GridMap::Matrix& HeightMap::getHeightMatrix()
{
  return get(layer_height_);
}

bool HeightMap::isEmptyAt(const grid_map::Index& index) const
{
  return !isValid(index);
}

bool HeightMap::isEmptyAt(const std::string& layer, const grid_map::Index& index) const
{
  return !std::isfinite(at(layer, index));
}

float HeightMap::getMaxHeight() const
{
  const auto& data = getHeightMatrix();

  // https://www.geeksforgeeks.org/difference-between-stdnumeric_limitst-min-max-and-lowest-in-cpp/
  auto fillNaNForFindingMaxVal = data.array().isNaN().select(std::numeric_limits<double>::lowest(), data);
  float max_height = fillNaNForFindingMaxVal.maxCoeff();

  return max_height;
}

float HeightMap::getMinHeight() const
{
  const auto& data = getHeightMatrix();

  auto fillNaNForFindingMinVal = data.array().isNaN().select(std::numeric_limits<double>::max(), data);
  float min_height = fillNaNForFindingMinVal.minCoeff();

  return min_height;
}

std::tuple<float, float> HeightMap::getMinMaxHeight() const
{
  const auto& data = getHeightMatrix();

  // https://www.geeksforgeeks.org/difference-between-stdnumeric_limitst-min-max-and-lowest-in-cpp/
  auto fillNaNForFindingMaxVal = data.array().isNaN().select(std::numeric_limits<double>::lowest(), data);
  auto fillNaNForFindingMinVal = data.array().isNaN().select(std::numeric_limits<double>::max(), data);

  float min_value = fillNaNForFindingMinVal.minCoeff();
  float max_value = fillNaNForFindingMaxVal.maxCoeff();

  return { min_value, max_value };
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

//         const auto &elevation = at(layer_height_, grid);
//         const auto &variance = at(layer_variance_, grid);
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
//             at(layer_height_, grid) = NAN;
//             at(layer_variance_, grid) = NAN;
//         }
//     }
// }
}  // namespace grid_map
