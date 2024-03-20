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
  add(layer_elevation_);
  add(layer_uncertainty_);
  add(layer_intensity_);

  // Add consistency checking layers
  add(layer_min_z_);
  add(layer_max_z_);

  // Add statistics layers
  add(layer_statistics_mu_);
  add(layer_statistics_sigma2_);
  add(layer_statistics_n_);

  setFrameId("map");
  setGeometry(grid_map::Length(length_x, length_y), grid_resolution);
  setBasicLayers({ layer_elevation_, layer_uncertainty_, layer_intensity_ });

  // For grid-based downsampling of pointcloud
  add(layer_downsampled_cloud_);
  add(layer_downsampled_cloud_intensity_);
}

HeightMap::HeightMap() : HeightMap(15, 15, 0.15)
{
}

pcl::PointCloud<pcl::PointXYZI>::Ptr
HeightMap::getGridDownsampledCloud(const pcl::PointCloud<pcl::PointXYZI>& pointcloud)
{
  // Note: Add() function and erase() function is too slow. Be careful!
  clear(layer_downsampled_cloud_);
  clear(layer_downsampled_cloud_intensity_);

  auto& data_downsampled_cloud = get(layer_downsampled_cloud_);
  auto& data_downsampled_cloud_intensity = get(layer_downsampled_cloud_intensity_);

  // Create a set to keep track of unique grid indices.
  std::vector<grid_map::Index> measured_index_list;
  grid_map::Index index;
  for (const auto& point : pointcloud)
  {
    // Check whether point is inside the map
    if (!getIndex(grid_map::Position(point.x, point.y), index))
      continue;

    // First grid height measuerment
    if (isEmptyAt(layer_downsampled_cloud_, index))
    {
      data_downsampled_cloud(index(0), index(1)) = point.z;
      data_downsampled_cloud_intensity(index(0), index(1)) = point.intensity;
      measured_index_list.push_back(index);
    }
    else if (point.z > data_downsampled_cloud(index(0), index(1)))
    {
      data_downsampled_cloud(index(0), index(1)) = point.z;
      data_downsampled_cloud_intensity(index(0), index(1)) = point.intensity;
    }
  }  // pointcloud loop ends

  // Get overrided point height (per grid) >> saved in downsampled_cloud
  auto downsampled_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  downsampled_cloud->header = pointcloud.header;
  downsampled_cloud->reserve(measured_index_list.size());

  // declare outside of for loop : for faster speed
  // Eigen constructors are pretty slow
  grid_map::Position3 grid_position3D;
  pcl::PointXYZI point;
  for (const auto& index : measured_index_list)
  {
    getPosition3(layer_downsampled_cloud_, index, grid_position3D);

    point.x = grid_position3D.x();
    point.y = grid_position3D.y();
    point.z = grid_position3D.z();
    point.intensity = data_downsampled_cloud_intensity(index(0), index(1));

    downsampled_cloud->push_back(point);
  }

  return downsampled_cloud;
}

void HeightMap::update(const pcl::PointCloud<pcl::PointXYZI>& pointcloud, const std::string& method)
{
  if (pointcloud.empty())
  {
    std::cout << " [HeightMap] Warning: Skipping map update - point cloud is empty! \n";
    return;
  }

  if (pointcloud.header.frame_id != getFrameId())
  {
    std::cout << " [HeightMap] Warning: Frame ID mismatch - pointcloud is in a different frame! \n";
    return;
  }

  // Access to data matrix -> this is faster than calling at() function for each point
  auto& data_elevation = getHeightMatrix();
  auto& data_uncertainty = getUncertaintyMatrix();
  auto& data_intensity = get(layer_intensity_);

  auto& data_min_z = get(layer_min_z_);
  auto& data_max_z = get(layer_max_z_);

  auto& data_mu = get(layer_statistics_mu_);
  auto& data_sigma2 = get(layer_statistics_sigma2_);
  auto& data_n = get(layer_statistics_n_);

  // Only check the measured grid indices for efficiency
  grid_map::Index index;
  for (const auto& point : pointcloud)
  {
    // We assume that the point with large distance is less reliable for height estimation
    auto point_uncertainty = std::sqrt(point.x * point.x + point.y * point.y);

    // Skip if point is outside of the map
    if (!getIndex(grid_map::Position(point.x, point.y), index))
      continue;

    // 1. Update basic layers
    auto& elevation = data_elevation(index(0), index(1));
    auto& uncertainty = data_uncertainty(index(0), index(1));
    auto& intensity = data_intensity(index(0), index(1));

    // 2. Update consistency checking layers
    auto& min_z = data_min_z(index(0), index(1));
    auto& max_z = data_max_z(index(0), index(1));

    // 3. Update statistics layers
    auto& n_sample = data_n(index(0), index(1));
    auto& sample_maen = data_mu(index(0), index(1));
    auto& sample_variance = data_sigma2(index(0), index(1));

    if (isEmptyAt(index))
    {
      // 1. Initialize height attributes
      elevation = point.z;              // at(layer_elevation, index) = point.z
      uncertainty = point_uncertainty;  // at(layer_variance, index) = point_uncertainty
      intensity = point.intensity;      // at(layer_intensity, index) = point.intensity

      // 2. Initialize consistency check layers
      min_z = point.z;
      max_z = point.z;

      // 3. Initialize statistics layers: mu, sigma2, n
      data_mu(index(0), index(1)) = point.z;  // at(layer_statistics_mu_, index) = point.z
      data_sigma2(index(0), index(1)) = 0;    // at(layer_statistics_sigma2_, index) = 0;
      data_n(index(0), index(1)) = 1;         // at(layer_statistics_n_, index) = 1

      continue;
    }

    // Enough elevation update
    if (uncertainty < 0.5)
      continue;

    // 1. Elevation update
    if (method == "KalmanFilter")
    {
      doKF(elevation, uncertainty, point.z, point_uncertainty);
      doKF(intensity, uncertainty, point.intensity, point_uncertainty);
    }

    else if (method == "EwmaFilter")
    {
      doEWMA(elevation, uncertainty, point.z, point_uncertainty);
      doEWMA(intensity, uncertainty, point.intensity, point_uncertainty);
    }

    else if (method == "KalmanFilter+ConsistencyCheck")
    {
      doKF(elevation, uncertainty, point.z, point_uncertainty);
      doKF(intensity, uncertainty, point.intensity, point_uncertainty);

      // 2. Consistency check -> reset basic layers if inconsistency is detected
      min_z = std::min(min_z, point.z);
      max_z = std::max(max_z, point.z);
      if (max_z - min_z > 0.2)
      {
        uncertainty = 1e+9;
      }
    }

    else
    {
      std::cout << " [HeightMap] Warning: Unknown method for height update! \n";
    }

    // 3. Update statistics layers: mu, sigma2, n
    // recursive update of mean and variance:
    // https://math.stackexchange.com/questions/374881/recursive-formula-for-variance
    n_sample += 1;
    auto prev_sample_mean = sample_maen;
    sample_maen = sample_maen + (point.z - sample_maen) / n_sample;
    sample_variance = sample_variance + std::pow(prev_sample_mean, 2) - std::pow(sample_maen, 2) +
                      (std::pow(point.z, 2) - sample_variance - std::pow(prev_sample_mean, 2)) / n_sample;
  }
}

void HeightMap::doKF(float& mu, float& sigma2, float point_z, float point_sigma2)
{
  mu = (mu * point_sigma2 + point_z * sigma2) / (sigma2 + point_sigma2);
  sigma2 = sigma2 * point_sigma2 / (sigma2 + point_sigma2);
}

void HeightMap::doEWMA(float& mu, float& sigma2, float point_z, float point_sigma2, float alpha)
{
  mu = (1 - alpha) * mu + alpha * point_z;
  sigma2 = (1 - alpha) * sigma2 + alpha * point_sigma2;
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
      if (!getPosition3(layer_elevation_, *subiter, nearGridPosition3))
        continue;

      if ((nearGridPosition3.head<2>() - thisGridPosition).norm() < 0.05)
        continue;

      elevation_sum += nearGridPosition3.z();
      ++n_sum;
    }
    if (n_sum == 0)
      continue;

    auto& elevation = at(layer_elevation_, thisGrid);
    const auto& elevation_bottom = at("height_ground", thisGrid);
    // Smooth only potentially ground cell
    if (elevation - elevation_bottom > 0.15)
      continue;

    elevation = elevation_sum / n_sum;
  }
}

const std::string& HeightMap::getHeightLayer() const
{
  return layer_elevation_;
}

const std::string& HeightMap::getUncertaintyLayer() const
{
  return layer_uncertainty_;
}

const grid_map::GridMap::Matrix& HeightMap::getHeightMatrix() const
{
  return get(layer_elevation_);
}

grid_map::GridMap::Matrix& HeightMap::getHeightMatrix()
{
  return get(layer_elevation_);
}

const grid_map::GridMap::Matrix& HeightMap::getUncertaintyMatrix() const
{
  return get(layer_uncertainty_);
}

grid_map::GridMap::Matrix& HeightMap::getUncertaintyMatrix()
{
  return get(layer_uncertainty_);
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

//         const auto &elevation = at(layer_elevation_, grid);
//         const auto &variance = at(layer_uncertainty_, grid);
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
//             at(layer_elevation_, grid) = NAN;
//             at(layer_uncertainty_, grid) = NAN;
//         }
//     }
// }
}  // namespace grid_map
