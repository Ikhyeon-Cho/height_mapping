/*
 * DescriptorMap.cpp
 *
 *  Created on: Aug 17, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "terrain_mapping/DescriptorMap.h"

DescriptorMap::DescriptorMap(const grid_map::GridMap& map)
  : ElevationMap(
        { "elevation", "variance", "step", "slope", "roughness", "curvature", "normal_x", "normal_y", "normal_z" })
{
  setFrameId(map.getFrameId());
  setGeometry(map.getLength(), map.getResolution());
  setBasicLayers({ "elevation" });
}

DescriptorMap::DescriptorMap()
  : ElevationMap(
        { "elevation", "variance", "step", "slope", "roughness", "curvature", "normal_x", "normal_y", "normal_z" })
{
  setFrameId("map");
  setGeometry(grid_map::Length(10, 10), 0.1);
  setBasicLayers({ "elevation" });
}

void DescriptorMap::setElevationMap(const grid_map::GridMap& map)
{
  // Copy elevation and variance layer
  get("elevation") = map.get("elevation");
  get("variance") = map.get("variance");
  this->move(map.getPosition());
}

void DescriptorMap::update(const pcl::PointCloud<pcl::PointXYZI>& pointcloud)
{
  // 1. Log currently measured grid index : inspect these regions only for computational efficiency
  const std::string measurement_checking_layer("measured");
  add(measurement_checking_layer);
  auto& measurement_checking_layer_data = get(measurement_checking_layer);
  std::vector<grid_map::Index> measured_index_list;
  for (const auto& point : pointcloud)
  {
    // Check whether point is inside the map
    grid_map::Index index;
    if (!getIndex(grid_map::Position(point.x, point.y), index))
      continue;

    // Save measuerment received area
    if (isEmptyAt(measurement_checking_layer, index))
    {
      measurement_checking_layer_data(index(0), index(1)) = 1;
      measured_index_list.push_back(index);
    }
  }  // pointcloud loop ends
  // erase(measurement_checking_layer);

  // 2. Compute terrain descriptor for currently measured cell
  auto& step_layer = get("step");
  auto& slope_layer = get("slope");
  auto& roughness_layer = get("roughness");
  auto& curvature_layer = get("curvature");
  auto& normalX_layer = get("normal_x");
  auto& normalY_layer = get("normal_y");
  auto& normalZ_layer = get("normal_z");

  for (const auto& index : measured_index_list)
  {
    TerrainDescriptor descriptor;
    if (!extractDescriptorAt(index, descriptor))
      continue;

    step_layer(index(0), index(1)) = descriptor.computeStep();
    slope_layer(index(0), index(1)) = descriptor.computeSlope();
    roughness_layer(index(0), index(1)) = descriptor.computeRoughness();
    curvature_layer(index(0), index(1)) = descriptor.computeCurvature();
    normalX_layer(index(0), index(1)) = descriptor.getNormalVector().x();
    normalY_layer(index(0), index(1)) = descriptor.getNormalVector().y();
    normalZ_layer(index(0), index(1)) = descriptor.getNormalVector().z();
  }
}

bool DescriptorMap::extractDescriptorAt(const grid_map::Index& queried_index, TerrainDescriptor& descriptor)
{
  std::vector<grid_map::Position3> points_in_local;

  grid_map::Position queried_position;
  getPosition(queried_index, queried_position);

  for (grid_map::CircleIterator iterator(*this, queried_position, local_radius_); !iterator.isPastEnd(); ++iterator)
  {
    grid_map::Position3 elevation_point;
    if (!getPosition3("elevation", *iterator, elevation_point))
      continue;

    points_in_local.push_back(elevation_point);
  }  // circle iterator ends

  return descriptor.doCovarianceAnalysis(points_in_local);
}

bool TerrainDescriptor::doCovarianceAnalysis(const std::vector<Eigen::Vector3d>& points)
{
  points_ = points;

  if (points.size() < 3)
    // std::cout << "Not enough points to establish normal direction (nPoints = " << nPoints << ")" << std::endl;
    return false;

  Eigen::Vector3d sum_points(Eigen::Vector3d::Zero());
  Eigen::Matrix3d squared_sum_points(Eigen::Matrix3d::Zero());
  for (const auto& point : points)
  {
    sum_points += point;
    squared_sum_points.noalias() += point * point.transpose();
  }
  const auto mean_points = sum_points / points.size();
  const auto covariance = squared_sum_points / points.size() - mean_points * mean_points.transpose();

  if (covariance.trace() < std::numeric_limits<float>::epsilon())
    return false;  // Covariance matrix needed for eigen decomposition is degenerated.

  trace_ = covariance.trace();

  // Compute Eigenvectors.
  // Eigenvalues are ordered small to large.
  // Worst case bound for zero eigenvalue from :
  // https://eigen.tuxfamily.org/dox/classEigen_1_1SelfAdjointEigenSolver.html
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver;
  solver.computeDirect(covariance, Eigen::DecompositionOptions::ComputeEigenvectors);

  normal_vector_ = solver.eigenvectors().col(0);
  eigen_vector2_ = solver.eigenvectors().col(1);
  eigen_vector3_ = solver.eigenvectors().col(2);
  eigen_values_ = solver.eigenvalues();

  if (eigen_values_(1) < 1e-8)  // second eigen value is near zero: Line feature
    return false;               // normal is not defined

  // Check direction of the normal vector and flip the sign towards the user defined direction.
  auto positive_Zaxis(Eigen::Vector3d::UnitZ());
  if (normal_vector_.dot(positive_Zaxis) < 0.0)
    normal_vector_ *= -1;

  return true;
}

float TerrainDescriptor::computeStep() const
{
  auto minMax =
      std::minmax_element(points_.begin(), points_.end(), [](const Eigen::Vector3d& lhs, const Eigen::Vector3d& rhs) {
        return lhs(2) < rhs(2);  // Compare z-components.
      });

  double minZ = (*minMax.first)(2);   // Minimum z-component.
  double maxZ = (*minMax.second)(2);  // Maximum z-component.
  return maxZ - minZ;
}

float TerrainDescriptor::computeSlope() const
{
  return std::acos(std::abs(normal_vector_.z())) * 180 / M_PI;
}

float TerrainDescriptor::computeRoughness() const
{
  return std::sqrt(eigen_values_(0));
}

float TerrainDescriptor::computeCurvature() const
{
  return std::abs(eigen_values_(0) / trace_);
}

const double& TerrainDescriptor::getSmallestEigenValue() const
{
  return eigen_values_(0);
}

const Eigen::Vector3d& TerrainDescriptor::getNormalVector() const
{
  return normal_vector_;
}