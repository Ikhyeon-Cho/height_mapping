/*
 * DescriptorMap.h
 *
 *  Created on: Aug 17, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef TRAVERSABILITY_MAPPING_DESCRIPTOR_MAP_H
#define TRAVERSABILITY_MAPPING_DESCRIPTOR_MAP_H

#include "terrain_mapping/ElevationMap.h"
#include <grid_map_core/grid_map_core.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>

class TerrainDescriptor
{
public:
  TerrainDescriptor() = default;
  bool doCovarianceAnalysis(const std::vector<Eigen::Vector3d>& points);
  const double& getSmallestEigenValue() const;
  const Eigen::Vector3d& getNormalVector() const;
  float computeStep() const;
  float computeSlope() const;
  float computeRoughness() const;
  float computeCurvature() const;

private:
  // points in local
  std::vector<Eigen::Vector3d> points_;

  // covariance matrix
  Eigen::Vector3d eigen_values_;
  Eigen::Vector3d normal_vector_;  // eigen_vector1_
  Eigen::Vector3d eigen_vector2_;
  Eigen::Vector3d eigen_vector3_;
  double trace_;
};

class DescriptorMap : public ElevationMap
{
public:
  DescriptorMap();
  DescriptorMap(const grid_map::GridMap& map);
  void setElevationMap(const grid_map::GridMap& map);

  void update(const pcl::PointCloud<pcl::PointXYZI>& pointcloud);

private:
  bool extractDescriptorAt(const grid_map::Index& index, TerrainDescriptor& descriptor);

  double local_radius_{ 0.15 };  // 15cm radius circle as local area
};

#endif