/*
 * HeightMapping.h
 *
 *  Created on: Aug 17, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#pragma once

// Utility
#include "utils/TransformHandler.h"
#include "utils/pointcloud.h"

// Height Map
#include <height_map_core/height_map_core.h>
#include <height_map_msgs/HeightMapMsgs.h>
#include <height_map_pcl/pclProcessor.h>

// Point types
#include "PointTypes.h"

class FastHeightFilter {
public:
  FastHeightFilter(float min_z, float max_z) : min_z_(min_z), max_z_(max_z) {}

  template <typename PointT>
  void filter(const typename pcl::PointCloud<PointT>::Ptr &cloud,
              typename pcl::PointCloud<PointT>::Ptr &filtered_cloud) {
    filtered_cloud->points.clear();
    filtered_cloud->points.reserve(cloud->points.size());

    for (const auto &point : cloud->points) {
      if (point.z >= min_z_ && point.z <= max_z_) {
        filtered_cloud->points.push_back(point);
      }
    }

    filtered_cloud->width = filtered_cloud->points.size();
    filtered_cloud->height = 1;
    filtered_cloud->is_dense = cloud->is_dense;
  }

private:
  float min_z_, max_z_;
};

class HeightMapping {
public:
  struct Parameters {
    // Height mapping parameters
    std::string heightEstimatorType;
    std::string mapFrame;
    double mapLengthX;
    double mapLengthY;
    double gridResolution;
    float minHeight;
    float maxHeight;
  };

  HeightMapping(const Parameters &params);

  template <typename PointT>
  void fastHeightFilter(const typename pcl::PointCloud<PointT>::Ptr &cloud,
                        typename pcl::PointCloud<PointT>::Ptr &filtered_cloud);
  template <typename PointT>
  void update(const typename pcl::PointCloud<PointT>::Ptr &cloud,
                     const Eigen::Affine3d &transform);

  void updateMapOrigin(const grid_map::Position &position);

  const grid_map::HeightMap &getHeightMap() const;

private:
  void paramValidityCheck();
  void initHeightMap();
  void initHeightEstimator();

  grid_map::HeightMap map_{10, 10, 0.1};
  Parameters params_;

  FastHeightFilter heightFilter_;
  height_map::HeightEstimatorBase::Ptr height_estimator_;
};
