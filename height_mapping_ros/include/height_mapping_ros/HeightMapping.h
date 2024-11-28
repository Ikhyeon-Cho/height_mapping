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
#include "height_mapping_ros/CloudTypes.h"
#include <height_mapping_core/height_mapping_core.h>
#include <height_mapping_msgs/HeightMapMsgs.h>

class FastHeightFilter {
public:
  FastHeightFilter(float min_z, float max_z) : min_z_(min_z), max_z_(max_z) {}

  template <typename PointT>
  void filter(const typename pcl::PointCloud<PointT>::Ptr &cloud,
              typename pcl::PointCloud<PointT>::Ptr &filtered_cloud) {
    filtered_cloud->header = cloud->header;

    filtered_cloud->points.clear();
    filtered_cloud->points.reserve(cloud->points.size());

    for (const auto &point : cloud->points) {
      if (point.z >= min_z_ && point.z <= max_z_) {
        filtered_cloud->points.emplace_back(point);
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
  typename pcl::PointCloud<PointT>::Ptr
  mapping(const typename pcl::PointCloud<PointT>::Ptr &cloud);

  /*
   * Correct heightmap using raycasting
   * @param pointcloud: pointcloud for raycasting [ref: map frame]
   * @param sensorOrigin: sensor origin [ref: map frame]
   */
  template <typename PointT>
  void raycasting(const Eigen::Vector3f &sensorOrigin,
                  const typename pcl::PointCloud<PointT>::Ptr &cloud);

  void updateMapOrigin(const grid_map::Position &position);

  const grid_map::HeightMap &getHeightMap() const;

private:
  void paramValidityCheck();
  void initHeightMap();
  void initHeightEstimator();

  template <typename PointT>
  typename pcl::PointCloud<PointT>::Ptr
  griddedFilterWithMaxHeight(const typename pcl::PointCloud<PointT>::Ptr &cloud,
                             float gridSize);

  struct pair_hash {
    template <class T1, class T2>
    std::size_t operator()(const std::pair<T1, T2> &p) const {
      auto h1 = std::hash<T1>{}(p.first);
      auto h2 = std::hash<T2>{}(p.second);
      return h1 ^ h2;
    }
  };

  grid_map::HeightMap map_;
  Parameters params_;

  FastHeightFilter heightFilter_;
  height_mapping::HeightEstimatorBase::Ptr heightEstimator_;
  height_mapping::HeightMapRaycaster raycaster_;

  float correction_threshold_{0.15f};
};
