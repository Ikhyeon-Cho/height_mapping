/*
 * HeightEstimatorBase.h
 *
 *  Created on: Apr 2, 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_ESTIMATOR_BASE_H
#define HEIGHT_ESTIMATOR_BASE_H

#include "height_mapping_core/heightmap/HeightMap.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

// #include <pcl/common/eigen.h>

namespace height_map
{
class HeightEstimatorBase
{
public:
  using Ptr = std::unique_ptr<HeightEstimatorBase>;

  HeightEstimatorBase() = default;
  virtual ~HeightEstimatorBase() = default;

  virtual void estimate(grid_map::HeightMap& map, const pcl::PointCloud<pcl::PointXYZ>& cloud) = 0;
  virtual void estimate(grid_map::HeightMap& map, const pcl::PointCloud<pcl::PointXYZI>& cloud) = 0;
  virtual void estimate(grid_map::HeightMap& map, const pcl::PointCloud<pcl::PointXYZRGB>& cloud) = 0;

protected:
  template <typename PointT>
  bool hasEmptyCloud(const pcl::PointCloud<PointT>& cloud)
  {
    if (cloud.empty())
    {
      std::cout << "\033[33m[ HeightEstimator] Warning: Input cloud is empty! \033[0m" << std::endl;
      return true;
    }
    return false;
  }
};
}  // namespace height_map

#endif /* HEIGHT_ESTIMATOR_BASE_H */
