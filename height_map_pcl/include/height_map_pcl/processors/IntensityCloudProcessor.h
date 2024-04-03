/*
 * IntensityCloudProcessor.h
 *
 *  Created on: Apr 3, 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef INTENSITY_CLOUD_PROCESSOR_H
#define INTENSITY_CLOUD_PROCESSOR_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <height_map_core/HeightMap.h>

namespace height_map
{
class IntensityCloudProcessor
{
public:
  IntensityCloudProcessor() = default;

  std::pair<bool, pcl::PointCloud<pcl::PointXYZI>::Ptr> gridDownsampling(const pcl::PointCloud<pcl::PointXYZI>& cloud,
                                                                         const grid_map::HeightMap& map);

  std::pair<bool, pcl::PointCloud<pcl::PointXYZI>::Ptr>
  removeInconsistentPoints(const pcl::PointCloud<pcl::PointXYZI>& cloud, const grid_map::HeightMap& map);

private:
  template <typename PointT>
  bool isEmpty(const pcl::PointCloud<PointT>& cloud)
  {
    if (cloud.empty())
    {
      std::cout << "\033[33m[ PointCloudProcessor] Warning: Input cloud is empty! \033[0m" << std::endl;
      return false;
    }
    return true;
  }

  grid_map::HeightMap map_{ 10, 10, 0.1 };
  bool is_map_initialized_{ false };
};
}  // namespace height_map

#endif /* INTENSITY_CLOUD_PROCESSOR_H */