/*
 * RGBCloudProcessor.h
 *
 *  Created on: Apr 3, 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef RGB_CLOUD_PROCESSOR_H
#define RGB_CLOUD_PROCESSOR_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <height_map_core/HeightMap.h>

namespace height_map
{
class RGBCloudProcessor
{
public:
  RGBCloudProcessor() = default;

  std::pair<bool, pcl::PointCloud<pcl::PointXYZRGB>::Ptr>
  gridDownsampling(const pcl::PointCloud<pcl::PointXYZRGB>& cloud, const grid_map::HeightMap& map);

  std::pair<bool, pcl::PointCloud<pcl::PointXYZRGB>::Ptr>
  removeInconsistentPoints(const pcl::PointCloud<pcl::PointXYZRGB>& cloud, const grid_map::HeightMap& map);

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

#endif /* RGB_CLOUD_PROCESSOR_H */