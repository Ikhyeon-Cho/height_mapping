/*
 * pclProcessor.h
 *
 *  Created on: Apr 3, 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAP_PCL_PROCESSOR_H
#define HEIGHT_MAP_PCL_PROCESSOR_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <height_map_core/HeightMap.h>

namespace height_map
{
class pclProcessor
{
  struct pair_hash
  {
    template <class T1, class T2>
    std::size_t operator()(const std::pair<T1, T2>& p) const
    {
      auto h1 = std::hash<T1>{}(p.first);
      auto h2 = std::hash<T2>{}(p.second);

      return h1 ^ h2;
    }
  };

public:
  pclProcessor() = default;

  template <typename PointT>
  static typename pcl::PointCloud<PointT>::Ptr gridDownsampling(const typename pcl::PointCloud<PointT>::Ptr& cloud,
                                                                float gridSize)
  {
    // Check if the cloud is valid
    if (isEmpty(*cloud))
      return cloud;

    // Create a map to store the maximum Z value in each grid cell
    // std::map<std::pair<int, int>, PointT> grid_map;
    std::unordered_map<std::pair<int, int>, PointT, pair_hash> grid_map;
    for (const auto& point : *cloud)
    {
      // Compute the grid cell index for each point
      int x_index = std::floor(point.x / gridSize);
      int y_index = std::floor(point.y / gridSize);

      // If the grid cell is empty or the current point is higher, update the grid cell
      auto grid_key = std::make_pair(x_index, y_index);
      auto [iter, inserted] = grid_map.emplace(grid_key, point);

      if (!inserted && point.z > iter->second.z)
      {
        iter->second = point;
      }
    }

    // Add the highest point in each grid cell to the downsampled cloud
    auto cloud_downsampled = boost::make_shared<pcl::PointCloud<PointT>>();
    cloud_downsampled->reserve(grid_map.size());
    for (const auto& grid_cell : grid_map)
    {
      cloud_downsampled->push_back(grid_cell.second);
    }

    cloud_downsampled->header = cloud->header;
    return cloud_downsampled;
  }

private:
  template <typename PointT>
  static bool isEmpty(const pcl::PointCloud<PointT>& cloud)
  {
    if (cloud.empty())
    {
      std::cout << "\033[33m[ PointCloudProcessor] Warning: Input cloud is empty! \033[0m" << std::endl;
      return true;
    }
    return false;
  }
};
}  // namespace height_map

#endif /* HEIGHT_MAP_PCL_PROCESSOR_H */