/*
 * HeightMap.h
 *
 *  Created on: Aug 17, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAP_H
#define HEIGHT_MAP_H

#include <grid_map_core/grid_map_core.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/eigen.h>

namespace grid_map
{
class HeightMap : public GridMap
{
public:
  HeightMap(double length_x, double length_y, double grid_resolution);

  void addLayer(const std::string& layer, float default_value = NAN);
  void deleteLayer(const std::string& layer);

  const std::string& getHeightLayer() const;
  const std::string& getVarianceLayer() const;

  const GridMap::Matrix& getHeightMatrix() const;
  GridMap::Matrix& getHeightMatrix();

  const GridMap::Matrix& getVarianceMatrix() const;
  GridMap::Matrix& getVarianceMatrix();

  bool isEmptyAt(const Index& index) const;
  bool isEmptyAt(const std::string& layer, const Index& index) const;

  void smoothing();

  float getMaxHeight() const;
  float getMinHeight() const;

private:
  // Basic Layers
  std::string layer_height_{ "elevation" };
  std::string layer_variance_{ "variance" };
};

}  // namespace grid_map

class HeightMapMath
{
public:
  static float getMinVal(const grid_map::HeightMap& map, const std::string& layer);

  static float getMaxVal(const grid_map::HeightMap& map, const std::string& layer);
};

#endif  // HEIGHT_MAP_H