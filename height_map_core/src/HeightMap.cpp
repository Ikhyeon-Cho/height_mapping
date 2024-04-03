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
  add(layer_height_);
  add(layer_variance_);

  setFrameId("map");
  setGeometry(grid_map::Length(length_x, length_y), grid_resolution);
  setBasicLayers({ layer_height_ });
}

void HeightMap::addLayer(const std::string& layer, float default_value)
{
  if (!exists(layer))
    add(layer, default_value);
}

void HeightMap::deleteLayer(const std::string& layer)
{
  if (exists(layer))
    erase(layer);
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
      if (!getPosition3(layer_height_, *subiter, nearGridPosition3))
        continue;

      if ((nearGridPosition3.head<2>() - thisGridPosition).norm() < 0.05)
        continue;

      elevation_sum += nearGridPosition3.z();
      ++n_sum;
    }
    if (n_sum == 0)
      continue;

    auto& elevation = at(layer_height_, thisGrid);
    const auto& elevation_bottom = at("height_ground", thisGrid);
    // Smooth only potentially ground cell
    if (elevation - elevation_bottom > 0.15)
      continue;

    elevation = elevation_sum / n_sum;
  }
}

const std::string& HeightMap::getHeightLayer() const
{
  return layer_height_;
}

const std::string& HeightMap::getVarianceLayer() const
{
  return layer_variance_;
}

const grid_map::GridMap::Matrix& HeightMap::getHeightMatrix() const
{
  return get(layer_height_);
}

grid_map::GridMap::Matrix& HeightMap::getHeightMatrix()
{
  return get(layer_height_);
}

const grid_map::GridMap::Matrix& HeightMap::getVarianceMatrix() const
{
  return get(layer_variance_);
}

grid_map::GridMap::Matrix& HeightMap::getVarianceMatrix()
{
  return get(layer_variance_);
}

bool HeightMap::isEmptyAt(const grid_map::Index& index) const
{
  return !isValid(index);
}

bool HeightMap::isEmptyAt(const std::string& layer, const grid_map::Index& index) const
{
  return !std::isfinite(at(layer, index));
}

float HeightMap::getMaxHeight() const
{
  return HeightMapMath::getMaxVal(*this, getHeightLayer());
}

float HeightMap::getMinHeight() const
{
  return HeightMapMath::getMinVal(*this, getHeightLayer());
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

//         const auto &elevation = at(layer_height_, grid);
//         const auto &variance = at(layer_variance_, grid);
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
//             at(layer_height_, grid) = NAN;
//             at(layer_variance_, grid) = NAN;
//         }
//     }
// }

}  // namespace grid_map

float HeightMapMath::getMinVal(const grid_map::HeightMap& map, const std::string& layer)
{
  const auto& data = map[layer];

  auto fillNaNForFindingMinVal = data.array().isNaN().select(std::numeric_limits<double>::max(), data);
  return fillNaNForFindingMinVal.minCoeff();
}

float HeightMapMath::getMaxVal(const grid_map::HeightMap& map, const std::string& layer)
{
  const auto& data = map[layer];

  // https://www.geeksforgeeks.org/difference-between-stdnumeric_limitst-min-max-and-lowest-in-cpp/
  auto fillNaNForFindingMaxVal = data.array().isNaN().select(std::numeric_limits<double>::lowest(), data);
  return fillNaNForFindingMaxVal.maxCoeff();
}