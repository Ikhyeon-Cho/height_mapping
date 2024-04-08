/*
 * HeightMapConverter.cpp
 *
 *  Created on: Mar 21, 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "height_map_msgs/HeightMapConverter.h"

bool HeightMapConverter::toGrayImage(const grid_map::HeightMap& map, const std::string& layer, cv::Mat& img,
                                     float min_value, float max_value)
{
  auto map_copy = map;
  auto& data = map_copy[layer];

  // Apply the min-max cutoff
  data = data.unaryExpr([min_value, max_value](float val) {
    if (val < min_value)
      return min_value;
    else if (val > max_value)
      return max_value;
    else
      return val;
  });

  auto scaler = (VALID_PIXEL_MAX - VALID_PIXEL_MIN) / (max_value - min_value);
  auto offset = (VALID_PIXEL_MIN * max_value - VALID_PIXEL_MAX * min_value) / (max_value - min_value);

  data = data * scaler + Eigen::MatrixXf::Constant(data.rows(), data.cols(), offset);

  // Fill NaN values with 0
  data = data.array().isNaN().select(0, data);

  // Convert to Image
  bool success = grid_map::GridMapCvConverter::toImage<unsigned char, 1>(map_copy, layer, CV_8UC1, 0, 255, img);
  if (!success)
  {
    std::cout << "Could not convert the height map to image" << std::endl;
    return false;
  }
  return true;
}

bool HeightMapConverter::fromGrayImage(const cv::Mat& image, float min_value, float max_value, grid_map::HeightMap& map,
                                       const std::string& layer)
{
  // Set geometry of the map
  if (map.getSize().x() != image.cols || map.getSize().y() != image.rows)
  {
    bool success =
        grid_map::GridMapCvConverter::initializeFromImage(image, map.getResolution(), map, map.getPosition());
    if (!success)
      return false;
  }

  // Convert to Height map
  if (!grid_map::GridMapCvConverter::addLayerFromImage<unsigned char, 1>(image, layer, map, min_value, max_value))
    return false;

  // Change the values lower than invalid_pixel_cutoff to NaN
  auto invalid_pixel_cutoff = min_value + 1e-2;
  map[layer] = map[layer].unaryExpr([invalid_pixel_cutoff](float val) {
    if (val < invalid_pixel_cutoff)
      return std::numeric_limits<float>::quiet_NaN();
    else
      return val;
  });

  return true;
}

void HeightMapConverter::fromPointCloud2(const sensor_msgs::PointCloud2& cloud, grid_map::HeightMap& map)
{
  // Assuming gridMap is already initialized with desired resolution and size
  // 1. Extract the layers from the point cloud
  std::vector<std::string> layers;  // except x, y, z, rgb
  bool has_rgb = false;
  for (const auto& field : cloud.fields)
  {
    if (field.name == "x" || field.name == "y" || field.name == "z")
      continue;

    // Check for the presence of the RGB field.
    if (field.name == "rgb" && field.datatype == sensor_msgs::PointField::UINT32)
    {
      has_rgb = true;
      continue;
    }

    layers.push_back(field.name);
  }

  // 2. Initialize the layers in the grid map
  for (const auto& layer : layers)
  {
    if (!map.exists(layer))
    {
      map.add(layer);
      std::cout << "[@ HeightMapConverter] Added " << layer << " layer to the height map" << std::endl;
    }
  }
  if (has_rgb)
  {
    if (!map.exists("r"))
    {
      map.add("r");
      map.add("g");
      map.add("b");
      std::cout << "[@ HeightMapConverter] Added RGB layers to the height map" << std::endl;
    }
  }

  // 3. Iterate through the point cloud and populate the grid map
  // Initialize iterators
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

  std::vector<std::tuple<std::string, sensor_msgs::PointCloud2ConstIterator<float>>> iterator_fields;
  for (const auto& layer : layers)
    iterator_fields.emplace_back(layer, sensor_msgs::PointCloud2ConstIterator<float>(cloud, layer));

  // Populate the grid map
  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
  {
    grid_map::Position position(*iter_x, *iter_y);  // Create position from x, y values

    // Check if the position falls within the grid map's bounds
    if (!map.isInside(position))
      continue;

    map.atPosition(map.getHeightLayer(), position) = *iter_z;

    for (auto& iter_field : iterator_fields)
    {
      auto& [layer, iter] = iter_field;
      map.atPosition(layer, position) = *iter;
      ++iter;
    }
  }

  // Populate the RGB layers
  if (!has_rgb)
    return;
  sensor_msgs::PointCloud2ConstIterator<float> iter_x2(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y2(cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<uint32_t> iter_rgb(cloud, "rgb");

  for (; iter_x2 != iter_x2.end(); ++iter_x2, ++iter_y2)
  {
    grid_map::Position position(*iter_x2, *iter_y2);  // Create position from x, y values

    // Check if the position falls within the grid map's bounds
    if (!map.isInside(position))
      continue;

    auto rgb_packed = *iter_rgb;
    uint8_t r, g, b;
    unpackRgb(rgb_packed, r, g, b);
    map.atPosition("r", position) = r;
    map.atPosition("g", position) = g;
    map.atPosition("b", position) = b;
  }
}

void HeightMapConverter::toPointCloud2(const grid_map::HeightMap& map, sensor_msgs::PointCloud2& cloud)
{
  // Convert to PointCloud
  grid_map::GridMapRosConverter::toPointCloud(map, map.getHeightLayer(), cloud);
}

void HeightMapConverter::toPointCloud2(const grid_map::HeightMap& map, const std::vector<std::string>& layers,
                                       sensor_msgs::PointCloud2& cloud)
{
  // Convert to PointCloud
  grid_map::GridMapRosConverter::toPointCloud(map, layers, map.getHeightLayer(), cloud);
}

void HeightMapConverter::toPointCloud2(const grid_map::HeightMap& map, const std::vector<std::string>& layers,
                                       const std::vector<grid_map::Index>& measured_indices,
                                       sensor_msgs::PointCloud2& cloud)
{
  // Header.
  cloud.header.frame_id = map.getFrameId();
  cloud.header.stamp.fromNSec(map.getTimestamp());
  cloud.is_dense = false;

  // Fields.
  std::vector<std::string> fieldNames;
  for (const auto& layer : layers)
  {
    if (layer == map.getHeightLayer())
    {
      fieldNames.push_back("x");
      fieldNames.push_back("y");
      fieldNames.push_back("z");
    }
    else if (layer == "color")
    {
      fieldNames.push_back("rgb");
    }
    else
    {
      fieldNames.push_back(layer);
    }
  }

  cloud.fields.clear();
  cloud.fields.reserve(fieldNames.size());
  int offset = 0;

  for (auto& name : fieldNames)
  {
    sensor_msgs::PointField pointField;
    pointField.name = name;
    pointField.count = 1;
    pointField.datatype = sensor_msgs::PointField::FLOAT32;
    pointField.offset = offset;
    cloud.fields.push_back(pointField);
    offset = offset + pointField.count * 4;  // 4 for sensor_msgs::PointField::FLOAT32
  }                                          // offset value goes from 0, 4, 8, 12, ...

  // Adjusted Resize: Instead of maxNumberOfPoints, use measured_indices.size().
  size_t numberOfMeasuredPoints = measured_indices.size();
  cloud.height = 1;
  cloud.width = numberOfMeasuredPoints;  // Use the size of measured_indices.
  cloud.point_step = offset;
  cloud.row_step = cloud.width * cloud.point_step;
  cloud.data.resize(cloud.height * cloud.row_step);

  // Adjust Points section to iterate over measured_indices.
  std::unordered_map<std::string, sensor_msgs::PointCloud2Iterator<float>> pointFieldIterators;
  for (auto& name : fieldNames)
  {
    pointFieldIterators.insert(std::make_pair(name, sensor_msgs::PointCloud2Iterator<float>(cloud, name)));
  }

  // Iterate over measured_indices instead of using GridMapIterator.
  int count = 0;
  for (const auto& measured_index : measured_indices)
  {
    grid_map::Position3 position;
    if (!map.getPosition3(map.getHeightLayer(), measured_index, position))
      continue;

    const auto& cell_position_x = (float)position.x();
    const auto& cell_position_y = (float)position.y();
    const auto& cell_height = (float)position.z();

    for (auto& pointFieldIterator : pointFieldIterators)
    {
      const auto& pointField = pointFieldIterator.first;
      auto& pointFieldValue = *(pointFieldIterator.second);
      if (pointField == "x")
      {
        pointFieldValue = cell_position_x;
      }
      else if (pointField == "y")
      {
        pointFieldValue = cell_position_y;
      }
      else if (pointField == "z")
      {
        pointFieldValue = cell_height;
      }
      else if (pointField == "rgb")
      {
        pointFieldValue = (float) (map.at("color", measured_index));
      }
      else
      {
        pointFieldValue = (float)(map.at(pointField, measured_index));
      }
    }

    for (auto& pointFieldIterator : pointFieldIterators)
    {
      ++(pointFieldIterator.second);
    }
    ++count;
  }
  cloud.height = 1;
  cloud.width = count;
  cloud.point_step = offset;
  cloud.row_step = cloud.width * cloud.point_step;
  cloud.data.resize(cloud.height * cloud.row_step);
}

void unpackRgb(uint32_t rgb_packed, uint8_t& r, uint8_t& g, uint8_t& b)
{
  r = (rgb_packed >> 16) & 0xFF;
  g = (rgb_packed >> 8) & 0xFF;
  b = rgb_packed & 0xFF;
}