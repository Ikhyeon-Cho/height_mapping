/*
 * GlobalMapping.cpp
 *
 *  Created on: Dec 2, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "height_mapping_ros/GlobalMapping.h"

template <typename PointT>
GlobalMapping<PointT>::GlobalMapping()
{
  globalmap_.setFrameId(map_frame_);
  globalmap_.setPosition(grid_map::Position(0.0, 0.0));
  // globalmap_.setPosition(grid_map::Position(globalmap_.getLength().x() / 2, globalmap_.getLength().y() / 2));

  measured_indices_.reserve(globalmap_.getSize().prod());

  if (height_estimator_type_ == "KalmanFilter")
  {
    height_estimator_ = std::make_unique<height_map::KalmanEstimator>();
  }
  else if (height_estimator_type_ == "MovingAverage")
  {
    height_estimator_ = std::make_unique<height_map::MovingAverageEstimator>();
  }
  else if (height_estimator_type_ == "StatMean")
  {
    height_estimator_ = std::make_unique<height_map::StatMeanEstimator>();
  }
  else
  {
    height_estimator_ = std::make_unique<height_map::StatMeanEstimator>();
  }

  if (debug_)
  {
    pub_processing_time_ =
        nh_.advertise<jsk_rviz_plugins::OverlayText>("/height_mapping/debug/globalmap/vis_processing_time", 1);
  }

  std::cout << "\033[1;32m[HeightMapping::GlobalMapping]: Waiting for Height map... \033[0m\n";
}

template <typename PointT>
void GlobalMapping<PointT>::visualize(const ros::TimerEvent& event)
{
  // Timer for benchmarking
  auto start = std::chrono::high_resolution_clock::now();

  // Note: grid_map_msgs are not visualizable in large scale map -> use PointCloud2 instead
  sensor_msgs::PointCloud2 cloud_msg;
  toPointCloud2(globalmap_, globalmap_.getLayers(), measured_indices_, cloud_msg);
  pub_globalmap_.publish(cloud_msg);

  // Visualize map region
  visualization_msgs::Marker msg_map_region;
  HeightMapMsgs::toMapRegion(globalmap_, msg_map_region);
  pub_map_region_.publish(msg_map_region);

  auto stop = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);

  jsk_rviz_plugins::OverlayText overlay_text;
  overlay_text.height = 100;
  overlay_text.width = 300;
  overlay_text.text = "Visualization processing Time: " + std::to_string(duration.count()) + " ms";
  // pub_processing_time_.publish(
  // HeightMapMsgs::toOverlayText("Processing Time: " + std::to_string(duration.count()) + " ms"));
  pub_processing_time_.publish(overlay_text);
}

template <typename PointT>
void GlobalMapping<PointT>::addMeasuredGridIndices(const grid_map::HeightMap& map, const pcl::PointCloud<PointT>& cloud)
{
  // Save measured indices for efficient visualization
  grid_map::Index cell_index;
  grid_map::Position cell_position;
  for (auto& point : cloud.points)
  {
    // Skip if the point is out of the map
    cell_position << point.x, point.y;
    if (!map.getIndex(cell_position, cell_index))
      continue;

    if (!map.isEmptyAt(cell_index))
      continue;

    measured_indices_.insert(cell_index);
  }
}

template <typename PointT>
bool GlobalMapping<PointT>::clearMap(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  globalmap_.clearAll();
  return true;
}

template <typename PointT>
bool GlobalMapping<PointT>::saveLayerToImage(height_map_msgs::SaveLayerToImage::Request& req,
                                             height_map_msgs::SaveLayerToImage::Response& res)
{
  // Srv input
  auto img_path = std::filesystem::path(map_save_directory_) / req.map_name;
  const auto& layer = req.layer_name;

  // Check if the directory exists and create if not
  std::filesystem::path output_path(img_path);
  if (output_path.has_parent_path())
    std::filesystem::create_directories(output_path.parent_path());

  if (layer == "")  // Save all layers to image
  {
    for (const auto& layer : globalmap_.getLayers())
    {
      if (!saveMapToImage(layer, img_path))
      {
        res.success = false;
        return false;
      }
    }
    res.success = true;
    res.img_name = (img_path / "**.png").string();
    return true;
  }

  else  // Save a single layer to image
  {
    if (!saveMapToImage(layer, img_path))
    {
      res.success = false;
      return false;
    }
    res.success = true;
    res.img_name = (img_path / (layer + ".png")).string();
    return true;
  }
}

template <typename PointT>
bool GlobalMapping<PointT>::saveMapToImage(const std::string& layer, const std::string& img_path)
{
  const auto& data_matrix = globalmap_[layer];

  if (data_matrix.rows() == 0 || data_matrix.cols() == 0)
  {
    ROS_ERROR("Map has zero size. Skip map saver service");
    return false;
  }

  // Convert to cv image
  cv::Mat image;
  auto min_val = HeightMapMath::getMinVal(globalmap_, layer);
  auto max_val = HeightMapMath::getMaxVal(globalmap_, layer);
  if (!HeightMapConverter::toGrayImage(globalmap_, layer, image, min_val, max_val))
  {
    ROS_ERROR("Failed to convert %s data to image.", layer.c_str());
    return false;
  }

  // Write cv image to a png file
  if (!cv::imwrite(img_path + layer + ".png", image))
  {
    ROS_ERROR("Failed to write image to %s.", img_path.c_str());
    return false;
  }

  // Write metadata to a YAML file
  std::string yaml_path = img_path + layer + ".yaml";
  YAML::Emitter out;
  out << YAML::BeginMap;
  out << YAML::Key << "Layer" << YAML::Value << layer;
  out << YAML::Key << "Grid Resolution" << YAML::Value << globalmap_.getResolution();
  out << YAML::Key << "Min" << YAML::Value << min_val;
  out << YAML::Key << "Max" << YAML::Value << max_val;
  out << YAML::EndMap;

  std::ofstream fout(yaml_path);
  fout << out.c_str();
  fout.close();

  // // Write a single metadata.txt: layer, grid resolution, min, max values
  // std::ofstream metadata;
  // metadata.open(img_path + +"_metadata.txt", std::ios_base::app);
  // metadata << "Layer: " << layer << std::endl;
  // metadata << "Grid Resolution: " << map_.getResolution() << std::endl;
  // metadata << "Min: " << min_val << std::endl;
  // metadata << "Max: " << max_val << std::endl << std::endl;
  // metadata.close();

  return true;
}

template <typename PointT>
void GlobalMapping<PointT>::toPointCloud2(
    const grid_map::HeightMap& map, const std::vector<std::string>& layers,
    const std::unordered_set<grid_map::Index, IndexHash, IndexEqual>& measured_indices, sensor_msgs::PointCloud2& cloud)
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
        pointFieldValue = (float)(map.at("color", measured_index));
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

template class GlobalMapping<pcl::PointXYZRGB>;
template class GlobalMapping<pcl::PointXYZI>;