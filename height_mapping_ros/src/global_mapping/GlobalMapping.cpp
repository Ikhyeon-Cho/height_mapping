/*
 * GlobalMapping.cpp
 *
 *  Created on: Dec 2, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "height_mapping/GlobalMapping.h"

GlobalMapping::GlobalMapping()
{
  map_.setFrameId(map_frame_);
  map_.setPosition(grid_map::Position(map_.getLength().x() / 2, map_.getLength().y() / 2));

  if (height_estimator_type_ == "KalmanFilter")
  {
    height_estimator_ = std::make_unique<height_map::KalmanEstimator>();
    ROS_INFO("[GlobalMapping] Height estimator: KalmanFilter");
  }
  else if (height_estimator_type_ == "MovingAverage")
  {
    height_estimator_ = std::make_unique<height_map::MovingAverageEstimator>();
    ROS_INFO("[GlobalMapping] Height estimator: MovingAverageFilter");
  }
  else if (height_estimator_type_ == "SimpleMean")
  {
    height_estimator_ = std::make_unique<height_map::SimpleMeanEstimator>();
    ROS_INFO("[GlobalMapping] Height estimator: SimpleMeanFilter");
  }
  else
  {
    ROS_WARN("[GlobalMapping] Invalid height estimator type. Set Default: SimpleMean");
    height_estimator_ = std::make_unique<height_map::SimpleMeanEstimator>();
  }

  heightmap_cloud_ = boost::make_shared<pcl::PointCloud<PointT>>();
}

void GlobalMapping::mapping(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  // Convert msg to pcl::PointCloud
  pcl::fromROSMsg(*msg, *heightmap_cloud_);

  // Update global map
  height_estimator_->estimate(map_, *heightmap_cloud_);
}

void GlobalMapping::visualize(const ros::TimerEvent& event)
{
  auto start = std::chrono::high_resolution_clock::now();

  auto layer_to_visualize = { map_.getHeightLayer() };

  auto map_cloud = toPointCloudMap(map_);
  sensor_msgs::PointCloud2 map_cloud_msg;
  pcl::toROSMsg(*map_cloud, map_cloud_msg);
  pub_globalmap_.publish(map_cloud_msg);

  // Note: grid_map_msgs are not visualizable in large scale map -> use PointCloud2 instead
  // sensor_msgs::PointCloud2 msg_pc;
  // grid_map::GridMapRosConverter::toPointCloud(map_, map_.getHeightLayer(), msg_pc);
  // pub_globalmap_.publish(msg_pc);

  // Visualize map region
  visualization_msgs::Marker msg_map_region;
  HeightMapMsgs::toMapRegion(map_, msg_map_region);
  pub_map_region_.publish(msg_map_region);

  auto stop = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
  std::cout << "Global map visualization takes: " << duration.count() << " milliseconds" << std::endl;
}

pcl::PointCloud<PointT>::Ptr GlobalMapping::toPointCloudMap(const grid_map::HeightMap& map)
{
  pcl::PointCloud<PointT>::Ptr map_cloud{ boost::make_shared<pcl::PointCloud<PointT>>() };
  map_cloud->header.frame_id = map.getFrameId();

  std::vector<grid_map::Index> measured_index_list;
  const auto& height_matrix = map.getHeightMatrix();
  for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator)
  {
    const size_t i = iterator.getLinearIndex();
    const auto& is_valid = std::isfinite(height_matrix(i));
    if (!is_valid)
      continue;

    measured_index_list.push_back(*iterator);
  }

  for (const auto& cell_index : measured_index_list)
  {
    grid_map::Position3 cell_position3;
    map_.getPosition3(map_.getHeightLayer(), cell_index, cell_position3);

    PointT point;
    point.x = cell_position3.x();
    point.y = cell_position3.y();
    point.z = cell_position3.z();
    // point.intensity = map_["intensity"](cell_index(0), cell_index(1));

    map_cloud->push_back(point);
  }

  return map_cloud;
}

std::pair<bool, pcl::PointCloud<PointT>::Ptr>
GlobalMapping::downsampleCloud(const pcl::PointCloud<PointT>::ConstPtr& cloud)
{
  // Check if the cloud is empty
  if (cloud->empty())
  {
    ROS_WARN("[GlobalMapping]: Empty cloud received.");
    return { false, nullptr };
  }

  // Check if the cloud is in the same frame as the map
  if (cloud->header.frame_id != map_.getFrameId())
  {
    ROS_WARN("[GlobalMapping]: Frame ID mismatch - pointcloud is in a different frame!");
    return { false, nullptr };
  }

  map_.addLayer("n_measured", 0.0);
  map_.clear(map_.getHeightLayer());
  auto& height_matrix = map_.getHeightMatrix();
  auto& n_measured_matrix = map_["n_measured"];

  std::vector<grid_map::Index> measured_index_list;
  grid_map::Index cell_index;
  grid_map::Position cell_position;
  for (const auto& point : *cloud)
  {
    cell_position << point.x, point.y;
    if (!map_.getIndex(cell_position, cell_index))
      continue;

    if (n_measured_matrix(cell_index(0), cell_index(1)) > 10)
      continue;

    if (map_.isEmptyAt(map_.getHeightLayer(), cell_index))
    {
      height_matrix(cell_index(0), cell_index(1)) = point.z;
      n_measured_matrix(cell_index(0), cell_index(1)) = 1;
      measured_index_list.push_back(cell_index);
    }
    else
    {
      auto& height = height_matrix(cell_index(0), cell_index(1));
      height = 0.9 * height + 0.1 * point.z;
      n_measured_matrix(cell_index(0), cell_index(1)) += 1;
    }
  }

  auto cloud_downsampled = boost::make_shared<pcl::PointCloud<PointT>>();
  cloud_downsampled->header = cloud->header;
  cloud_downsampled->reserve(measured_index_list.size());

  grid_map::Position3 cell_position3;
  PointT point;
  for (const auto& cell_index : measured_index_list)
  {
    map_.getPosition3(map_.getHeightLayer(), cell_index, cell_position3);

    point.x = cell_position3.x();
    point.y = cell_position3.y();
    point.z = cell_position3.z();

    cloud_downsampled->push_back(point);
  }
  return { true, cloud_downsampled };
}

bool GlobalMapping::clearMap(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  map_.clearAll();
  return true;
}

bool GlobalMapping::saveLayerToImage(height_map_msgs::SaveLayerToImage::Request& req,
                                     height_map_msgs::SaveLayerToImage::Response& res)
{
  // Srv input
  auto img_path = file_save_path_ + "/" + req.map_name + "/";
  const auto& layer = req.layer_name;

  // Check if the directory exists and create if not
  std::filesystem::path output_path(img_path);
  if (output_path.has_parent_path())
    std::filesystem::create_directories(output_path.parent_path());

  if (layer == "")  // Save all layers to image
  {
    for (const auto& layer : map_.getLayers())
    {
      if (!saveMapToImage(layer, img_path))
      {
        res.success = false;
        return false;
      }
    }
    res.success = true;
    res.img_name = img_path + "**.png";
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
    res.img_name = img_path + layer + ".png";
    return true;
  }
}

bool GlobalMapping::saveMapToImage(const std::string& layer, const std::string& img_path)
{
  const auto& data_matrix = map_[layer];

  if (data_matrix.rows() == 0 || data_matrix.cols() == 0)
  {
    ROS_ERROR("Map has zero size. Skip map saver service");
    return false;
  }

  // Convert to cv image
  cv::Mat image;
  auto min_val = HeightMapMath::getMinVal(map_, layer);
  auto max_val = HeightMapMath::getMaxVal(map_, layer);
  if (!HeightMapConverter::toGrayImage(map_, layer, image, min_val, max_val))
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
  out << YAML::Key << "Grid Resolution" << YAML::Value << map_.getResolution();
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