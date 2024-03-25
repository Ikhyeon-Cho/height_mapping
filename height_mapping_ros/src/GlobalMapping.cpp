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
}

void GlobalMapping::updateGlobalMap(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  auto start = std::chrono::high_resolution_clock::now();

  // // Convert msg to pcl::PointCloud
  // auto elevation_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  // pcl::fromROSMsg(*msg, *elevation_cloud);
  // // Update global map
  // map_.update(*elevation_cloud);

  HeightMapConverter::fromPointCloud(*msg, map_);

  auto stop = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  std::cout << "Global map update takes: " << duration.count() << " microseconds" << std::endl;
}

void GlobalMapping::visualize(const ros::TimerEvent& event)
{
  auto start = std::chrono::high_resolution_clock::now();

  auto layer_to_visualize = { map_.getHeightLayer(), std::string("intensity") };

  // Note: grid_map_msgs are not visualizable in large size -> use PointCloud2 instead
  sensor_msgs::PointCloud2 msg_pc;
  grid_map::GridMapRosConverter::toPointCloud(map_, layer_to_visualize, map_.getHeightLayer(), msg_pc);
  pub_globalmap_.publish(msg_pc);

  // Visualize map region
  visualization_msgs::Marker msg_map_region;
  HeightMapMsgs::toMapRegion(map_, msg_map_region);
  pub_map_region_.publish(msg_map_region);

  auto stop = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
  std::cout << "Global map visualization takes: " << duration.count() << " milliseconds" << std::endl;
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