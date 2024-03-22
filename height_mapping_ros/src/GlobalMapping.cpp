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

void GlobalMapping::registerLocalMap(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  auto start = std::chrono::high_resolution_clock::now();

  // Convert msg to pcl::PointCloud
  auto elevation_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  pcl::fromROSMsg(*msg, *elevation_cloud);

  map_.update(*elevation_cloud);

  // HeightMapConverter::fromPointCloud(*msg, map_);

  auto stop = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  std::cout << "Global map update takes: " << duration.count() << " microseconds" << std::endl;
}

void GlobalMapping::visualize(const ros::TimerEvent& event)
{
  auto start = std::chrono::high_resolution_clock::now();

  auto layer_to_visualize = { map_.getHeightLayer(), std::string("intensity") };
  // Map Visualization : grid_map_msgs types are not visualizable in global mapping
  sensor_msgs::PointCloud2 msg_pc;
  grid_map::GridMapRosConverter::toPointCloud(map_, layer_to_visualize, map_.getHeightLayer(), msg_pc);
  pub_globalmap_.publish(msg_pc);

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

bool GlobalMapping::saveLayerToImage(height_map_msgs::SaveGridMapLayer::Request& req,
                                     height_map_msgs::SaveGridMapLayer::Response& res)
{
  const auto& layer = map_[req.layer_name];
  const auto& file_path = file_save_path_ + "/" + req.filename;
  res.file_path = file_path;

  if (layer.rows() == 0 || layer.cols() == 0)
  {
    ROS_WARN("Layer %s has zero size.", req.layer_name.c_str());
    return false;
  }



  cv::Mat image;
  bool success = HeightMapConverter::toGrayImage(map_, req.layer_name, image, -0.1, 10);
  
  // bool success = grid_map::GridMapCvConverter::toImage<unsigned char, 1>(map_, req.layer_name, CV_8UC1, -20, 20, image);

  if (!success)
  {
    ROS_ERROR("Failed to convert layer to image.");
    res.success = false;
    return false;
  }

  // Write image to file
  if (!cv::imwrite(file_path, image))
  {
    ROS_ERROR("Failed to write image to file.");
    res.success = false;
    return false;
  }

  res.success = true;
  return true;
}