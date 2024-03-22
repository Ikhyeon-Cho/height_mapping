/*
 * HeightMapLoader.cpp
 *
 *  Created on: Mar 21, 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "heightmap_loader/HeightMapLoader.h"

HeightMapLoader::HeightMapLoader()
{
  // Read Height map image
  cv::Mat image = cv::imread(image_path_);
  if (image.empty())
  {
    ROS_ERROR("Could not read the image");
    return;  // TODO: throw exception
  }

  // Convert to Height map
  if (!HeightMapMsgs::fromImage(image, map_))
  {
    ROS_ERROR("Could not convert the image to height map");
    return;  // TODO: throw exception
  }

  visualization_timer_.start();
}

void HeightMapLoader::visualizeHeightMap(const ros::WallTimerEvent& event)
{
  // Visualize height map
  // visualization_msgs::Marker msg_region;
  // HeightMapMsgs::toMapRegion(map_, msg_region);
  // pub_map_region_.publish(msg_region);

  grid_map_msgs::GridMap msg_map;
  grid_map::GridMapRosConverter::toMessage(map_, msg_map);
  pub_heightmap_.publish(msg_map);

  sensor_msgs::PointCloud2 msg_cloud;
  grid_map::GridMapRosConverter::toPointCloud(map_, map_.getHeightLayer() ,msg_cloud);
  pub_heightcloud_.publish(msg_cloud);

  std::cout << "test \n";
}