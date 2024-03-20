/*
 * GlobalMapping.cpp
 *
 *  Created on: Dec 2, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "height_mapping_ros/GlobalMapping.h"

GlobalMapping::GlobalMapping()
{
  map_.setFrameId(map_frame_);
  map_.setPosition(grid_map::Position(map_.getLength().x() / 2, map_.getLength().y() / 2));
}

void GlobalMapping::registerLocalMap(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  // Convert msg to pcl::PointCloud
  auto elevation_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  pcl::fromROSMsg(*msg, *elevation_cloud);

  map_.update(*elevation_cloud);
}

void GlobalMapping::visualize(const ros::TimerEvent& event)
{
  auto start = std::chrono::high_resolution_clock::now();

  // Map Visualization : grid_map_msgs types are not visualizable in global mapping
  sensor_msgs::PointCloud2 msg_pc;
  grid_map::GridMapRosConverter::toPointCloud(map_, map_.getHeightLayer(), msg_pc);
  pub_globalmap_.publish(msg_pc);

  visualization_msgs::Marker msg_map_region;
  HeightMapMsgs::toMapRegion(map_, msg_map_region);
  pub_map_region_.publish(msg_map_region);

  auto stop = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
  std::cout << "Global map visualization takes: " << duration.count() << " milliseconds" << std::endl;
}