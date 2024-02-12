/*
 * GlobalTerrainMapping.cpp
 *
 *  Created on: Dec 2, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "terrain_mapping/GlobalTerrainMapping.h"
#include <sensor_msgs/point_cloud2_iterator.h>

namespace ros
{
GlobalTerrainMapping::GlobalTerrainMapping()
{
  map_visualization_timer.start();
}

void GlobalTerrainMapping::updateMap(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  auto cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  pcl::fromROSMsg(*msg, *cloud);  // moveFromROSMsg is a bit faster (100 ~ 200 us) than fromROSMsg

  bool has_transformToBase;
  auto base_cloud = pointcloud_processor_.transformPointcloud(cloud, frameId_robot.param(), has_transformToBase);
  if (!has_transformToBase)
    return;

  // Filter Pointcloud: only use short-range data
  auto height_filtered_cloud =
      pointcloud_processor_.filterPointcloudByAxis(base_cloud, "z", min_height_thres.param(), max_height_thres.param());
      
  auto local_cloud = pointcloud_processor_.filterPointcloudByRange2D(height_filtered_cloud, min_range_thres.param(),
                                                                     max_range_thres.param());

  bool has_transformToMap;
  auto registered_cloud = pointcloud_processor_.transformPointcloud(local_cloud, map_.getFrameId(), has_transformToMap);
  if (!has_transformToMap)
    return;

  // Update Elevation map
  map_.update(*registered_cloud);  // takes about 150 ms at 400 x 400 size

  // Update Descriptor map
  descriptor_map_.setElevationMap(map_);
  descriptor_map_.update(*registered_cloud);  // only update values at measured grid

  saveMeasuredIndices(*registered_cloud);
}

void GlobalTerrainMapping::visualize(const ros::TimerEvent& event)
{
  // Map Visualization : grid_map_msgs types are not visualizable in global mapping
  // sensor_msgs::PointCloud2 msg_pc;
  // grid_map::GridMapRosConverter::toPointCloud(descriptor_map_, "elevation", msg_pc);
  // elevation_map_publisher.publish(msg_pc);

  visualizeGlobalMap();

  visualization_msgs::Marker msg_gridmap_boundary;
  ElevationMapRosConverter::toMapBoundaryMsg(map_, msg_gridmap_boundary);
  map_boundary_publisher.publish(msg_gridmap_boundary);

  // nav_msgs::OccupancyGrid msg_map;
  // ElevationMapRosConverter::toOccupancyGridMsg(map_, msg_map);
  // occupancy_map_publisher.publish(msg_map);
}

void GlobalTerrainMapping::saveMeasuredIndices(const pcl::PointCloud<pcl::PointXYZI>& pointcloud)
{
  grid_map::Index index;
  for (const auto& point : pointcloud)
  {
    if (!map_.getIndex(grid_map::Position(point.x, point.y), index))
      continue;

    measured_index_set_.insert(index);
  }
}

void GlobalTerrainMapping::visualizeGlobalMap()
{
  sensor_msgs::PointCloud2 pointCloudMsg;
  // Header.
  pointCloudMsg.header.frame_id = descriptor_map_.getFrameId();
  pointCloudMsg.header.stamp.fromNSec(descriptor_map_.getTimestamp());
  pointCloudMsg.is_dense = false;

  // Fields.
  std::vector<std::string> fieldNames;

  for (const auto& layer : descriptor_map_.getLayers())
  {
    if (layer == "elevation")
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

  pointCloudMsg.fields.clear();
  pointCloudMsg.fields.reserve(fieldNames.size());
  int offset = 0;

  for (auto& name : fieldNames)
  {
    sensor_msgs::PointField pointField;
    pointField.name = name;
    pointField.count = 1;
    pointField.datatype = sensor_msgs::PointField::FLOAT32;
    pointField.offset = offset;
    pointCloudMsg.fields.push_back(pointField);
    offset = offset + pointField.count * 4;  // 4 for sensor_msgs::PointField::FLOAT32
  }

  // Resize.
  pointCloudMsg.height = 1;
  pointCloudMsg.width = measured_index_set_.size();
  pointCloudMsg.point_step = offset;
  pointCloudMsg.row_step = pointCloudMsg.width * pointCloudMsg.point_step;
  pointCloudMsg.data.resize(pointCloudMsg.height * pointCloudMsg.row_step);

  // Points.
  std::unordered_map<std::string, sensor_msgs::PointCloud2Iterator<float>> fieldIterators;
  for (auto& name : fieldNames)
  {
    fieldIterators.insert(std::pair<std::string, sensor_msgs::PointCloud2Iterator<float>>(
        name, sensor_msgs::PointCloud2Iterator<float>(pointCloudMsg, name)));
  }

  const bool hasBasicLayers = descriptor_map_.getBasicLayers().size() > 0;
  for (const auto& index : measured_index_set_)
  {
    grid_map::Position3 position;
    if (!descriptor_map_.getPosition3("elevation", index, position))
    {
      continue;
    }

    for (auto& iterator : fieldIterators)
    {
      if (iterator.first == "x")
      {
        *iterator.second = (float)position.x();
      }
      else if (iterator.first == "y")
      {
        *iterator.second = (float)position.y();
      }
      else if (iterator.first == "z")
      {
        *iterator.second = (float)position.z();
      }
      else if (iterator.first == "rgb")
      {
        *iterator.second = descriptor_map_.at("color", index);
      }
      else
      {
        *iterator.second = descriptor_map_.at(iterator.first, index);
      }
    }

    for (auto& iterator : fieldIterators)
    {
      ++iterator.second;
    }
  }

  pointCloudMsg.width = measured_index_set_.size();
  pointCloudMsg.row_step = pointCloudMsg.width * pointCloudMsg.point_step;
  pointCloudMsg.data.resize(pointCloudMsg.height * pointCloudMsg.row_step);

  // publish
  elevation_map_publisher.publish(pointCloudMsg);
}

}  // namespace ros