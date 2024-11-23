#ifndef HEIGHT_MAPPING_NODE_H
#define HEIGHT_MAPPING_NODE_H

#include <ros/ros.h>
#include <grid_map_msgs/GridMap.h>
#include <jsk_rviz_plugins/OverlayText.h>
#include <sensor_msgs/PointCloud2.h>

#include "height_mapping_ros/HeightMapping.h"
#include "ros_utils/TransformHandler.h"

class HeightMappingNode
{
public:
  HeightMappingNode(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  ~HeightMappingNode() = default;

private:
  void laserCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
  void rgbCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
  void updatePositionCallback(const ros::TimerEvent& event);
  void publishMapCallback(const ros::TimerEvent& event);
  void setupROSInterface();

  // ROS members
  ros::NodeHandle& nh_;
  ros::NodeHandle& pnh_;
  utils::TransformHandler tf_handler_;

  // Subscribers
  ros::Subscriber sub_laser_cloud_;
  ros::Subscriber sub_rgb_cloud_;

  // Publishers
  ros::Publisher pub_height_map_;
  ros::Publisher pub_laser_downsampled_;
  ros::Publisher pub_rgb_downsampled_;
  ros::Publisher pub_laser_processing_time_;
  ros::Publisher pub_rgb_processing_time_;

  // Timers
  ros::Timer position_update_timer_;
  ros::Timer map_publish_timer_;

  // Core height mapping implementation
  std::unique_ptr<HeightMapping> height_mapping_;

  // Parameters
  std::string lidar_topic_;
  std::string rgb_topic_;
  double pose_update_rate_;
  double map_publish_rate_;
  bool debug_mode_;
};

#endif  // HEIGHT_MAPPING_ROS_H