#include "height_mapping_ros/HeightMappingNode.h"

HeightMappingNode::HeightMappingNode(ros::NodeHandle& nh, ros::NodeHandle& pnh) : nh_(nh), pnh_(pnh)
{
  // Load parameters

  lidar_topic_ = pnh_.param<std::string>("lidarCloudTopic", "/height_mapping/sensor/laser/points");
  rgb_topic_ = pnh_.param<std::string>("rgbCloudTopic", "/height_mapping/sensor/color/points");
  pose_update_rate_ = pnh_.param<double>("poseUpdateRate", 20.0);
  map_publish_rate_ = pnh_.param<double>("mapPublishRate", 10.0);
  debug_mode_ = pnh_.param<bool>("debugMode", false);

  // Initialize core height mapping
  height_mapping_ = std::make_unique<HeightMapping>();

  setupROSInterface();
}

void HeightMappingNode::setupROSInterface()
{
  // Subscribers
  sub_laser_cloud_ = pnh_.subscribe(lidar_topic_, 1, &HeightMappingNode::laserCloudCallback, this);
  sub_rgb_cloud_ = pnh_.subscribe(rgb_topic_, 1, &HeightMappingNode::rgbCloudCallback, this);

  // Publishers
  pub_height_map_ = pnh_.advertise<grid_map_msgs::GridMap>("/height_mapping/map/gridmap", 1);

  // Timers
  position_update_timer_ =
      pnh_.createTimer(ros::Duration(1.0 / pose_update_rate_), &HeightMappingNode::updatePositionCallback, this);
  map_publish_timer_ =
      pnh_.createTimer(ros::Duration(1.0 / map_publish_rate_), &HeightMappingNode::publishMapCallback, this);

  // Debug publishers
  if (debug_mode_)
  {
    pub_laser_downsampled_ = pnh_.advertise<sensor_msgs::PointCloud2>("/height_mapping/debug/laser_downsampled", 1);
    pub_rgb_downsampled_ = pnh_.advertise<sensor_msgs::PointCloud2>("/height_mapping/debug/rgbd_downsampled", 1);
    pub_laser_processing_time_ =
        pnh_.advertise<jsk_rviz_plugins::OverlayText>("/height_mapping/debug/laser_processing_time", 1);
    pub_rgb_processing_time_ =
        pnh_.advertise<jsk_rviz_plugins::OverlayText>("/height_mapping/debug/rgbd_processing_time", 1);
  }
}

void HeightMappingNode::laserCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  height_mapping_->updateFromLaserCloud(msg);
}

void HeightMappingNode::rgbCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  height_mapping_->updateFromRGBCloud(msg);
}

void HeightMappingNode::updatePositionCallback(const ros::TimerEvent& event)
{
  height_mapping_->updatePosition(event);
}

void HeightMappingNode::publishMapCallback(const ros::TimerEvent& event)
{
  height_mapping_->publishHeightmap(event);
}