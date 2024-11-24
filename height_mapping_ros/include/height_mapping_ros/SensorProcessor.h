/*
 * SensorProcessor.h
 *
 *  Created on: Apr 29, 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef SENSOR_PROCESSOR_H
#define SENSOR_PROCESSOR_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include "utils/TransformHandler.h"
#include "utils/pointcloud.h"

template <typename PointT>
class SensorProcessor
{
public:
  SensorProcessor();

  // Currently, maximum 3 sensor inputs are supported
  void cloudCallback1(const sensor_msgs::PointCloud2ConstPtr& msg);
  void cloudCallback2(const sensor_msgs::PointCloud2ConstPtr& msg);
  void cloudCallback3(const sensor_msgs::PointCloud2ConstPtr& msg);

  void publishCloud(const ros::TimerEvent& event);

  /// @brief Templated function to process pointcloud
  /// @param cloud_raw input pointcloud
  /// @param cloud_processed output pointcloud in base_link frame. If the processing fails, return input pointcloud
  /// @return true if the processing is successful
  bool processCloud(const typename pcl::PointCloud<PointT>::Ptr& cloud_raw,
                    typename pcl::PointCloud<PointT>::Ptr& cloud_processed)
  {
    // Do voxel grid downsampling
    typename pcl::PointCloud<PointT>::Ptr cloud_downsampled;
    if (downsampling_resolution_ > 1e-3)
      cloud_downsampled = utils::pcl::filterPointcloudByVoxel<PointT>(cloud_raw, downsampling_resolution_);
    else
      cloud_downsampled = cloud_raw;

    // Transform pointcloud to base_link: required for range-based filtering
    auto sensor_frame = cloud_raw->header.frame_id;
    auto [get_transform_s2b, sensor_to_baselink] = tf_tree_.getTransform(sensor_frame, baselink_frame);
    if (!get_transform_s2b)
      return false;

    auto cloud_at_baselink = utils::pcl::transformPointcloud<PointT>(cloud_downsampled, sensor_to_baselink);

    // Range filter
    auto cloud_filtered =
        utils::pcl::filterPointcloudByRange<PointT>(cloud_at_baselink, range_min_thrsh_, range_max_thrsh_);

    cloud_processed = cloud_filtered;
    return true;
  }

private:
  ros::NodeHandle nh_priv_{ "~" };
  utils::TransformHandler tf_tree_;

  // Topics
  std::string inputcloud_topic_1_{ nh_priv_.param<std::string>("inputCloudTopic1", "/sensor1/points") };
  std::string inputcloud_topic_2_{ nh_priv_.param<std::string>("inputCloudTopic2", "/sensor2/points") };
  std::string inputcloud_topic_3_{ nh_priv_.param<std::string>("inputCloudTopic3", "/sensor3/points") };
  std::string processed_cloud_topic_{ nh_priv_.param<std::string>("outputTopic", "points") };

  // Frame Ids
  std::string baselink_frame{ nh_priv_.param<std::string>("/frame_id/base_link", "base_link") };
  std::string odom_frame{ nh_priv_.param<std::string>("/frame_id/odom", "odom") };
  std::string map_frame{ nh_priv_.param<std::string>("/frame_id/map", "map") };

  // Pointcloud Preprocessing Parameters
  double downsampling_resolution_{ nh_priv_.param<double>("downsamplingResolution", 0.0) };
  double range_min_thrsh_{ nh_priv_.param<double>("minRangeThreshold", 0.3) };
  double range_max_thrsh_{ nh_priv_.param<double>("maxRangeThreshold", 10.0) };

  // Timer
  double cloud_pub_rate_{ nh_priv_.param<double>("cloudPublishRate", 15.0) };

  // ROS
  ros::Subscriber sub_cloud_1_{ nh_priv_.subscribe(inputcloud_topic_1_, 1, &SensorProcessor::cloudCallback1, this) };
  ros::Subscriber sub_cloud_2_{ nh_priv_.subscribe(inputcloud_topic_2_, 1, &SensorProcessor::cloudCallback2, this) };
  ros::Subscriber sub_cloud_3_{ nh_priv_.subscribe(inputcloud_topic_3_, 1, &SensorProcessor::cloudCallback3, this) };
  ros::Publisher pub_cloud_{ nh_priv_.advertise<sensor_msgs::PointCloud2>(processed_cloud_topic_, 1) };

  ros::Timer publish_cloud_timer_{ nh_priv_.createTimer(cloud_pub_rate_, &SensorProcessor::publishCloud, this, false,
                                                        false) };  // oneshot: false, autostart: false

private:
  typename pcl::PointCloud<PointT>::Ptr inputcloud1_{ boost::make_shared<pcl::PointCloud<PointT>>() };
  typename pcl::PointCloud<PointT>::Ptr inputcloud2_{ boost::make_shared<pcl::PointCloud<PointT>>() };
  typename pcl::PointCloud<PointT>::Ptr inputcloud3_{ boost::make_shared<pcl::PointCloud<PointT>>() };
  typename pcl::PointCloud<PointT>::Ptr cloud_processed_{ boost::make_shared<pcl::PointCloud<PointT>>() };

  bool is_activated_;
};

#endif /* SENSOR_PROCESSOR_H */