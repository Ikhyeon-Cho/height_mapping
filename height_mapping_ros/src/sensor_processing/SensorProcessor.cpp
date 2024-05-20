/*
 * SensorProcessor.cpp
 *
 *  Created on: Aug 17, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "height_mapping/SensorProcessor.h"
#include <chrono>

template <typename PointT>
SensorProcessor<PointT>::SensorProcessor() : is_activated_{ false }
{
  // Drop the connection if empty input topic is provided (single sensor usage)
  if (inputcloud_topic_2_.empty())
    sub_cloud_2_.shutdown();
  if (inputcloud_topic_3_.empty())
    sub_cloud_3_.shutdown();
}

template <typename PointT>
void SensorProcessor<PointT>::cloudCallback1(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  if (!is_activated_)
  {
    is_activated_ = true;
    publish_cloud_timer_.start();
    std::cout << "\033[32m[HeightMapping::SensorProcessor]: Data received! Start processing...\033[0m" << std::endl;
  }

  inputcloud1_->clear();

  auto cloud = boost::make_shared<pcl::PointCloud<PointT>>();
  auto cloud_processed = boost::make_shared<pcl::PointCloud<PointT>>();
  pcl::fromROSMsg(*msg, *cloud);
  if (!processCloud(cloud, cloud_processed))
    return;

  inputcloud1_ = cloud_processed;
}

template <typename PointT>
void SensorProcessor<PointT>::cloudCallback2(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  inputcloud2_->clear();

  auto cloud = boost::make_shared<pcl::PointCloud<PointT>>();
  auto cloud_processed = boost::make_shared<pcl::PointCloud<PointT>>();
  pcl::fromROSMsg(*msg, *cloud);
  if (!processCloud(cloud, cloud_processed))
    return;

  inputcloud2_ = cloud_processed;
}

template <typename PointT>
void SensorProcessor<PointT>::cloudCallback3(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  inputcloud3_->clear();

  auto cloud = boost::make_shared<pcl::PointCloud<PointT>>();
  auto cloud_processed = boost::make_shared<pcl::PointCloud<PointT>>();
  pcl::fromROSMsg(*msg, *cloud);
  if (!processCloud(cloud, cloud_processed))
    return;

  inputcloud3_ = cloud_processed;
}

template <typename PointT>
void SensorProcessor<PointT>::publishCloud(const ros::TimerEvent& event)
{
  cloud_processed_ = inputcloud1_;
  *cloud_processed_ += *inputcloud2_;
  *cloud_processed_ += *inputcloud3_;

  cloud_processed_->header = inputcloud1_->header;
  cloud_processed_->header.frame_id = baselink_frame;

  if (cloud_processed_->empty())
    return;

  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(*cloud_processed_, msg);
  pub_cloud_.publish(msg);
}

template class SensorProcessor<pcl::PointXYZRGB>;
template class SensorProcessor<pcl::PointXYZI>;