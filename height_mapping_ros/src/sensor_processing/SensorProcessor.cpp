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
SensorProcessor<PointT>::SensorProcessor()
{
}

template <typename PointT>
void SensorProcessor<PointT>::cloudCallback1(const sensor_msgs::PointCloud2ConstPtr& msg)
{
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

  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(*cloud_processed_, msg);
  msg.header.stamp = ros::Time::now();
  pub_cloud_.publish(msg);
}

template class SensorProcessor<pcl::PointXYZRGB>;
template class SensorProcessor<pcl::PointXYZI>;