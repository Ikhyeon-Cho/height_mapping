/*
 * pointcloud.h
 *
 *  Created on: Apr 22, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef ROS_UTILS_POINTCLOUD_H
#define ROS_UTILS_POINTCLOUD_H

// PCL
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>

// TF Transform
#include "height_mapping_utils/tf/transform.h"

template <typename T> using PointCloud = pcl::PointCloud<T>;
template <typename T> using PointCloudPtr = typename PointCloud<T>::Ptr;

namespace utils {
namespace pcl {
/// @brief The users should check whether the transform is succeeded or not
/// (nullptr)
/// @tparam T pcl point type
/// @param input The pointcloud to be transformed
/// @param target_frame The frame to which the data should be transformed
/// @param success True if succeed to get transform. False otherwise
/// @return A shared_ptr of the transformed data. If fails to get transform,
/// returns nullptr
namespace {
  const char* RED = "\033[31m";
  const char* RESET = "\033[0m";
}

template <typename T>
static PointCloudPtr<T>
transformPointcloud(const PointCloudPtr<T> &input,
                    const geometry_msgs::TransformStamped &transform_stamped) {

  std::string source_frame(input->header.frame_id);
  if (source_frame.empty()) {
    std::cout << RED 
              << "[height_mapping::utils::pcl] Warning: Transform failure - pointcloud has no frame id"
              << RESET << std::endl;
    return input;
  }

  if (input->empty()) {
    std::cout << RED 
              << "[height_mapping::utils::pcl] Warning: Transform failure - pointcloud is empty"
              << RESET << std::endl;
    return input;
  }

  // Skip transformation if the source and target frame are the same
  if (transform_stamped.child_frame_id == transform_stamped.header.frame_id) {
    return input;
  }

  auto transform_affine3d = utils::tf::toAffine3d(transform_stamped.transform);

  auto output = boost::make_shared<PointCloud<T>>();
  ::pcl::transformPointCloud(*input, *output, transform_affine3d);
  output->header = input->header;
  output->header.frame_id = transform_stamped.header.frame_id;
  return output;
}

template <typename T>
static PointCloudPtr<T>
filterPointcloudByField(const PointCloudPtr<T> &input, const std::string &field,
                        double field_min, double field_max,
                        bool negative = false) {
  if (input->empty())
    return input;

  PointCloudPtr<T> output = boost::make_shared<PointCloud<T>>();

  ::pcl::PassThrough<T> ps;
  ps.setInputCloud(input);
  ps.setFilterFieldName(field);
  ps.setFilterLimits(field_min, field_max);
  ps.setFilterLimitsNegative(negative);
  ps.filter(*output);
  output->header = input->header;
  return output;
}

template <typename T>
static PointCloudPtr<T> filterPointcloudByRange2D(const PointCloudPtr<T> &input,
                                                  double range_min,
                                                  double range_max) {
  if (input->empty())
    return input;

  PointCloudPtr<T> output = boost::make_shared<PointCloud<T>>();
  for (auto point : input->points) {
    double range_2D = sqrt(point.x * point.x + point.y * point.y);

    if (range_2D > range_min && range_2D < range_max) {
      T filtered_point = point;
      output->points.emplace_back(filtered_point);
    }
  }
  output->header = input->header;
  return output;
}

template <typename T>
static PointCloudPtr<T> filterPointcloudByRange(const PointCloudPtr<T> &input,
                                                double range_min,
                                                double range_max) {
  if (input->empty())
    return input;

  PointCloudPtr<T> output = boost::make_shared<PointCloud<T>>();
  for (auto point : input->points) {
    double range =
        sqrt(point.x * point.x + point.y * point.y + point.z * point.z);

    if (range > range_min && range < range_max) {
      T filtered_point = point;
      output->points.push_back(filtered_point);
    }
  }
  output->header = input->header;
  return output;
}

template <typename T>
static PointCloudPtr<T>
filterPointcloudByAngle(const PointCloudPtr<T> &input, double x_angle_start_deg,
                        double x_angle_end_deg, bool negative = false) {

  if (input->empty()) {
    return input;
  }

  // Normalize angles to [-180, 180] range
  auto normalizeAngle = [](double angle) {
    while (angle > 180.0)
      angle -= 360.0;
    while (angle < -180.0)
      angle += 360.0;
    return angle;
  };

  const double start_rad = DEG2RAD(normalizeAngle(x_angle_start_deg));
  const double end_rad = DEG2RAD(normalizeAngle(x_angle_end_deg));

  auto output = boost::make_shared<PointCloud<T>>();
  output->points.reserve(input->points.size() / 2);
  output->header = input->header;

  for (const auto &point : input->points) {
    // Get angle in [-π, π] range
    const double angle_horizon = std::atan2(point.y, point.x);

    bool keep_point;
    if (start_rad <= end_rad) {
      // Normal case: keep points within range
      keep_point = (angle_horizon >= start_rad && angle_horizon <= end_rad);
    } else {
      // Wraparound case (e.g., 135° to -135°)
      keep_point = (angle_horizon >= start_rad || angle_horizon <= end_rad);
    }

    if (negative) // Apply negative flag
      keep_point = !keep_point;

    if (keep_point) {
      output->points.emplace_back(point);
    }
  }

  return output;
}

template <typename T>
static PointCloudPtr<T> filterPointcloudByVoxel(const PointCloudPtr<T> &input,
                                                double voxel_x, double voxel_y,
                                                double voxel_z) {
  if (input->empty())
    return input;

  PointCloudPtr<T> output = boost::make_shared<PointCloud<T>>();
  ::pcl::VoxelGrid<T> vox;
  vox.setInputCloud(input);
  vox.setLeafSize(voxel_x, voxel_y, voxel_z);
  vox.filter(*output);
  output->header = input->header;
  return output;
}

template <typename T>
static PointCloudPtr<T> filterPointcloudByVoxel(const PointCloudPtr<T> &input,
                                                double voxel_size) {
  return filterPointcloudByVoxel<T>(input, voxel_size, voxel_size, voxel_size);
}

} // namespace pcl
}; // namespace utils

#endif // ROS_UTILS_POINTCLOUD_H