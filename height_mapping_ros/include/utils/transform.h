/*
 * transform.h
 *
 *  Created on: Feb 10, 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef ROS_UTILS_TRANSFORM_H
#define ROS_UTILS_TRANSFORM_H

// ROS TF
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace utils
{
namespace tf
{
/// @brief
/// @tparam T Datatype
/// @param in The data to be transformed
/// @param target_frame Target frame to be transformed
/// @param source_frame Current frame of the data
/// @param out A reference to the transformed data
/// @return True if succeed to get transformed data. False otherwise
template <typename T>
static bool doTransform(const T& in, T& out, geometry_msgs::TransformStamped& transform)
{
  tf2::doTransform(in, out, transform);
  return true;
}

/// @brief
/// @param quaternion tf2 quaternion
/// @return roll, pitch, yaw angle [rad]
static std::tuple<double, double, double> getRPYFrom(const tf2::Quaternion& quaternion)
{
  double roll, pitch, yaw;
  tf2::Matrix3x3 matrix(quaternion);
  matrix.getEulerYPR(yaw, pitch, roll);
  return { roll, pitch, yaw };
}

/// @brief
/// @param quaternion geometry_msgs quaternion
/// @return roll, pitch, yaw angle [rad]
static std::tuple<double, double, double> getRPYFrom(const geometry_msgs::Quaternion& quaternion)
{
  double roll, pitch, yaw;
  tf2::Quaternion quaternion_tf;
  tf2::fromMsg(quaternion, quaternion_tf);
  return getRPYFrom(quaternion_tf);
}

/// @brief
/// @param roll roll angle [rad]
/// @param pitch pitch angle [rad]
/// @param yaw yaw angle [rad]
static tf2::Quaternion getQuaternionFrom(double roll, double pitch, double yaw)
{
  tf2::Quaternion quaternion;
  quaternion.setRPY(roll, pitch, yaw);
  return quaternion;
}

/// @brief
/// @param roll roll angle [rad]
/// @param pitch pitch angle [rad]
/// @param yaw yaw angle [rad]
static geometry_msgs::Quaternion getQuaternionMsgFrom(double roll, double pitch, double yaw)
{
  tf2::Quaternion quaternion_tf{ getQuaternionFrom(roll, pitch, yaw) };
  return tf2::toMsg(quaternion_tf);
}

/// @brief
/// @param transform geometry_msgs::Transform Type
/// @param transform_eigen Eigen Transform Matrix
static Eigen::Affine3d toAffine3d(const geometry_msgs::Transform& transform)
{
  auto transform_eigen = Eigen::Affine3d::Identity();

  // set translation
  Eigen::Vector3d translation;
  translation << transform.translation.x, transform.translation.y, transform.translation.z;
  transform_eigen.translation() = translation;

  // set rotation
  Eigen::Quaterniond rotation;
  rotation.x() = transform.rotation.x;
  rotation.y() = transform.rotation.y;
  rotation.z() = transform.rotation.z;
  rotation.w() = transform.rotation.w;
  transform_eigen.rotate(rotation);

  return transform_eigen;
}
}  // namespace tf
}  // namespace utils

#endif  // ROS_UTILS_TRANSFORM_H