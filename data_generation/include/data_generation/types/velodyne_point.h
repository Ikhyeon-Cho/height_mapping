/*
 * velodyne_point.h
 *
 *  Created on: Feb 10, 2025
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#pragma once

#include <pcl/point_types.h>

namespace data_generation_types {

struct EIGEN_ALIGN16 VelodynePoint {
  PCL_ADD_POINT4D; // This adds the members x,y,z
  float intensity;
  uint16_t ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

} // namespace data_generation_types

POINT_CLOUD_REGISTER_POINT_STRUCT(
    data_generation_types::VelodynePoint,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint16_t,
                                                                         ring,
                                                                         ring))
