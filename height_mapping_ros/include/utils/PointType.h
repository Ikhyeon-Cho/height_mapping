#ifndef POINT_TYPE_H
#define POINT_TYPE_H

#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/impl/voxel_grid.hpp>

struct PointXYZRGBI
{
  PCL_ADD_POINT4D;
  PCL_ADD_RGB;
  float intensity;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZRGBI,
                                  (float, x, x)(float, y, y)(float, z, z)(uint32_t, rgb, rgb)(float, intensity,
                                                                                              intensity))

#endif /* POINT_TYPE_H */