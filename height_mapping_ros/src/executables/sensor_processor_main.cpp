/*
 * sensor_processor_node.cpp
 *
 *  Created on: Apr 29, 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include <ros/ros.h>
#include "height_mapping_ros/SensorProcessor.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sensor_processor");
  ros::NodeHandle nh{ "sensor_processor" };

  std::string point_type{ nh.param<std::string>("pointType", "laser") };  // By default, laser pointcloud is used
  if (point_type == "color")
  {
    nh.setParam("outputTopic", "/height_mapping/sensor/color/points");
    std::cout << "\033[1;32m[HeightMapping::SensorProcessor]: Waiting for RGB pointcloud...\033[0m" << std::endl;

    SensorProcessor<pcl::PointXYZRGB> node;
    ros::MultiThreadedSpinner spinner(3);
    spinner.spin();
  }
  else if (point_type == "laser")
  {
    nh.setParam("outputTopic", "/height_mapping/sensor/laser/points");
    std::cout << "\033[1;32m[HeightMapping::SensorProcessor]: Waiting for Laser pointcloud...\033[0m" << std::endl;

    SensorProcessor<pcl::PointXYZI> node;
    ros::MultiThreadedSpinner spinner(3);
    spinner.spin();
  }
  else
  {
    std::cout << "\033[33m[HeightMapping::SensorProcessor]: Invalid Point Type!! Supported point types: color, "
                 "laser\033[0m"
              << std::endl;
    return -1;
  }

  return 0;
}